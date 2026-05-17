// TFT_Display.cpp
// Full touch UI for RNode Firmware - Heltec WiFi LoRa 32 V4 Expansion Kit

#include "TFT_Display.h"
#include <WiFi.h>

// ── Firmware display variables ──────────────────────────────────────────────
extern bool display_blanked;
extern bool display_blanking_enabled;

// ── Firmware function externs ───────────────────────────────────────────────
void display_unblank();
void sleep_now();

// ── Firmware battery variables ───────────────────────────────────────────────
extern bool    battery_ready;
extern float   battery_voltage;
extern float   battery_percent;
extern uint8_t battery_state;
#define BATTERY_STATE_UNKNOWN     0x00
#define BATTERY_STATE_DISCHARGING 0x01
#define BATTERY_STATE_CHARGING    0x02
#define BATTERY_STATE_CHARGED     0x03

// ── Firmware BT variables ────────────────────────────────────────────────────
#define BT_STATE_OFF       0x00
#define BT_STATE_PAIRING   0x02
#define BT_STATE_CONNECTED 0x03
extern uint8_t dev_bt_mac[6];
extern char     bt_devname[11];
extern bool     bt_ready;
extern bool     bt_enabled;
extern uint8_t  bt_state;
extern bool wifi_host_is_connected();
extern bool     bt_allow_pairing;
extern uint32_t bt_ssp_pin;
extern uint32_t bt_pairing_started;
bool bt_init();
void bt_start();
void bt_stop();
void bt_conf_save(bool is_enabled);
// WiFi externs/forward decls
extern uint8_t wifi_mode;
extern uint8_t wr_state;
void wifi_remote_init();
void wifi_remote_stop();
void wr_conf_save(uint8_t mode);
// Mirror constants from Config.h (not directly included here)
#ifndef WR_WIFI_OFF
  #define WR_WIFI_OFF        0x00
  #define WR_WIFI_STA        0x01
  #define WR_WIFI_AP         0x02
  #define WR_STATE_OFF       0x00
  #define WR_STATE_ON        0x01
  #define WR_STATE_CONNECTED 0x02
#endif
#ifndef MAJ_VERS
  #define MAJ_VERS  0x01
  #define MIN_VERS  0x56
#endif
void bt_enable_pairing();
void bt_disable_pairing();


// ── Driver ───────────────────────────────────────────────────────────────────
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_PIN_CS, TFT_PIN_DC, TFT_PIN_MOSI,
                                       TFT_PIN_SCLK, TFT_PIN_RST);

// ── CHSC6X touch (direct I2C, no library needed) ─────────────────────────────
#define CHSC6X_ADDR  0x2E
static bool _touch_read(uint16_t *x, uint16_t *y) {
    uint8_t b[8] = {0};
    Wire1.beginTransmission(CHSC6X_ADDR);
    Wire1.write(0x00);
    Wire1.endTransmission(false);
    delay(2);
    Wire1.requestFrom((uint8_t)CHSC6X_ADDR, (uint8_t)8);
    int idx = 0;
    while (Wire1.available() && idx < 8) { b[idx++] = Wire1.read(); }
    if (idx < 7) return false;
    if (b[2] == 0) return false;  // b[2]=01 means touched
    *x = ((uint16_t)(b[3] & 0x0F) << 8) | b[4];
    *y = ((uint16_t)(b[5] & 0x0F) << 8) | b[6];
    if (*x == 0 && *y == 0) return false;
    return true;
}
static void _touch_hw_init() {
    pinMode(TOUCH_PIN_RST, OUTPUT);
    digitalWrite(TOUCH_PIN_RST, LOW); delay(10);
    digitalWrite(TOUCH_PIN_RST, HIGH); delay(50);
    pinMode(TOUCH_PIN_INT, INPUT);
    Wire1.begin(TOUCH_PIN_SDA, TOUCH_PIN_SCL);

}

// ── State ─────────────────────────────────────────────────────────────────────
static TabID    _tab           = TAB_HOME;
static bool     _needs_redraw  = true;
static uint32_t _boot_ms       = 0;
static uint32_t _last_touch_ms = 0;
static uint32_t _last_refresh  = 0;
static bool     _splash_done   = false;

// BT state
static BTState  _bt_state      = BT_IDLE;
static uint32_t _bt_pair_start = 0;
static char     _bt_pin[7]     = "------";
static bool     _bt_connected  = false;

// Cached values
static float    _freq   = 0; static uint8_t  _sf   = 0;
static uint32_t _bw     = 0; static int8_t   _cr   = 0;
static int16_t  _rssi   = 0; static float    _snr  = 0;
static uint32_t _rx     = 0; static uint32_t _tx   = 0;
static bool     _airlock= false; static bool _rx_ongoing = false;
static uint8_t  _txp    = 0;     static bool     _host_connected = false;
static uint32_t _last_rx_count_seen = 0;
static uint32_t _last_rx_time       = 0;
static uint32_t _last_tx_count_seen = 0;
static uint32_t _last_tx_time       = 0;
// RSSI sparkline ring buffer (64 samples, ~1 sample/sec)
#define RSSI_HIST_LEN 64
static int16_t  _rssi_hist[RSSI_HIST_LEN];
static uint8_t  _rssi_hist_head     = 0;
static bool     _rssi_hist_init     = false;
static uint32_t _last_rssi_sample   = 0;

// Touch debounce
#define TOUCH_DEBOUNCE  150
#define SPLASH_MS       500
#define BT_PIN_TIMEOUT  60000  // 60 seconds to pair

// ── Power / init ──────────────────────────────────────────────────────────────
void tft_backlight(bool on) {
    pinMode(TFT_PIN_BLK, OUTPUT);
    digitalWrite(TFT_PIN_BLK, on ? HIGH : LOW);
}

void tft_invalidate() { _needs_redraw = true; }

void tft_init() {
    _boot_ms = millis();
    pinMode(VEXT_PIN, OUTPUT);
    digitalWrite(VEXT_PIN, LOW);
    delay(80);
    tft.init(TFT_WIDTH, TFT_HEIGHT);
    tft.setRotation(2);
    tft.invertDisplay(true);
    tft.fillScreen(C_BG);
    tft_backlight(true);
    _touch_hw_init();
    _draw_splash();
}

// ── Drawing primitives ────────────────────────────────────────────────────────

// Rounded rectangle filled card
static void _card(int16_t x, int16_t y, int16_t w, int16_t h,
                  uint16_t col, uint16_t border = 0) {
    tft.fillRoundRect(x, y, w, h, CARD_RADIUS, col);
    if (border) tft.drawRoundRect(x, y, w, h, CARD_RADIUS, border);
}

// Label on left, value on right, inside a row area
static void _row(int16_t y, const char* label, const char* value,
                 uint16_t vcol = C_ACCENT, uint8_t vsize = 2) {
    tft.fillRect(CARD_PAD, y, TFT_WIDTH - CARD_PAD*2, CARD_H - 2, C_CARD);
    tft.drawFastHLine(CARD_PAD, y + CARD_H - 2, TFT_WIDTH - CARD_PAD*2, C_DIVIDER);
    tft.setTextColor(C_TEXT_DIM); tft.setTextSize(1);
    tft.setCursor(CARD_PAD + 4, y + 6);
    tft.print(label);
    tft.setTextColor(vcol); tft.setTextSize(vsize);
    int16_t vw = strlen(value) * (vsize == 2 ? 12 : 6);
    tft.setCursor(TFT_WIDTH - CARD_PAD - 4 - vw, y + (vsize == 2 ? 16 : 20));
    tft.print(value);
}

// Centred text in a button
static void _btn(int16_t x, int16_t y, int16_t w, int16_t h,
                 const char* label, uint16_t bg, uint16_t fg, uint8_t sz = 2) {
    _card(x, y, w, h, bg, fg);
    tft.setTextColor(fg); tft.setTextSize(sz);
    int16_t tw = strlen(label) * (sz == 2 ? 12 : 6);
    tft.setCursor(x + (w - tw) / 2, y + (h - sz*8) / 2);
    tft.print(label);
}

// Header bar
static void _header(const char* title, uint16_t dot_col = 0) {
    tft.fillRect(0, 0, TFT_WIDTH, HEADER_H, C_HEADER);
    tft.setTextColor(C_TEXT); tft.setTextSize(2);
    tft.setCursor(12, (HEADER_H - 16) / 2);
    tft.print(title);
    // Battery indicator top right
    if (battery_ready) {
        uint16_t bc = (battery_percent > 50) ? C_GREEN : (battery_percent > 20) ? C_YELLOW : C_RED;
        // Battery icon outline
        int16_t bx = TFT_WIDTH - 68, by = 8, bw = 24, bh = 14;
        tft.drawRect(bx, by, bw, bh, C_TEXT_DIM);
        tft.fillRect(bx + bw, by + 4, 3, 6, C_TEXT_DIM); // terminal
        // Fill level
        int16_t fill = (int16_t)((battery_percent / 100.0f) * (bw - 2));
        if (fill > 0) tft.fillRect(bx + 1, by + 1, fill, bh - 2, bc);
        // Charging bolt
        if (battery_state == BATTERY_STATE_CHARGING) {
            tft.setTextColor(C_YELLOW); tft.setTextSize(1);
            tft.setCursor(bx + 8, by + 3); tft.print("+");
        }
        // Percentage text
        tft.setTextColor(bc); tft.setTextSize(1);
        char pbuf[6];
        snprintf(pbuf, sizeof(pbuf), "%d%%", (int)battery_percent);
        tft.setCursor(TFT_WIDTH - 68, by + bh + 2);
        tft.print(pbuf);
    }
    if (dot_col) {
        tft.fillCircle(TFT_WIDTH - 16, HEADER_H / 2, 7, dot_col);
    }
    tft.drawFastHLine(0, HEADER_H, TFT_WIDTH, C_ACCENT);
}

// Bottom navigation bar with 4 tabs
static void _navbar() {
    tft.fillRect(0, NAVBAR_Y, TFT_WIDTH, NAVBAR_H, C_NAVBG);
    tft.drawFastHLine(0, NAVBAR_Y, TFT_WIDTH, C_DIVIDER);

    const char* labels[] = {"HOME", "RADIO", "BT", "TOOLS"};
    int16_t tab_w = TFT_WIDTH / TAB_COUNT;

    for (int i = 0; i < TAB_COUNT; i++) {
        int16_t tx = i * tab_w;
        bool sel = (_tab == (TabID)i);

        if (sel) {
            tft.fillRect(tx, NAVBAR_Y, tab_w, NAVBAR_H, C_CARD);
            tft.drawFastHLine(tx + 4, NAVBAR_Y, tab_w - 8, C_NAVSEL);
            tft.setTextColor(C_NAVSEL);
        } else {
            tft.setTextColor(C_TEXT_DIM);
        }

        // Draw icon
        int16_t cx = tx + tab_w / 2;
        int16_t iy = NAVBAR_Y + 8;

        switch (i) {
            case TAB_HOME:
                // House icon
                tft.drawTriangle(cx-10,iy+10, cx,iy, cx+10,iy+10, sel?C_NAVSEL:C_TEXT_DIM);
                tft.drawRect(cx-7, iy+10, 14, 10, sel?C_NAVSEL:C_TEXT_DIM);
                tft.fillRect(cx-3, iy+14, 6, 6, sel?C_NAVSEL:C_TEXT_DIM);
                break;
            case TAB_RADIO:
                // Radio waves
                tft.drawCircle(cx, iy+12, 4, sel?C_NAVSEL:C_TEXT_DIM);
                tft.drawCircle(cx, iy+12, 8, sel?C_NAVSEL:C_TEXT_DIM);
                tft.drawCircle(cx, iy+12, 12, sel?C_NAVSEL:C_TEXT_DIM);
                break;
            case TAB_BT:
                // BT symbol (simplified)
                tft.drawLine(cx, iy, cx, iy+20, sel?C_NAVSEL:C_TEXT_DIM);
                tft.drawLine(cx, iy, cx+8, iy+6, sel?C_NAVSEL:C_TEXT_DIM);
                tft.drawLine(cx+8, iy+6, cx-6, iy+14, sel?C_NAVSEL:C_TEXT_DIM);
                tft.drawLine(cx-6, iy+14, cx+8, iy+20, sel?C_NAVSEL:C_TEXT_DIM);
                break;
            case TAB_STATS: {
                // Sliders icon: 3 horizontal lines with offset filled knobs
                uint16_t col = sel?C_NAVSEL:C_TEXT_DIM;
                tft.drawFastHLine(cx-10, iy+4,  20, col);
                tft.drawFastHLine(cx-10, iy+12, 20, col);
                tft.drawFastHLine(cx-10, iy+20, 20, col);
                tft.fillCircle(cx-4, iy+4,  2, col);
                tft.fillCircle(cx+6, iy+12, 2, col);
                tft.fillCircle(cx-1, iy+20, 2, col);
                break;
            }
        }

        // Label
        tft.setTextSize(1);
        int16_t lw = strlen(labels[i]) * 6;
        tft.setCursor(tx + (tab_w - lw) / 2, NAVBAR_Y + 34);
        tft.print(labels[i]);
    }
}

// ── Splash screen ─────────────────────────────────────────────────────────────
void _draw_splash() {
    tft.fillScreen(C_BG);

    tft.drawFastHLine(20, 88, TFT_WIDTH - 40, C_ACCENT);
    tft.drawFastHLine(20, 90, TFT_WIDTH - 40, C_HEADER);

    tft.setTextColor(C_ACCENT); tft.setTextSize(4);
    tft.setCursor(28, 100);
    tft.print("RNode");

    tft.setTextColor(C_TEXT); tft.setTextSize(2);
    tft.setCursor(14, 148);
    tft.print("Heltec V4 + PA");

    tft.drawFastHLine(20, 175, TFT_WIDTH - 40, C_HEADER);
    tft.drawFastHLine(20, 177, TFT_WIDTH - 40, C_ACCENT);

    tft.setTextColor(C_TEXT_DIM); tft.setTextSize(1);
    tft.setCursor(38, 190);
    tft.print("Reticulum Network Stack");
    tft.setCursor(62, 208);
    tft.print("Touch to continue");

    // Animated dots hint
    for (int i = 0; i < 3; i++) {
        tft.fillCircle(100 + i*20, 240, 4, C_ACCENT);
    }
}

// ── HOME tab ──────────────────────────────────────────────────────────────────
// Paint just the sparkline card on HOME (avoids full screen redraw on each sample).
// Card is at y = CONTENT_Y + 4 + (CARD_H + 3) * 2 (3rd card after CONNECTION + RSSI/SNR).
static void _paint_radio_state() {
    // Card position: 3rd card (after CONNECTION + RSSI/SNR)
    int16_t y = CONTENT_Y + 4 + (CARD_H + 3) * 2;
    _card(CARD_PAD, y, TFT_WIDTH - CARD_PAD*2, CARD_H, C_CARD, C_DIVIDER);
    tft.setTextColor(C_TEXT_DIM); tft.setTextSize(1);
    tft.setCursor(CARD_PAD + 8, y + 6); tft.print("RADIO STATE");
    const char* label;
    uint16_t color;
    if (_airlock) {
        label = "LOCKED";
        color = C_RED;
    } else if (_rx_ongoing) {
        label = "RECEIVING";
        color = C_YELLOW;
    } else {
        label = "IDLE";
        color = C_TEXT_DIM;
    }
    tft.setTextColor(color); tft.setTextSize(3);
    int16_t lw = strlen(label) * 18;  // TextSize 3 = ~18px wide per char
    tft.setCursor((TFT_WIDTH - lw) / 2, y + 22);
    tft.print(label);
}

// ── HOME tab ──────────────────────────────────────────────────────────────────
// Paint just the sparkline card on HOME (avoids full screen redraw on each sample).
// Card is at y = CONTENT_Y + 4 + (CARD_H + 3) * 2 (3rd card after CONNECTION + RSSI/SNR).
static void _paint_sparkline() {
    int16_t y = CONTENT_Y + 4 + (CARD_H + 3) * 2;
    // Redraw the whole card (background, label, value, bars)
    _card(CARD_PAD, y, TFT_WIDTH - CARD_PAD*2, CARD_H, C_CARD, C_DIVIDER);
    tft.setTextColor(C_TEXT_DIM); tft.setTextSize(1);
    tft.setCursor(CARD_PAD + 8, y + 4); tft.print("RSSI HISTORY");
    char vbuf[16];
    if (_rssi == -292) snprintf(vbuf, sizeof(vbuf), "--- dBm");
    else               snprintf(vbuf, sizeof(vbuf), "%d dBm", _rssi);
    int16_t vw = strlen(vbuf) * 6;
    uint16_t vc = (_rssi == -292) ? C_TEXT_DIM
                : (_rssi > -80)   ? C_GREEN
                : (_rssi > -100)  ? C_YELLOW : C_RED;
    tft.setTextColor(vc);
    tft.setCursor(TFT_WIDTH - CARD_PAD - 8 - vw, y + 4); tft.print(vbuf);
    const int16_t bar_w   = 3;
    const int16_t bar_top = y + 16;
    const int16_t bar_max = CARD_H - 16 - 4;
    const int16_t bars_total_w = RSSI_HIST_LEN * bar_w;
    const int16_t bars_x0 = (TFT_WIDTH - bars_total_w) / 2;
    for (int i = 0; i < RSSI_HIST_LEN; i++) {
        uint8_t idx = (_rssi_hist_head + i) % RSSI_HIST_LEN;
        int16_t v = _rssi_hist[idx];
        int16_t bx = bars_x0 + i * bar_w;
        if (v == -292) continue;
        int16_t mapped = v + 120;
        if (mapped < 0) mapped = 0;
        if (mapped > 90) mapped = 90;
        int16_t bh = (mapped * bar_max) / 90;
        uint16_t bc = (v > -80) ? C_GREEN : (v > -100) ? C_YELLOW : C_RED;
        tft.fillRect(bx, bar_top + bar_max - bh, bar_w - 1, bh, bc);
    }
}
static void _draw_home() {
    tft.fillRect(0, CONTENT_Y, TFT_WIDTH, CONTENT_H, C_BG);

    bool wifi_conn = (wifi_mode != WR_WIFI_OFF) && (wr_state == WR_STATE_CONNECTED);
    bool any_conn = _host_connected || _bt_connected || wifi_conn;
    uint16_t dot = any_conn ? C_GREEN :
                   (_bt_state == BT_STATE_PAIRING) ? C_YELLOW : C_TEXT_DIM;
    _header("RNode Status", dot);

    char buf[32];
    int16_t y = CONTENT_Y + 4;

    // CONNECTION card - real connection type
    _card(CARD_PAD, y, TFT_WIDTH - CARD_PAD*2, CARD_H, C_CARD, C_DIVIDER);
    tft.setTextColor(C_TEXT_DIM); tft.setTextSize(1);
    tft.setCursor(CARD_PAD + 8, y + 6); tft.print("CONNECTION");
    tft.setTextSize(2);
    if (_host_connected) {
        tft.setTextColor(C_GREEN);
        tft.setCursor(CARD_PAD + 8, y + 18); tft.print("USB Host");
    } else if (_bt_connected) {
        tft.setTextColor(C_GREEN);
        tft.setCursor(CARD_PAD + 8, y + 18); tft.print("Bluetooth");
    } else if (wifi_conn) {
        tft.setTextColor(C_GREEN);
        tft.setCursor(CARD_PAD + 8, y + 18); tft.print("Network");
    } else if (_bt_state == BT_STATE_PAIRING) {
        tft.setTextColor(C_YELLOW);
        tft.setCursor(CARD_PAD + 8, y + 18); tft.print("Pairing...");
    } else {
        tft.setTextColor(C_TEXT_DIM);
        tft.setCursor(CARD_PAD + 8, y + 18); tft.print("Disconnected");
    }
    y += CARD_H + 3;

    // RSSI / SNR card (split)
    _card(CARD_PAD, y, TFT_WIDTH - CARD_PAD*2, CARD_H, C_CARD, C_DIVIDER);
    tft.setTextColor(C_TEXT_DIM); tft.setTextSize(1);
    tft.setCursor(CARD_PAD + 8, y + 6); tft.print("RSSI");
    uint16_t rc = (_rssi > -80) ? C_GREEN : (_rssi > -100) ? C_YELLOW : C_RED;
    tft.setTextColor(_rssi == -292 ? C_TEXT_DIM : rc); tft.setTextSize(2);
    if (_rssi == -292) snprintf(buf, sizeof(buf), "No signal");
    else snprintf(buf, sizeof(buf), "%ddBm", _rssi);
    tft.setCursor(CARD_PAD + 8, y + 18); tft.print(buf);
    tft.setTextColor(C_TEXT_DIM); tft.setTextSize(1);
    tft.setCursor(TFT_WIDTH/2 + 4, y + 6); tft.print("SNR");
    uint16_t sc = (_snr >= 0) ? C_GREEN : (_snr >= -5) ? C_YELLOW : C_RED;
    tft.setTextColor(sc); tft.setTextSize(2);
    snprintf(buf, sizeof(buf), "%.1fdB", _snr);
    tft.setCursor(TFT_WIDTH/2 + 4, y + 18); tft.print(buf);
    y += CARD_H + 3;

    // RADIO STATE card (drawn by helper)
    _paint_radio_state();
    y += CARD_H + 3;

    // Airtime lock warning
    if (_airlock) {
        _card(CARD_PAD, y, TFT_WIDTH - CARD_PAD*2, 30, C_RED, 0);
        tft.setTextColor(C_TEXT); tft.setTextSize(1);
        tft.setCursor(TFT_WIDTH/2 - 42, y + 10);
        tft.print("! AIRTIME LOCK ACTIVE !");
    }
}

// ── RADIO tab ─────────────────────────────────────────────────────────────────
// 6 cards at 34px each fit into CONTENT_H = 222 with room to spare.
#define RADIO_CARD_H 34
#define BT_CARD_H    34
static void _draw_radio() {
    tft.fillRect(0, CONTENT_Y, TFT_WIDTH, CONTENT_H, C_BG);
    _header("Radio Config", _airlock ? C_RED : _rx_ongoing ? C_GREEN : C_TEXT_DIM);

    char buf[32];
    int16_t y = CONTENT_Y + 2;
    const int16_t cw = TFT_WIDTH - CARD_PAD*2;

    // Helper-free; copy-paste pattern keeps the original style.

    // Frequency
    _card(CARD_PAD, y, cw, RADIO_CARD_H, C_CARD, C_DIVIDER);
    tft.setTextColor(C_TEXT_DIM); tft.setTextSize(1);
    tft.setCursor(CARD_PAD + 8, y + 4); tft.print("FREQUENCY");
    tft.setTextColor(C_ACCENT); tft.setTextSize(2);
    snprintf(buf, sizeof(buf), "%.4f MHz", _freq);
    tft.setCursor(CARD_PAD + 8, y + 16); tft.print(buf);
    y += RADIO_CARD_H + 2;

    // Spreading factor
    _card(CARD_PAD, y, cw, RADIO_CARD_H, C_CARD, C_DIVIDER);
    tft.setTextColor(C_TEXT_DIM); tft.setTextSize(1);
    tft.setCursor(CARD_PAD + 8, y + 4); tft.print("SPREADING FACTOR");
    tft.setTextColor(C_ACCENT); tft.setTextSize(2);
    snprintf(buf, sizeof(buf), "SF%u", _sf);
    tft.setCursor(CARD_PAD + 8, y + 16); tft.print(buf);
    y += RADIO_CARD_H + 2;

    // Bandwidth
    _card(CARD_PAD, y, cw, RADIO_CARD_H, C_CARD, C_DIVIDER);
    tft.setTextColor(C_TEXT_DIM); tft.setTextSize(1);
    tft.setCursor(CARD_PAD + 8, y + 4); tft.print("BANDWIDTH");
    tft.setTextColor(C_ACCENT); tft.setTextSize(2);
    snprintf(buf, sizeof(buf), "%lu kHz", _bw / 1000UL);
    tft.setCursor(CARD_PAD + 8, y + 16); tft.print(buf);
    y += RADIO_CARD_H + 2;

    // Coding rate
    _card(CARD_PAD, y, cw, RADIO_CARD_H, C_CARD, C_DIVIDER);
    tft.setTextColor(C_TEXT_DIM); tft.setTextSize(1);
    tft.setCursor(CARD_PAD + 8, y + 4); tft.print("CODING RATE");
    tft.setTextColor(C_ACCENT); tft.setTextSize(2);
    snprintf(buf, sizeof(buf), "4/%d", _cr);
    tft.setCursor(CARD_PAD + 8, y + 16); tft.print(buf);
    y += RADIO_CARD_H + 2;

    // TX Power
    _card(CARD_PAD, y, cw, RADIO_CARD_H, C_CARD, C_DIVIDER);
    tft.setTextColor(C_TEXT_DIM); tft.setTextSize(1);
    tft.setCursor(CARD_PAD + 8, y + 4); tft.print("TX POWER");
    tft.setTextColor(C_ACCENT); tft.setTextSize(2);
    if (_txp == 0xFF) {
        tft.setCursor(CARD_PAD + 8, y + 16); tft.print("-- dBm");
    } else {
        snprintf(buf, sizeof(buf), "%u dBm", _txp);
        tft.setCursor(CARD_PAD + 8, y + 16); tft.print(buf);
    }
    y += RADIO_CARD_H + 2;

    // Host connection status
    uint16_t status_color = _host_connected ? C_GREEN : C_RED;  // text color only
    _card(CARD_PAD, y, cw, RADIO_CARD_H, C_CARD, C_DIVIDER);    // neutral border
    tft.setTextColor(C_TEXT_DIM); tft.setTextSize(1);
    tft.setCursor(CARD_PAD + 8, y + 4); tft.print("CONNECTED TO");
    tft.setTextColor(status_color); tft.setTextSize(2);
    tft.setCursor(CARD_PAD + 8, y + 16);
    {
        const char* link;
        uint16_t link_color;
        if (bt_state == BT_STATE_PAIRING) {
            link = "Pairing"; link_color = C_YELLOW;
        } else if (bt_state == BT_STATE_CONNECTED) {
            link = "Bluetooth"; link_color = C_GREEN;
        } else if (_host_connected) {
            link = "USB Host"; link_color = C_GREEN;
        } else if (wifi_host_is_connected()) {
            link = "Network"; link_color = C_GREEN;
        } else {
            link = "Disconnected"; link_color = C_RED;
        }
        tft.setTextColor(link_color);
        tft.print(link);
    }
}

// ── BT tab ────────────────────────────────────────────────────────────────────
static void _draw_bt() {
    tft.fillRect(0, CONTENT_Y, TFT_WIDTH, CONTENT_H, C_BG);

    bool fw_connected = (bt_state == BT_STATE_CONNECTED);
    bool fw_pairing   = (bt_state == BT_STATE_PAIRING);

    uint16_t dot = fw_connected ? C_GREEN : fw_pairing ? C_YELLOW : C_RED;
    _header("Bluetooth", dot);

    int16_t y = CONTENT_Y + 8;

    // Device name card
    _card(CARD_PAD, y, TFT_WIDTH - CARD_PAD*2, BT_CARD_H, C_CARD, C_DIVIDER);
    tft.setTextColor(C_TEXT_DIM); tft.setTextSize(1);
    tft.setCursor(CARD_PAD + 8, y + 4); tft.print("DEVICE NAME");
    tft.setTextColor(C_ACCENT); tft.setTextSize(2);
    tft.setCursor(CARD_PAD + 8, y + 16);
    if (bt_ready && strlen(bt_devname) > 0) {
        tft.print(bt_devname);
    } else {
        tft.print("Not ready");
    }
    y += BT_CARD_H + 4;

    // MAC address card (hidden when busy with pairing or connected)
    {  // MAC always shown
        _card(CARD_PAD, y, TFT_WIDTH - CARD_PAD*2, BT_CARD_H, C_CARD, C_DIVIDER);
        tft.setTextColor(C_TEXT_DIM); tft.setTextSize(1);
        tft.setCursor(CARD_PAD + 8, y + 4); tft.print("MAC ADDRESS");
        tft.setTextColor(C_ACCENT); tft.setTextSize(2);
        tft.setCursor(CARD_PAD + 8, y + 16);
        if (bt_ready) {
            char macbuf[20];
            snprintf(macbuf, sizeof(macbuf), "%02X:%02X:%02X:%02X:%02X:%02X",
                dev_bt_mac[0], dev_bt_mac[1], dev_bt_mac[2],
                dev_bt_mac[3], dev_bt_mac[4], dev_bt_mac[5]);
            tft.print(macbuf);
        } else {
            tft.print("--:--:--:--:--:--");
        }
        y += BT_CARD_H + 4;
    }

    // Status card
    _card(CARD_PAD, y, TFT_WIDTH - CARD_PAD*2, BT_CARD_H, C_CARD,
          fw_connected ? C_GREEN : fw_pairing ? C_YELLOW : C_DIVIDER);
    tft.setTextColor(C_TEXT_DIM); tft.setTextSize(1);
    tft.setCursor(CARD_PAD + 8, y + 4); tft.print("STATUS");
    tft.setTextSize(2);
    if (fw_connected) {
        tft.setTextColor(C_GREEN);
        tft.setCursor(CARD_PAD + 8, y + 16); tft.print("Connected");
    } else if (fw_pairing) {
        tft.setTextColor(C_YELLOW);
        uint32_t now_ms = millis();
        uint32_t elapsed = (now_ms > bt_pairing_started) ? now_ms - bt_pairing_started : 0;
        uint32_t remaining = (elapsed < 35000UL) ? (35000UL - elapsed) / 1000 : 0;
        char tbuf[20];
        snprintf(tbuf, sizeof(tbuf), "Pairing %lus", remaining);
        tft.setCursor(CARD_PAD + 8, y + 16); tft.print(tbuf);
    } else {
        tft.setTextColor(C_TEXT_DIM);
        tft.setCursor(CARD_PAD + 8, y + 16); tft.print("Not connected");
    }
    y += BT_CARD_H + 4;

    // Action buttons
    if (!fw_connected && !fw_pairing) {
        _btn(CARD_PAD, y, TFT_WIDTH - CARD_PAD*2, BT_CARD_H,
             "START PAIRING", C_BTN, C_ACCENT, 2);
        y += BT_CARD_H + 4;
        tft.setTextColor(C_TEXT_DIM); tft.setTextSize(1);
        tft.setCursor(CARD_PAD + 4, y + 4);
        tft.print("Tap to make discoverable.");
        tft.setCursor(CARD_PAD + 4, y + 16);
        tft.print("Connect from any Reticulum app.");
    } else if (fw_pairing) {
        _btn(CARD_PAD, y, TFT_WIDTH - CARD_PAD*2, BT_CARD_H,
             "CANCEL PAIRING", C_CARD, C_RED, 2);
        y += BT_CARD_H + 4;
        // PIN card
        _card(CARD_PAD, y, TFT_WIDTH - CARD_PAD*2, BT_CARD_H, C_CARD, C_YELLOW);
        tft.setTextColor(C_TEXT_DIM); tft.setTextSize(1);
        tft.setCursor(CARD_PAD + 8, y + 4);
        tft.print("PAIRING PIN");
        if (bt_ssp_pin != 0) {
            tft.setTextColor(C_YELLOW); tft.setTextSize(2);
            char pinbuf[10];
            snprintf(pinbuf, sizeof(pinbuf), "%06lu", bt_ssp_pin);
            // Add spaces between digits for readability: "876661" -> "8 7 6 6 6 1"
            char spaced[16] = {0};
            int sl = 0;
            for (int i = 0; pinbuf[i] && sl < 14; i++) {
                spaced[sl++] = pinbuf[i];
                if (pinbuf[i+1]) spaced[sl++] = ' ';
            }
            int16_t pw = sl * 12;
            tft.setCursor((TFT_WIDTH - pw) / 2, y + 16);
            tft.print(spaced);
        } else {
            tft.setTextColor(C_TEXT_DIM); tft.setTextSize(2);
            tft.setCursor(CARD_PAD + 8, y + 16);
            tft.print("Waiting...");
        }
        y += BT_CARD_H + 4;
        tft.setTextColor(C_TEXT_DIM); tft.setTextSize(1);
        tft.setCursor(CARD_PAD + 4, y);
        tft.print("Scan for RNode C6D9 and pair");
        tft.setCursor(CARD_PAD + 4, y + 14);
        tft.print("from your device BT settings.");
    } else if (fw_connected) {
        // Connected info card
        _card(CARD_PAD, y, TFT_WIDTH - CARD_PAD*2, BT_CARD_H, C_CARD, C_GREEN);
        tft.setTextColor(C_TEXT_DIM); tft.setTextSize(1);
        tft.setCursor(CARD_PAD + 8, y + 4);
        tft.print("CONNECTED DEVICE");
        tft.setTextColor(C_GREEN); tft.setTextSize(2);
        tft.setCursor(CARD_PAD + 8, y + 16);
        tft.print("BLE Client");
        y += BT_CARD_H + 4;
        _btn(CARD_PAD, y, TFT_WIDTH - CARD_PAD*2, BT_CARD_H,
             "DISCONNECT", C_CARD, C_RED, 2);
        y += BT_CARD_H + 4;
        tft.setTextColor(C_TEXT_DIM); tft.setTextSize(1);
        tft.setCursor(CARD_PAD + 4, y + 4);
        tft.print("Auto-reconnects when in range.");
        tft.setCursor(CARD_PAD + 4, y + 16);
        tft.print("No re-pairing needed.");
    }
}

// ── STATS tab ─────────────────────────────────────────────────────────────────
static void _draw_stats() {
    tft.fillRect(0, CONTENT_Y, TFT_WIDTH, CONTENT_H, C_BG);
    _header("Tools", 0);
    char buf[32];
    const int16_t SH = 40;
    const int16_t SG = 3;
    int16_t y = CONTENT_Y + 4;

    // Uptime card (single-row, compact)
    const int16_t UPT_H = 20;
    uint32_t uptime_s = (millis() - _boot_ms) / 1000UL;
    _card(CARD_PAD, y, TFT_WIDTH - CARD_PAD*2, UPT_H, C_CARD, C_DIVIDER);
    tft.setTextColor(C_TEXT_DIM); tft.setTextSize(1);
    tft.setCursor(CARD_PAD + 8, y + 6); tft.print("UPTIME");
    uint32_t h = uptime_s/3600, m = (uptime_s%3600)/60, s = uptime_s%60;
    snprintf(buf, sizeof(buf), "%02luh %02lum %02lus", h, m, s);
    int16_t up_w = strlen(buf) * 6;
    tft.setCursor(TFT_WIDTH - CARD_PAD - 8 - up_w, y + 6); tft.print(buf);
    y += UPT_H + SG;

    // Firmware info card (60px, 4 lines)
    const int16_t FW_H = 60;
    _card(CARD_PAD, y, TFT_WIDTH - CARD_PAD*2, FW_H, C_CARD, C_DIVIDER);
    tft.setTextColor(C_TEXT_DIM); tft.setTextSize(1);
    tft.setCursor(CARD_PAD + 8, y + 4); tft.print("FIRMWARE");
    char fwbuf[64];
    tft.setTextColor(C_ACCENT); tft.setTextSize(1);
    snprintf(fwbuf, sizeof(fwbuf), "v%u.%02u  (built %s)", MAJ_VERS, MIN_VERS, __DATE__);
    tft.setCursor(CARD_PAD + 8, y + 16); tft.print(fwbuf);
    tft.setTextColor(C_TEXT_DIM);
    snprintf(fwbuf, sizeof(fwbuf), "ESP32-S3  16MB Flash  SX1262");
    tft.setCursor(CARD_PAD + 8, y + 28); tft.print(fwbuf);
    tft.setTextColor(C_ACCENT);
    snprintf(fwbuf, sizeof(fwbuf), "Free RAM: %luKB / 320KB",
             ESP.getFreeHeap() / 1024UL);
    tft.setCursor(CARD_PAD + 8, y + 40); tft.print(fwbuf);
    y += FW_H + SG;

    // Bluetooth toggle card
    bool bt_on = (bt_state != BT_STATE_OFF);
    uint16_t bt_border = bt_on ? C_GREEN : C_RED;
    _card(CARD_PAD, y, TFT_WIDTH - CARD_PAD*2, SH, C_CARD, bt_border);
    // Top row: label left, tap hint right (both small)
    tft.setTextColor(C_TEXT_DIM); tft.setTextSize(1);
    tft.setCursor(CARD_PAD + 8, y + 4); tft.print("BLUETOOTH");
    const char* hint_txt = bt_on ? "(tap to disable)" : "(tap to enable)";
    int16_t hint_w = strlen(hint_txt) * 6;  // textSize 1 ≈ 6px/char
    tft.setCursor(TFT_WIDTH - CARD_PAD - 8 - hint_w, y + 4);
    tft.print(hint_txt);
    // Centered state (big)
    const char* state_txt = bt_on ? "Enabled" : "Disabled";
    int16_t state_w = strlen(state_txt) * 12;  // textSize 2 ≈ 12px/char
    tft.setTextColor(bt_on ? C_GREEN : C_RED); tft.setTextSize(2);
    tft.setCursor((TFT_WIDTH - state_w) / 2, y + 18); tft.print(state_txt);
    y += SH + SG;

    // WiFi toggle card
    bool wifi_on = (wifi_mode != WR_WIFI_OFF);
    bool wifi_connected = wifi_on && (wr_state == WR_STATE_CONNECTED);
    uint16_t wifi_border = !wifi_on ? C_RED : (wifi_connected ? C_GREEN : C_YELLOW);
    _card(CARD_PAD, y, TFT_WIDTH - CARD_PAD*2, SH, C_CARD, wifi_border);
    tft.setTextColor(C_TEXT_DIM); tft.setTextSize(1);
    tft.setCursor(CARD_PAD + 8, y + 4); tft.print("WIFI");
    const char* wifi_hint = wifi_on ? "(tap to disable)" : "(tap to enable)";
    int16_t wifi_hint_w = strlen(wifi_hint) * 6;
    tft.setCursor(TFT_WIDTH - CARD_PAD - 8 - wifi_hint_w, y + 4);
    tft.print(wifi_hint);
    if (wifi_connected) {
        String ssid = WiFi.SSID();
        char ssidbuf[14];
        snprintf(ssidbuf, sizeof(ssidbuf), "%s", ssid.c_str());
        int16_t ssid_w = strlen(ssidbuf) * 6;
        tft.setCursor((TFT_WIDTH - ssid_w) / 2, y + 4);
        tft.print(ssidbuf);
    }
    const char* wifi_state_txt;
    uint16_t wifi_state_col;
    if (!wifi_on)            { wifi_state_txt = "Disabled";    wifi_state_col = C_RED; }
    else if (wifi_connected) { wifi_state_txt = "Connected";   wifi_state_col = C_GREEN; }
    else                     { wifi_state_txt = "Connecting";  wifi_state_col = C_YELLOW; }
    int16_t wifi_state_w = strlen(wifi_state_txt) * 12;
    tft.setTextColor(wifi_state_col); tft.setTextSize(2);
    tft.setCursor((TFT_WIDTH - wifi_state_w) / 2, y + 18); tft.print(wifi_state_txt);
    y += SH + SG;

    // Display off button
    _btn(CARD_PAD, y, TFT_WIDTH - CARD_PAD*2, 44, "DISPLAY OFF", C_CARD, C_RED, 2);
}
// ── Touch handling ────────────────────────────────────────────────────────────
static void _handle_touch() {
    
    uint16_t tx = 0, ty = 0;
    if (!_touch_read(&tx, &ty)) return;
    uint32_t now = millis();
    if (now - _last_touch_ms < TOUCH_DEBOUNCE) return;
    _last_touch_ms = now;

    // Set redraw flag for button response
    _needs_redraw = true;

    // Splash: any touch advances
    if (!_splash_done) {
        _splash_done = true;
        _needs_redraw = true;
        return;
    }

    // Nav bar tap
    if (ty >= NAVBAR_Y) {
        int16_t tab_w = TFT_WIDTH / TAB_COUNT;
        TabID new_tab = (TabID)(tx / tab_w);
        if (new_tab < TAB_COUNT && new_tab != _tab) {
            // Flash the tapped tab white briefly
            int16_t tab_x = new_tab * tab_w;
            tft.fillRect(tab_x + 2, NAVBAR_Y + 2, tab_w - 4, NAVBAR_H - 4, C_ACCENT);
            delay(60);
            _tab = new_tab;
            _needs_redraw = true;
        }
        return;
    }

    // TOOLS tab buttons (BT toggle, WiFi toggle, DISPLAY OFF)
    if (_tab == TAB_STATS) {
        // BLUETOOTH toggle card sits after UPTIME (20+3) + FIRMWARE (60+3)
        int16_t bt_y = CONTENT_Y + 4 + (20 + 3) + (60 + 3);
        if (ty >= (uint16_t)bt_y && ty <= (uint16_t)(bt_y + 40)) {
            tft.fillRoundRect(CARD_PAD, bt_y, TFT_WIDTH - CARD_PAD*2, 40, CARD_RADIUS, C_ACCENT);
            delay(60);
            if (bt_state != BT_STATE_OFF) {
                bt_stop();
                bt_conf_save(false);
            } else {
                bt_start();
                bt_conf_save(true);
            }
            _needs_redraw = true;
        }
        // WIFI toggle card sits after UPTIME (20+3) + FIRMWARE (60+3) + BLUETOOTH (40+3)
        int16_t wf_y = CONTENT_Y + 4 + (20 + 3) + (60 + 3) + (40 + 3);
        if (ty >= (uint16_t)wf_y && ty <= (uint16_t)(wf_y + 40)) {
            tft.fillRoundRect(CARD_PAD, wf_y, TFT_WIDTH - CARD_PAD*2, 40, CARD_RADIUS, C_ACCENT);
            delay(60);
            if (wifi_mode != WR_WIFI_OFF) {
                wifi_remote_stop();
                wifi_mode = WR_WIFI_OFF;
                wr_conf_save(WR_WIFI_OFF);
            } else {
                wifi_mode = WR_WIFI_STA;
                wr_conf_save(WR_WIFI_STA);
                wifi_remote_init();
            }
            _needs_redraw = true;
        }
        // DISPLAY OFF button below all 4 cards
        int16_t pwr_y = CONTENT_Y + 4 + (20 + 3) + (60 + 3) + (40 + 3) * 2;
        if (ty >= (uint16_t)pwr_y && ty <= (uint16_t)(pwr_y + 44)) {
            tft.fillRoundRect(CARD_PAD, pwr_y, TFT_WIDTH - CARD_PAD*2, 44, CARD_RADIUS, C_RED);
            tft.setTextColor(C_BG); tft.setTextSize(2);
            int16_t tw = 11 * 12; tft.setCursor((TFT_WIDTH - tw)/2, pwr_y + 14);
            tft.print("DISPLAY OFF");
            delay(300);
            tft.fillScreen(C_BG);
            display_blanked = true;
        }
    }

    // BT tab button taps
    if (_tab == TAB_BT) {
        bool fw_connected = (bt_state == BT_STATE_CONNECTED);
        bool fw_pairing   = (bt_state == BT_STATE_PAIRING);
        // Button position depends on whether MAC card is shown (hidden during pairing)
        int cards_above = 3;  // name + MAC + status always shown
        int16_t btn_y_start = CONTENT_Y + 8 + (BT_CARD_H + 4) * cards_above;
        if (fw_connected) btn_y_start += (BT_CARD_H + 4);  // + connected-device card

        if (ty >= (uint16_t)btn_y_start && ty <= (uint16_t)(btn_y_start + BT_CARD_H)) {
            // Flash button
            tft.fillRoundRect(CARD_PAD, btn_y_start, TFT_WIDTH - CARD_PAD*2, BT_CARD_H, CARD_RADIUS, C_ACCENT);
            delay(60);
            if (!fw_connected && !fw_pairing) {
                // Start pairing via firmware function
                if (!bt_ready) bt_init();
                bt_enable_pairing();
                _bt_pair_start = now;
            } else if (fw_pairing) {
                // Cancel pairing
                bt_disable_pairing();
            } else if (fw_connected) {
                bt_disable_pairing();
            }
            _needs_redraw = true;
        }
    }
}

// ── Main update ──────────────────────────────────────────────────────────────
void tft_update(float freq_mhz, uint8_t sf, uint32_t bw_hz, int8_t cr,
                int16_t last_rssi, float last_snr,
                uint32_t rx_count, uint32_t tx_count,
                bool airtime_lock,
                uint8_t tx_power_dbm, bool host_connected,
                bool rx_ongoing) {

    uint32_t now = millis();

    // Sync backlight with firmware display blanking
    static bool _last_blanked = false;
    if (display_blanked != _last_blanked) {
        _last_blanked = display_blanked;
        tft_backlight(!display_blanked);
        if (!display_blanked) _needs_redraw = true;
    }
    if (display_blanked) return;

    // Detect state changes that should trigger a redraw
    bool new_bt_conn = (bt_state == BT_STATE_CONNECTED);
    if (host_connected != _host_connected ||
        new_bt_conn   != _bt_connected   ||
        last_rssi     != _rssi           ||
        last_snr      != _snr            ||
        airtime_lock  != _airlock        ||
        tx_power_dbm  != _txp) {
        _needs_redraw = true;
    }
    // Update cached values
    _freq=freq_mhz; _sf=sf; _bw=bw_hz; _cr=cr;
    _rssi=last_rssi; _snr=last_snr;
    if (rx_count != _last_rx_count_seen) {
        _last_rx_count_seen = rx_count;
        _last_rx_time = millis();
        _needs_redraw = true;
    }
    if (tx_count != _last_tx_count_seen) {
        _last_tx_count_seen = tx_count;
        _last_tx_time = millis();
        _needs_redraw = true;
    }
    _rx=rx_count; _tx=tx_count;
    _airlock=airtime_lock; _rx_ongoing=rx_ongoing;
    _txp=tx_power_dbm; _host_connected=host_connected;
    _bt_connected = new_bt_conn;

    // Always handle touch first
    _handle_touch();

    // Auto-advance splash after timeout
    if (!_splash_done && (now - _boot_ms > SPLASH_MS)) {
        _splash_done = true;
        _needs_redraw = true;

    }

    // Don't render until splash is dismissed
    if (!_splash_done) return;

    // BT pairing timeout
    if (_bt_state == BT_STATE_PAIRING && (now - _bt_pair_start > BT_PIN_TIMEOUT)) {
        _bt_state = BT_IDLE;
        snprintf(_bt_pin, sizeof(_bt_pin), "------");
        _needs_redraw = true;
    }

    // BT tab countdown refresh every second
    if (_tab == TAB_BT && _bt_state == BT_STATE_PAIRING &&
        now - _last_refresh >= 1000) {
        _needs_redraw = true;
    }

    // BT tab: in-place countdown update — no full-screen redraw
    static uint32_t _last_ssp = 0;
    static uint32_t _last_remaining = 0xFFFFFFFF;
    if (_tab == TAB_BT && bt_state == BT_STATE_PAIRING) {
        uint32_t elapsed = (now > bt_pairing_started) ? now - bt_pairing_started : 0;
        uint32_t remaining = (elapsed < 35000UL) ? (35000UL - elapsed) / 1000 : 0;
        if (bt_ssp_pin != _last_ssp) {
            // PIN changed (first appears) — needs full redraw to draw PIN card
            _last_ssp = bt_ssp_pin;
            _last_remaining = remaining;
            _needs_redraw = true;
        } else if (remaining != _last_remaining) {
            // Just countdown changed — repaint only the STATUS card value
            _last_remaining = remaining;
            int16_t status_y = CONTENT_Y + 8 + 2 * (BT_CARD_H + 4);
            tft.fillRect(CARD_PAD + 4, status_y + 14,
                         TFT_WIDTH - CARD_PAD*2 - 8, BT_CARD_H - 16, C_CARD);
            tft.setTextColor(C_YELLOW); tft.setTextSize(2);
            char tbuf[20];
            snprintf(tbuf, sizeof(tbuf), "Pairing %lus", remaining);
            tft.setCursor(CARD_PAD + 8, status_y + 16);
            tft.print(tbuf);
        }
    }

    // Periodic forced refresh every 60s
    if (now - _last_refresh >= 60000UL) {
        _needs_redraw = true;
        _last_refresh = now;
    }

    if (!_needs_redraw) return;
    _needs_redraw = false;
    _last_refresh = now;

    switch (_tab) {
        case TAB_HOME:  _draw_home();  break;
        case TAB_RADIO: _draw_radio(); break;
        case TAB_BT:    _draw_bt();    break;
        case TAB_STATS: _draw_stats(); break;
        default: break;
    }
    _navbar();
}
