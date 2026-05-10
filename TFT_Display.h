// TFT_Display.h
// Full touch UI for RNode Firmware - Heltec WiFi LoRa 32 V4 Expansion Kit
// Hardware: ST7789 240x320 SPI TFT + CHSC6X capacitive touch
//
// Pinout (from expansion board schematic AFC07-S18ECA-00):
//   TFT SPI:  CS=15  DC=16  MOSI=33  SCLK=17  RST=18  BLK=21
//   Touch I2C(Wire1): SDA=47  SCL=48  INT=45  RST=44
//   Vext enable: GPIO36 LOW=ON
//
// Required libraries:
//   - "Adafruit ST7735 and ST7789 Library" by Adafruit
//   - "Adafruit GFX Library"               by Adafruit

#pragma once
#include "Arduino.h"
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#include <Wire.h>

// ── Pin definitions ──────────────────────────────────────────────────────────
#define TFT_PIN_CS      15
#define TFT_PIN_RST     18
#define TFT_PIN_DC      16
#define TFT_PIN_MOSI    33
#define TFT_PIN_SCLK    17
#define TFT_PIN_BLK     21

#define TOUCH_PIN_SDA   47
#define TOUCH_PIN_SCL   48
#define TOUCH_PIN_INT   45
#define TOUCH_PIN_RST   44
#define VEXT_PIN        36

// ── Display dimensions ───────────────────────────────────────────────────────
#define TFT_WIDTH       240
#define TFT_HEIGHT      320

// ── Colour palette (RGB565) ──────────────────────────────────────────────────
#define C_BG            0x0841   // Very dark blue-grey background
#define C_HEADER        0x0A8F   // Teal header
#define C_CARD          0x1082   // Dark card background
#define C_CARD_SEL      0x1492   // Selected card
#define C_TEXT          0xFFFF   // White text
#define C_TEXT_DIM      0x8410   // Grey text
#define C_ACCENT        0x07FF   // Cyan accent
#define C_GREEN         0x07E0   // Green - good
#define C_YELLOW        0xFFE0   // Yellow - warning
#define C_RED           0xF800   // Red - bad
#define C_ORANGE        0xFC60   // Orange
#define C_DIVIDER       0x2104   // Subtle divider
#define C_BTN           0x0339   // Button background
#define C_BTN_SEL       0x03BF   // Selected button
#define C_NAVBG         0x0841   // Nav bar background
#define C_NAVSEL        0x07FF   // Selected nav item

// ── Layout ───────────────────────────────────────────────────────────────────
#define HEADER_H        42
#define NAVBAR_H        52
#define CONTENT_Y       (HEADER_H + 2)
#define CONTENT_H       (TFT_HEIGHT - HEADER_H - NAVBAR_H - 4)
#define NAVBAR_Y        (TFT_HEIGHT - NAVBAR_H)
#define CARD_PAD        8
#define CARD_H          52
#define CARD_RADIUS     6

// ── Nav tabs ─────────────────────────────────────────────────────────────────
typedef enum {
    TAB_HOME    = 0,
    TAB_RADIO   = 1,
    TAB_BT      = 2,
    TAB_STATS   = 3,
    TAB_COUNT
} TabID;

// ── BT pairing state ─────────────────────────────────────────────────────────
typedef enum {
    BT_IDLE     = 0,
    BT_PAIRING  = 1,
    BT_CONNECTED= 2,
    BT_DISABLED = 3
} BTState;

// ── Driver object ────────────────────────────────────────────────────────────
extern Adafruit_ST7789 tft;

// ── Public API ───────────────────────────────────────────────────────────────
void tft_init();
void tft_update(
    float    freq_mhz,
    uint8_t  sf,
    uint32_t bw_hz,
    int8_t   cr,
    int16_t  last_rssi,
    float    last_snr,
    uint32_t rx_count,
    uint32_t tx_count,
    bool     airtime_lock,
    uint8_t  led_state
);
void tft_invalidate();
void tft_backlight(bool on);
void _draw_splash();
