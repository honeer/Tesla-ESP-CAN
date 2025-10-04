/*
  Project:   Tesla-ESP-CAN
  Module:    ESP32 + MCP2515 CAN Bus Listener with WS2812B LED Feedback
  Company:   Honeer Automotive
  Date:      October 4, 2025
  Author:    Honeer Automotive Development Team

  Description:
  This sketch runs on a standalone ESP32 with an MCP2515 CAN controller and a 
  WS2812B LED strip (6 LEDs). It tests communication with the Tesla CAN bus and 
  provides visual feedback through the LEDs. When turn signals are activated on 
  the Tesla vehicle, the code mirrors them by blinking 3 LEDs on the strip to 
  indicate LEFT or RIGHT turns.

  Debug LED States (6 WS2812B diodes):
    • BOOTING:           Quick cyan wipe once at power-up
    • CAN_INIT:          Pulsing yellow while initializing MCP2515
    • WAITING_FOR_DATA:  Slow white blink until first valid 0x3F5 frame arrives
    • RUNNING:           Normal left/right indicator mirroring
    • ERROR:             Flashing red if CAN init fails

  Hardware:
    • ESP32 Development Board
    • MCP2515 CAN Controller Module (SPI)
    • WS2812B LED Strip (6 LEDs)

  IDE:       Arduino IDE 2.3.6
  Libraries: 
    • WiFi.h (built-in, ESP32 board support)
    • SPI.h (built-in)
    • mcp_can.h (MCP2515 CAN library)
    • Adafruit_NeoPixel.h (Adafruit WS2812B LED library)

  Board:     ESP32 Dev Module (or compatible ESP32 board)
  Version:   v1.0
*/

#include <SPI.h>
#include "mcp_can.h"
#include <Adafruit_NeoPixel.h>

// ---------- USER SETTINGS ----------
#define MCP2515_CS_PIN      15      // Chip Select for MCP2515
#define MCP2515_INT_PIN     4       // (optional) INT pin - not used here
#define NEOPIXEL_PIN        21      // WS2812B data pin
#define NUM_LEDS            6       // 0..2 left, 3..5 right

// CAN parameters
#define CAN_ID_VCFRONT_LIGHTING 0x3F5
#define CAN_BAUD        CAN_500KBPS     // Tesla PT CAN typically 500 kbps
#define CAN_CLK         MCP_8MHZ        // change to MCP_16MHZ if your board has a 16 MHz crystal

// ---------- DEBUG SETTINGS ----------
#define DEBUG_LEDS_ENABLED   1   // set to 0 to disable debug animations entirely

// Debug colors (GRB)
#define DBG_COLOR_BOOT       strip.Color(0, 120, 120)  // cyan
#define DBG_COLOR_INIT       strip.Color(255, 160, 0)  // yellow/orange
#define DBG_COLOR_WAIT       strip.Color(80, 80, 80)   // white (dim)
#define DBG_COLOR_ERROR      strip.Color(255, 0, 0)    // red

// Normal indicator colors
#define COLOR_LEFT_ON   strip.Color(0, 255, 0)     // green
#define COLOR_RIGHT_ON  strip.Color(0, 0, 255)     // blue
#define COLOR_FAULT     strip.Color(255, 200, 0)   // amber/yellow
#define COLOR_SNA       strip.Color(0, 0, 80)      // dim blue
#define COLOR_OFF       strip.Color(0, 0, 0)

// ---------- GLOBALS ----------
MCP_CAN CAN0(MCP2515_CS_PIN);
Adafruit_NeoPixel strip(NUM_LEDS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// Debug state machine
enum DebugMode : uint8_t {
  DBG_BOOTING,
  DBG_CAN_INIT,
  DBG_WAITING_FOR_DATA,
  DBG_RUNNING,
  DBG_ERROR
};
DebugMode dbgMode = DBG_BOOTING;

uint32_t dbgAnimMs = 0;
uint32_t lastFrameMs = 0;
bool     haveSeenData = false;

// Last decoded statuses
uint8_t leftStatus  = 0;  // 0=OFF,1=ON,2=FAULT,3=SNA
uint8_t rightStatus = 0;

// Timeouts/periods
const uint32_t FRAME_TIMEOUT_MS = 700;    // blank if data stops
const uint16_t BOOT_WIPE_DELAY  = 60;     // ms between boot wipe steps
const uint16_t INIT_PULSE_MS    = 700;    // yellow pulse period
const uint16_t WAIT_BLINK_MS    = 600;    // white blink when waiting for traffic
const uint16_t ERROR_FLASH_MS   = 250;    // red flash rate

// --------- Motorola/big-endian extract (start|len@1+) ---------
static uint64_t extract_moto_be(const uint8_t data[8], uint8_t startBit, uint8_t len) {
  uint64_t be = 0;
  for (int i = 0; i < 8; i++) be = (be << 8) | data[i];
  uint8_t shift = 64 - startBit - len;
  uint64_t mask = (len == 64) ? UINT64_MAX : ((1ULL << len) - 1ULL);
  return (be >> shift) & mask;
}

void setSide(uint8_t startIdx, uint32_t color) {
  for (uint8_t i = 0; i < 3; i++) strip.setPixelColor(startIdx + i, color);
}

uint32_t colorForStatus(uint8_t status, bool isLeft) {
  switch (status) {
    case 1:  return isLeft ? COLOR_LEFT_ON : COLOR_RIGHT_ON; // ON
    case 2:  return COLOR_FAULT;                              // FAULT
    case 3:  return COLOR_SNA;                                // SNA
    default: return COLOR_OFF;                                // OFF/unknown
  }
}

void showIndicators() {
  setSide(0, colorForStatus(leftStatus, true));   // LEDs 0..2
  setSide(3, colorForStatus(rightStatus, false)); // LEDs 3..5
  strip.show();
}

// ---------- Debug animations ----------
void debugBootWipeOnce() {
#if DEBUG_LEDS_ENABLED
  // Quick cyan wipe 0..5
  for (int i = 0; i < NUM_LEDS; i++) {
    strip.clear();
    strip.setPixelColor(i, DBG_COLOR_BOOT);
    strip.show();
    delay(BOOT_WIPE_DELAY);
  }
  strip.clear();
  strip.show();
#endif
}

void debugShowInit() {
#if DEBUG_LEDS_ENABLED
  // Yellow "breathing": all LEDs pulse
  uint16_t period = INIT_PULSE_MS;
  uint16_t t = millis() % period;
  float phase = (t < period/2) ? (t / float(period/2)) : (1.0f - (t - period/2) / float(period/2));
  uint8_t b = 20 + uint8_t(phase * 180); // 20..200
  uint32_t c = strip.Color(b, b/2, 0);   // warm yellow-ish
  for (int i=0;i<NUM_LEDS;i++) strip.setPixelColor(i, c);
  strip.show();
#endif
}

void debugShowWaiting() {
#if DEBUG_LEDS_ENABLED
  // Slow white blink on the two outer groups: [0..2] and [3..5]
  bool on = ((millis() / WAIT_BLINK_MS) % 2) == 0;
  uint32_t c = on ? DBG_COLOR_WAIT : COLOR_OFF;
  setSide(0, c);
  setSide(3, c);
  strip.show();
#endif
}

void debugShowError() {
#if DEBUG_LEDS_ENABLED
  bool on = ((millis() / ERROR_FLASH_MS) % 2) == 0;
  uint32_t c = on ? DBG_COLOR_ERROR : COLOR_OFF;
  for (int i=0;i<NUM_LEDS;i++) strip.setPixelColor(i, c);
  strip.show();
#endif
}

void enterMode(DebugMode m) {
  dbgMode = m;
  dbgAnimMs = millis();
  if (m == DBG_BOOTING) {
    debugBootWipeOnce(); // one-time animation
  }
}

void setup() {
  Serial.begin(115200);
  delay(50);

  strip.begin();
  strip.clear();
  strip.show();

  enterMode(DBG_BOOTING);

  // Init MCP2515
  enterMode(DBG_CAN_INIT);
  SPI.begin(); // VSPI default: SCK=18, MISO=19, MOSI=23
  if (CAN0.begin(MCP_ANY, CAN_BAUD, CAN_CLK) == CAN_OK) {
    Serial.println("MCP2515: init OK");
  } else {
    Serial.println("MCP2515: init FAILED");
    enterMode(DBG_ERROR);
    // Stay in error; don't proceed to normal mode
    return;
  }

  CAN0.setMode(MCP_NORMAL); // start receiving

  // Filters: only accept 0x3F5
  CAN0.init_Mask(0, 0, 0x7FF);                 // mask all 11 bits
  CAN0.init_Filt(0, 0, CAN_ID_VCFRONT_LIGHTING);
  CAN0.init_Mask(1, 0, 0x7FF);
  CAN0.init_Filt(2, 0, CAN_ID_VCFRONT_LIGHTING);

  haveSeenData = false;
  lastFrameMs = millis();
  enterMode(DBG_WAITING_FOR_DATA);
}

void loop() {
  // Drive debug UI if not running
  if (dbgMode == DBG_CAN_INIT) {
    debugShowInit();
  } else if (dbgMode == DBG_WAITING_FOR_DATA) {
    debugShowWaiting();
  } else if (dbgMode == DBG_ERROR) {
    debugShowError();
  }

  // Poll for CAN frames
  if (CAN_MSGAVAIL == CAN0.checkReceive()) {
    long unsigned int rxId = 0;
    unsigned char len = 0;
    uint8_t buf[8] = {0};

    if (CAN0.readMsgBuf(&rxId, &len, buf) == CAN_OK) {
      if ((rxId == CAN_ID_VCFRONT_LIGHTING) && (len == 8)) {
        uint8_t lStat = (uint8_t)extract_moto_be(buf, 50, 2);
        uint8_t rStat = (uint8_t)extract_moto_be(buf, 52, 2);

        bool changed = false;
        if (lStat != leftStatus)  { leftStatus = lStat;  changed = true; }
        if (rStat != rightStatus) { rightStatus = rStat; changed = true; }

        lastFrameMs = millis();

        if (!haveSeenData) {
          haveSeenData = true;
          enterMode(DBG_RUNNING);
        }

        if (dbgMode == DBG_RUNNING && changed) {
          showIndicators();
        }
      }
    }
  }

  // Timeout handling: if we were running and traffic stops, go back to WAITING
  if (dbgMode == DBG_RUNNING && (millis() - lastFrameMs > FRAME_TIMEOUT_MS)) {
    // blank indicators and show waiting pattern
    leftStatus = 0; rightStatus = 0; showIndicators();
    haveSeenData = false;
    enterMode(DBG_WAITING_FOR_DATA);
  }

  // If running and not changed this loop, ensure LEDs match current state
  if (dbgMode == DBG_RUNNING) {
    // Optional: keep updating every ~100ms to counter any LED glitches
    static uint32_t lastRefresh = 0;
    if (millis() - lastRefresh > 120) {
      showIndicators();
      lastRefresh = millis();
    }
  }
}
