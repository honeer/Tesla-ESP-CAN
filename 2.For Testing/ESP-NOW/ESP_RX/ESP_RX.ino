/*
  Project:   Tesla-ESP-CAN
  Module:    ESP32 ESP-NOW Receiver with LED Output
  Company:   Honeer Automotive
  Date:      October 4, 2025

  Description:
  This ESP32 listens for ESP-NOW messages from the transmitter 
  and controls a WS2812B LED strip (6 LEDs). 
  LEDs blink according to Tesla turn signal commands (LEFT / RIGHT). 
  Debug states are also shown with color-coded LED feedback.

  Hardware:  ESP32 Development Board, WS2812B LED Strip
  Protocol:  ESP-NOW
  Version:   v1.0
*/

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Adafruit_NeoPixel.h>

// ---------- LED STRIP ----------
#define NEOPIXEL_PIN   21
#define NUM_LEDS       6
Adafruit_NeoPixel strip(NUM_LEDS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// ---------- COLORS ----------
#define COLOR_LEFT_ON    strip.Color(0, 255, 0)
#define COLOR_RIGHT_ON   strip.Color(0, 0, 255)
#define COLOR_FAULT      strip.Color(255, 200, 0)
#define COLOR_SNA        strip.Color(0, 0, 80)
#define COLOR_OFF        strip.Color(0, 0, 0)

#define DBG_COLOR_BOOT   strip.Color(0, 120, 120) // cyan
#define DBG_COLOR_INIT   strip.Color(255, 160, 0) // yellow
#define DBG_COLOR_WAIT   strip.Color(80, 80, 80)  // dim white
#define DBG_COLOR_ERROR  strip.Color(255, 0, 0)   // red

const uint8_t WIFI_CHANNEL = 1; // must match TX

// ---------- PACKET ----------
struct __attribute__((packed)) Payload {
  uint8_t  magic;       // 0xA5
  uint8_t  version;     // 1
  uint8_t  leftStatus;  // 0..3
  uint8_t  rightStatus; // 0..3
  uint8_t  flags;       // bit0=haveSeenData
  uint16_t seq;         // rolling sequence
  uint32_t ms;          // sender millis
  uint8_t  crc;         // xor
};

static inline uint8_t calc_crc(const Payload& p) {
  const uint8_t* b = reinterpret_cast<const uint8_t*>(&p);
  uint8_t x = 0;
  for (size_t i = 0; i < sizeof(Payload)-1; i++) x ^= b[i];
  return x;
}

// ---------- STATE ----------
enum Mode : uint8_t {
  DBG_BOOTING,
  DBG_ESPNOW_INIT,
  DBG_WAITING_FOR_LINK,
  DBG_WAITING_FOR_DATA,
  DBG_RUNNING,
  DBG_ERROR
};
Mode mode_ = DBG_BOOTING;

uint8_t leftStatus = 0, rightStatus = 0;
bool haveSeenData = false;
bool linkSeen = false;

uint32_t lastPktMs = 0, lastShowMs = 0;
const uint32_t LINK_TIMEOUT_MS = 1000;
const uint16_t WAIT_BLINK_MS  = 600;
const uint16_t INIT_PULSE_MS  = 700;
const uint16_t ERROR_FLASH_MS = 250;

// --- Link blink (3x green) ---
#define LINK_BLINK_COLOR  strip.Color(0, 255, 0)
#define LINK_BLINK_ON_MS  200
#define LINK_BLINK_OFF_MS 200
#define LINK_BLINK_COUNT  3
bool linkBlinkActive = false;
uint8_t linkBlinkStep = 0;
uint32_t linkBlinkNext = 0;

// ---------- FWD DECLS ----------
void enterMode(Mode m);
void debugBootWipeOnce();
void debugShowInit();
void debugShowWaiting();
void debugShowError();
void showIndicators();

// ---------- HELPERS ----------
void setSide(uint8_t startIdx, uint32_t color) { for (uint8_t i=0;i<3;i++) strip.setPixelColor(startIdx+i, color); }
uint32_t colorForStatus(uint8_t s, bool isLeft) {
  switch (s) { case 1: return isLeft ? COLOR_LEFT_ON : COLOR_RIGHT_ON; case 2: return COLOR_FAULT; case 3: return COLOR_SNA; default: return COLOR_OFF; }
}
void showIndicators() {
  setSide(0, colorForStatus(leftStatus, true));
  setSide(3, colorForStatus(rightStatus, false));
  strip.show();
}

// ---------- DEBUG VISUALS ----------
void debugBootWipeOnce() {
  for (int i=0;i<NUM_LEDS;i++){ strip.clear(); strip.setPixelColor(i, DBG_COLOR_BOOT); strip.show(); delay(60); }
  strip.clear(); strip.show();
}
void debugShowInit() {
  uint16_t t = millis() % INIT_PULSE_MS;
  float phase = (t < INIT_PULSE_MS/2) ? (t / float(INIT_PULSE_MS/2)) : (1.0f - (t - INIT_PULSE_MS/2) / float(INIT_PULSE_MS/2));
  uint8_t b = 20 + uint8_t(phase * 180);
  uint32_t c = strip.Color(b, b/2, 0);
  for (int i=0;i<NUM_LEDS;i++) strip.setPixelColor(i, c);
  strip.show();
}
void debugShowWaiting() {
  bool on = ((millis() / WAIT_BLINK_MS) % 2) == 0;
  uint32_t c = on ? DBG_COLOR_WAIT : COLOR_OFF;
  setSide(0, c); setSide(3, c); strip.show();
}
void debugShowError() {
  bool on = ((millis() / ERROR_FLASH_MS) % 2) == 0;
  uint32_t c = on ? DBG_COLOR_ERROR : COLOR_OFF;
  for (int i=0;i<NUM_LEDS;i++) strip.setPixelColor(i, c);
  strip.show();
}

void enterMode(Mode m) {
  mode_ = m;
  if (m == DBG_BOOTING) debugBootWipeOnce();
}

// NEW signature for ESP-NOW receive callback
void onRecv(const esp_now_recv_info *info, const uint8_t *data, int len) {
  (void)info;
  if (len != (int)sizeof(Payload)) return;
  Payload p; memcpy(&p, data, sizeof(Payload));
  if (p.magic != 0xA5 || p.version != 1) return;
  if (calc_crc(p) != p.crc) return;

  // If we were previously "not linked", start 3x blink sequence
  if (!linkSeen) {
    linkBlinkActive = true;
    linkBlinkStep = 0;
    linkBlinkNext = millis(); // start immediately
  }
  linkSeen = true;
  lastPktMs = millis();

  leftStatus = p.leftStatus;
  rightStatus = p.rightStatus;
  haveSeenData = (p.flags & 0x01);

  if (haveSeenData && mode_ != DBG_RUNNING) enterMode(DBG_RUNNING);
  else if (!haveSeenData && mode_ == DBG_RUNNING) enterMode(DBG_WAITING_FOR_DATA);
  else if (!haveSeenData && (mode_ == DBG_WAITING_FOR_LINK)) enterMode(DBG_WAITING_FOR_DATA);

  if (mode_ == DBG_RUNNING) showIndicators();
}

void setup() {
  Serial.begin(115200); delay(50);

  strip.begin(); strip.clear(); strip.show();
  enterMode(DBG_BOOTING);

  Serial.print("Receiver MAC: "); Serial.println(WiFi.macAddress());

  WiFi.mode(WIFI_STA);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  enterMode(DBG_ESPNOW_INIT);
  if (esp_now_init() != ESP_OK) { Serial.println("ESP-NOW init failed"); enterMode(DBG_ERROR); return; }
  esp_now_register_recv_cb(onRecv);

  enterMode(DBG_WAITING_FOR_LINK);
}

void loop() {
  // --- Handle green blink sequence (3x) ---
  if (linkBlinkActive) {
    if (millis() >= linkBlinkNext) {
      if (linkBlinkStep % 2 == 0) {
        // ON phase
        for (int i=0; i<NUM_LEDS; i++) strip.setPixelColor(i, LINK_BLINK_COLOR);
        strip.show();
        linkBlinkNext = millis() + LINK_BLINK_ON_MS;
      } else {
        // OFF phase
        strip.clear(); strip.show();
        linkBlinkNext = millis() + LINK_BLINK_OFF_MS;
      }
      linkBlinkStep++;
      if (linkBlinkStep >= LINK_BLINK_COUNT * 2) {
        linkBlinkActive = false; // done
        strip.clear(); strip.show();
      }
    }
    return; // hold off normal UI while blinking
  }

  // --- Normal debug UI ---
  if (mode_ == DBG_ESPNOW_INIT) {
    debugShowInit();
  } else if (mode_ == DBG_WAITING_FOR_LINK || (mode_ == DBG_WAITING_FOR_DATA && !haveSeenData)) {
    debugShowWaiting();
  } else if (mode_ == DBG_ERROR) {
    debugShowError();
  }

  // Link supervision
  if (linkSeen && (millis() - lastPktMs > LINK_TIMEOUT_MS)) {
    linkSeen = false; haveSeenData = false; leftStatus = rightStatus = 0;
    strip.clear(); strip.show(); enterMode(DBG_WAITING_FOR_LINK);
  }

  // Periodic refresh in RUNNING
  if (mode_ == DBG_RUNNING && millis() - lastShowMs > 120) {
    showIndicators(); lastShowMs = millis();
  }
}
