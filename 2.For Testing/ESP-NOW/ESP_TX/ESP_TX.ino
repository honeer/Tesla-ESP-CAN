/*
  Project:   Tesla-ESP-CAN
  Module:    ESP32 ESP-NOW Transmitter
  Company:   Honeer Automotive
  Date:      October 4, 2025

  Description:
  This ESP32 reads data from the Tesla CAN bus and transmits 
  turn signal states (LEFT / RIGHT) wirelessly using ESP-NOW. 
  The receiver ESP32 mirrors the signals with LED output.

  Hardware:  ESP32 Development Board
  Protocol:  ESP-NOW
  Version:   v1.0
*/

#include <SPI.h>
#include "mcp_can.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// ---------- MCP2515 ----------
#define MCP2515_CS_PIN   15
#define MCP2515_INT_PIN  4   // optional
#define CAN_ID_VCFRONT_LIGHTING 0x3F5
#define CAN_BAUD   CAN_500KBPS
#define CAN_CLK    MCP_8MHZ   // change to MCP_16MHZ if your MCP2515 is 16 MHz

MCP_CAN CAN0(MCP2515_CS_PIN);

// ---------- ESP-NOW ----------
uint8_t PEER_MAC[6] = { 0x6C, 0xC8, 0x40, 0x94, 0x14, 0x70 }; // RX (LED side)
const uint8_t WIFI_CHANNEL = 1; // must match RX

struct __attribute__((packed)) Payload {
  uint8_t  magic;       // 0xA5
  uint8_t  version;     // 1
  uint8_t  leftStatus;  // 0..3
  uint8_t  rightStatus; // 0..3
  uint8_t  flags;       // bit0 = haveSeenData
  uint16_t seq;         // rolling sequence
  uint32_t ms;          // sender millis
  uint8_t  crc;         // xor of previous bytes
};

volatile uint16_t seq = 0;

static inline uint8_t make_crc(const Payload& p) {
  const uint8_t* b = reinterpret_cast<const uint8_t*>(&p);
  uint8_t x = 0;
  for (size_t i = 0; i < sizeof(Payload)-1; i++) x ^= b[i];
  return x;
}

// New send callback signature (ESP32 core 3.x / IDF v5)
void onSend(const esp_now_send_info_t *info, esp_now_send_status_t status) {
  (void)info; // not used
  // Serial.printf("ESP-NOW send: %s\n", status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

// ---------- State ----------
uint8_t leftStatus = 0, rightStatus = 0;
bool haveSeenData = false;
uint32_t lastFrameMs = 0;
const uint32_t FRAME_TIMEOUT_MS = 700;
uint32_t lastHeartbeatMs = 0;
const uint32_t HEARTBEAT_MS = 150; // ~6–7 Hz

static uint64_t extract_moto_be(const uint8_t data[8], uint8_t startBit, uint8_t len) {
  uint64_t be = 0;
  for (int i = 0; i < 8; i++) be = (be << 8) | data[i];
  uint8_t shift = 64 - startBit - len;
  uint64_t mask = (len == 64) ? UINT64_MAX : ((1ULL << len) - 1ULL);
  return (be >> shift) & mask;
}

void sendPacket(bool force) {
  static uint8_t prevL = 0xFF, prevR = 0xFF, prevFlags = 0xFF;
  uint8_t flags = haveSeenData ? 0x01 : 0x00;
  bool changed = (leftStatus != prevL) || (rightStatus != prevR) || (flags != prevFlags);

  if (!changed && !force) return;

  Payload p{};
  p.magic = 0xA5;
  p.version = 1;
  p.leftStatus = leftStatus;
  p.rightStatus = rightStatus;
  p.flags = flags;
  p.seq = ++seq;
  p.ms = millis();
  p.crc = make_crc(p);

  esp_now_send(PEER_MAC, reinterpret_cast<const uint8_t*>(&p), sizeof(p));
  prevL = leftStatus; prevR = rightStatus; prevFlags = flags;
}

void setup() {
  Serial.begin(115200); delay(50);

  // Wi-Fi station + fixed channel for ESP-NOW
  WiFi.mode(WIFI_STA);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) { Serial.println("ESP-NOW init failed"); while (1) delay(1000); }
  esp_now_register_send_cb(onSend);

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, PEER_MAC, 6);
  peer.channel = WIFI_CHANNEL;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) { Serial.println("Add peer failed"); while (1) delay(1000); }

  // MCP2515
  SPI.begin(); // VSPI: SCK=18, MISO=19, MOSI=23
  if (CAN0.begin(MCP_ANY, CAN_BAUD, CAN_CLK) != CAN_OK) { Serial.println("MCP2515 init FAILED"); while (1) delay(1000); }
  CAN0.setMode(MCP_NORMAL);
  CAN0.init_Mask(0, 0, 0x7FF); CAN0.init_Filt(0, 0, CAN_ID_VCFRONT_LIGHTING);
  CAN0.init_Mask(1, 0, 0x7FF); CAN0.init_Filt(2, 0, CAN_ID_VCFRONT_LIGHTING);

  lastFrameMs = millis();
}

void loop() {
  // Receive CAN
  if (CAN_MSGAVAIL == CAN0.checkReceive()) {
    long unsigned int rxId = 0; unsigned char len = 0; uint8_t buf[8] = {0};
    if (CAN0.readMsgBuf(&rxId, &len, buf) == CAN_OK) {
      if (rxId == CAN_ID_VCFRONT_LIGHTING && len == 8) {
        leftStatus  = (uint8_t)extract_moto_be(buf, 50, 2);
        rightStatus = (uint8_t)extract_moto_be(buf, 52, 2);
        haveSeenData = true;
        lastFrameMs = millis();
        sendPacket(true); // immediate update
      }
    }
  }

  // Heartbeat + timeout
  if (millis() - lastHeartbeatMs > HEARTBEAT_MS) {
    if (haveSeenData && (millis() - lastFrameMs > FRAME_TIMEOUT_MS)) {
      haveSeenData = false; leftStatus = 0; rightStatus = 0; sendPacket(true);
    } else {
      sendPacket(false); // only if changed
    }
    lastHeartbeatMs = millis();
  }
}
