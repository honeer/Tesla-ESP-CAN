/*
 * ESP32 ESP-NOW Transmitter (TX) - FAST
 * - Channel locked to 6 (match RX AP)
 * - Heartbeat every 120 ms
 * - Immediate send on CAN state change for low latency
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <SPI.h>
#include "mcp_can.h"

// ---------- MCP2515 ----------
#define MCP2515_CS_PIN   15
#define MCP2515_INT_PIN  4
#define CAN_ID_VCFRONT_LIGHTING 0x3F5
#define CAN_BAUD   CAN_500KBPS
#define CAN_CLK    MCP_8MHZ

MCP_CAN CAN0(MCP2515_CS_PIN);

// ---------- ESP-NOW ----------
static const uint8_t WIFI_CHANNEL = 6; // MUST match RX
static const uint8_t PEER_BROADCAST[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

typedef struct __attribute__((packed)) {
  uint8_t left_on  : 1;
  uint8_t right_on : 1;
  uint8_t ap_on    : 1;
  uint8_t charging : 1;
  uint8_t plugged  : 1;
  uint8_t track_on : 1;
  uint8_t reserved : 2;
} StateBits;

typedef struct __attribute__((packed)) {
  uint32_t seq;
  StateBits bits;
} TxPacket;

// Timing
static const uint32_t HEARTBEAT_MS    = 120;
static const uint32_t CAN_POLL_GAP_MS = 1;

static TxPacket g_pkt{};
static StateBits g_prev{};
static uint32_t g_seq = 0;
static uint32_t g_lastSendMs = 0;
static bool     g_haveCAN = false;

static void on_espnow_sent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  (void)info; (void)status;
}

static uint64_t extract_moto_be(const uint8_t data[8], uint8_t startBit, uint8_t len) {
  uint64_t be = 0;
  for (int i = 0; i < 8; i++) be = (be << 8) | data[i];
  uint8_t shift = 64 - startBit - len;
  uint64_t mask = (len == 64) ? UINT64_MAX : ((1ULL << len) - 1ULL);
  return (be >> shift) & mask;
}

static inline bool bitsEqual(const StateBits &a, const StateBits &b){
  return *(const uint8_t*)&a == *(const uint8_t*)&b;
}

static void parse_0x3F5(const uint8_t *d, StateBits &b) {
  uint8_t leftStatus  = (uint8_t)extract_moto_be(d, 50, 2);
  uint8_t rightStatus = (uint8_t)extract_moto_be(d, 52, 2);
  b.left_on  = (leftStatus  == 1);
  b.right_on = (rightStatus == 1);
  b.ap_on = 0; b.charging = 0; b.plugged = 0; b.track_on = 0;
}

static inline void send_now() {
  g_pkt.seq = ++g_seq;
  esp_now_send(PEER_BROADCAST, (uint8_t*)&g_pkt, sizeof(g_pkt));
}

static void handle_can_once() {
  if (!g_haveCAN) return;
  if (CAN0.checkReceive() == CAN_MSGAVAIL) {
    long unsigned id = 0; byte len = 0; uint8_t buf[8] = {0};
    if (CAN0.readMsgBuf(&id, &len, buf) == CAN_OK) {
      if ((id & 0x7FF) == CAN_ID_VCFRONT_LIGHTING && len == 8) {
        StateBits nb = g_pkt.bits;
        parse_0x3F5(buf, nb);
        if (!bitsEqual(nb, g_pkt.bits)) {
          g_pkt.bits = nb;
          send_now();            // immediate on change
          g_lastSendMs = millis();
        }
      }
    }
  }
}

void setup() {
  Serial.begin(115200); delay(30);

  WiFi.mode(WIFI_STA);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed"); while (1) delay(1000);
  }
  esp_now_register_send_cb(on_espnow_sent);

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, PEER_BROADCAST, 6);
  peer.ifidx = WIFI_IF_STA;
  peer.channel = WIFI_CHANNEL;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Add peer failed"); while (1) delay(1000);
  }

  SPI.begin();
  if (CAN0.begin(MCP_ANY, CAN_BAUD, CAN_CLK) == CAN_OK) {
    CAN0.setMode(MCP_NORMAL);
    CAN0.init_Mask(0, 0, 0x7FF); CAN0.init_Filt(0, 0, CAN_ID_VCFRONT_LIGHTING);
    CAN0.init_Mask(1, 0, 0x7FF); CAN0.init_Filt(2, 0, CAN_ID_VCFRONT_LIGHTING);
    g_haveCAN = true;
    Serial.println("MCP2515 ready");
  } else {
    Serial.println("MCP2515 init FAILED (will still heartbeat zeros)");
  }

  memset(&g_pkt, 0, sizeof(g_pkt));
  g_prev = g_pkt.bits;
  Serial.printf("TX FAST ready, ch=%u\n", WIFI_CHANNEL);
}

void loop() {
  handle_can_once();

  uint32_t now = millis();
  if (now - g_lastSendMs >= HEARTBEAT_MS) {
    send_now();                   // heartbeat
    g_lastSendMs = now;
  }

  delay(CAN_POLL_GAP_MS);
}
