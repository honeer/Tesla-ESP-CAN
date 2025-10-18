/*
 * ESP32 ESP-NOW RX -> Dual Ambient LED Strips + Hotspot Web UI (v6d FULL)
 * - Hotspot (AP) + Web UI
 * - Color picker, Brightness slider
 * - Per-side strip lengths & alert tail sizes
 * - Enable/disable displayed effects (master + per-effect)
 * - Startup animations (None / Fade / Wipe / Sparkle)
 * - Change GPIO pins for Left/Right strips from the UI (live re-init)
 * - Test Panel (simulate states without the car/TX)
 * - Responsiveness presets (Chill / Normal / Snappy)
 * - TX alive debug: seq/age/RSSI; serial debug + packet tick dots
 * - ESP-NOW (IDF v5+ signature)
 *
 * Notes:
 * - Designed to match the TX packet { uint32_t seq; StateBits bits; } via ESP-NOW.
 * - Uses uint8_t constants for alert priority to avoid Arduino prototype issues.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Adafruit_NeoPixel.h>
#include <WebServer.h>
#include <Preferences.h>

/************ Runtime-configurable Pins & LED type ************/
static uint8_t  g_pinLeft  = 22;
static uint8_t  g_pinRight = 21;
static const neoPixelType LED_TYPE = NEO_GRB + NEO_KHZ800; // WS2812B

/************ Defaults (editable in UI) ************/
static const uint16_t DEFAULT_LEDS_LEFT   = 67; // set to your real left length
static const uint16_t DEFAULT_LEDS_RIGHT  = 50; // set to your real right length
static const uint8_t  DEFAULT_TAIL_LEFT   = 6;  // last LEDs used to show alerts
static const uint8_t  DEFAULT_TAIL_RIGHT  = 6;
static const uint8_t  DEFAULT_BRIGHTNESS  = 100;

/************ Wi-Fi AP + Web ************/
static const char* AP_SSID     = "AmbientLights";
static const char* AP_PASSWORD = "12345678";
static const uint8_t AP_CHANNEL = 6; // keep in sync with TX channel
WebServer server(80);
Preferences prefs;

/************ Colors ************/
struct RGB { uint8_t r,g,b; };
static RGB ambientColor       = { 60, 0, 180 }; // violet-y blue (Mercedes vibe)
static RGB TURN_COLOR         = {255,128,  0};
static RGB AUTOPILOT_COLOR    = {  0, 64,255};
static RGB CHARGING_COLOR     = {  0,255,  0};
static RGB PLUGGED_COLOR      = { 32, 32, 32};
static RGB TRACK_COLOR        = {255,  0, 64};

/************ Packet from TX over ESP-NOW ************/
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
} RxPacket;

/************ Per-side turn smoothing ************/
struct TurnSide { bool active=false; uint32_t lastSeenMs=0; };

/************ Effects enable flags ************/
struct EffectEn {
  bool alerts   = true; // master
  bool turn     = true;
  bool charging = true;
  bool ap       = true;
  bool track    = true;
  bool plugged  = true;
} g_eff;

/************ Startup animation ************/
enum StartupMode : uint8_t { START_NONE=0, START_FADE=1, START_WIPE=2, START_SPARKLE=3 };
static StartupMode g_startMode = START_FADE;

/************ Responsiveness preset ************/
enum RespPreset : uint8_t { RESP_CHILL=0, RESP_NORMAL=1, RESP_SNAPPY=2 };
static uint8_t g_respPreset = RESP_NORMAL;
// Runtime-adjustable timing
static uint16_t g_turnPulseMs    = 600;
static uint16_t g_statusPulseMs  = 1000;
static uint16_t g_frameMs        = 10;   // ~100 FPS
// Smoothing for brief dropouts (turn)
static uint16_t g_turnLossHoldMs = 200;

static void applyResponsiveness(uint8_t rp){
  g_respPreset = rp;
  switch (rp){
    case RESP_CHILL:  g_turnPulseMs=800; g_statusPulseMs=1200; g_frameMs=20; g_turnLossHoldMs=250; break;
    case RESP_SNAPPY: g_turnPulseMs=450; g_statusPulseMs=800;  g_frameMs=8;  g_turnLossHoldMs=180; break;
    case RESP_NORMAL:
    default:          g_turnPulseMs=600; g_statusPulseMs=1000; g_frameMs=10; g_turnLossHoldMs=200; break;
  }
}

/************ Priority constants (uint8_t for Arduino-proto safety) ************/
static const uint8_t P_TURN=0, P_CHARGING=1, P_AUTOPILOT=2, P_TRACK=3, P_PLUGGED=4, P_NONE=255;

/************ Globals ************/
static RxPacket g_lastPkt = {0};
static volatile bool g_hasPkt = false;
static portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
static int g_lastRSSI = 0;
static bool g_debugSerial=false, g_pktTicks=false;
static uint8_t g_brightness = DEFAULT_BRIGHTNESS;
static uint16_t g_lenLeft=DEFAULT_LEDS_LEFT, g_lenRight=DEFAULT_LEDS_RIGHT;
static uint8_t g_tailLeft=DEFAULT_TAIL_LEFT, g_tailRight=DEFAULT_TAIL_RIGHT;
static uint32_t g_lastPktMs=0;
static const uint32_t TX_ALIVE_WINDOW_MS = 1500;

Adafruit_NeoPixel stripL(g_lenLeft, /*pin*/g_pinLeft, LED_TYPE);
Adafruit_NeoPixel stripR(g_lenRight, /*pin*/g_pinRight, LED_TYPE);

TurnSide g_leftTurn, g_rightTurn;

/************ Test Panel (simulation) ************/
static bool g_testMode = false;
static StateBits g_testBits = {0};

/************ Utils ************/
static inline uint32_t toColor(const RGB &c){ return ((uint32_t)c.r<<16)|((uint32_t)c.g<<8)|c.b; }
static void applyBrightness(){ stripL.setBrightness(g_brightness); stripR.setBrightness(g_brightness); }

static void applyStripConfig(){
  stripL.updateLength(g_lenLeft);
  stripR.updateLength(g_lenRight);
  stripL.setPin(g_pinLeft);
  stripR.setPin(g_pinRight);
  stripL.begin(); stripR.begin();
  applyBrightness();
  stripL.show(); stripR.show();
}

static void fillAmbient(Adafruit_NeoPixel &s){
  uint32_t col=toColor(ambientColor);
  for(uint16_t i=0;i<s.numPixels();++i) s.setPixelColor(i,col);
}

static void fillTail(Adafruit_NeoPixel &s,const RGB &c,uint8_t tail){
  uint32_t col=toColor(c); int16_t n=s.numPixels(); int16_t st=n-tail; if(st<0) st=0;
  for(int16_t i=st;i<n;++i) s.setPixelColor((uint16_t)i,col);
}

static float pulseFactor(uint32_t ms,uint16_t period){
  float ph=(ms%period)/(float)period; float v=0.5f*(1.0f+sinf(ph*6.2831853f)); return 0.35f+0.65f*v;
}
static RGB scaleRGB(const RGB &c,float f){ RGB o; o.r=(uint8_t)constrain((int)(c.r*f),0,255); o.g=(uint8_t)constrain((int)(c.g*f),0,255); o.b=(uint8_t)constrain((int)(c.b*f),0,255); return o; }

static void updateTurnSide(TurnSide &ts,bool rawOn,uint32_t now){
  if(rawOn){ts.active=true; ts.lastSeenMs=now;} else if(ts.active && (now-ts.lastSeenMs>g_turnLossHoldMs)) ts.active=false;
}

/************ Priority helpers ************/
static uint8_t pickPriority(bool turnActive,const StateBits &b){
  if(!g_eff.alerts) return P_NONE;
  if(turnActive && g_eff.turn)    return P_TURN;
  if(b.charging && g_eff.charging)return P_CHARGING;
  if(b.ap_on   && g_eff.ap)       return P_AUTOPILOT;
  if(b.track_on&& g_eff.track)    return P_TRACK;
  if(b.plugged && g_eff.plugged)  return P_PLUGGED;
  return P_NONE;
}

static void applySidePulse(Adafruit_NeoPixel &s, uint8_t pr, uint32_t now, uint8_t tail){
  fillAmbient(s); if(pr==P_NONE) return;
  RGB col; uint16_t period=g_statusPulseMs;
  switch(pr){case P_TURN: col=TURN_COLOR; period=g_turnPulseMs; break;
             case P_CHARGING: col=CHARGING_COLOR; break;
             case P_AUTOPILOT: col=AUTOPILOT_COLOR; break;
             case P_TRACK: col=TRACK_COLOR; break;
             case P_PLUGGED: col=PLUGGED_COLOR; break;
             default:return;}
  float f=pulseFactor(now,period); fillTail(s, scaleRGB(col,f), tail);
}

/************ Startup animations ************/
static void startupFade(){
  for(int i=0;i<=100;i+=4){
    float f=i/100.0f;
    RGB c=scaleRGB(ambientColor, f);
    uint32_t col=toColor(c);
    for(uint16_t n=0;n<stripL.numPixels();++n) stripL.setPixelColor(n,col);
    for(uint16_t n=0;n<stripR.numPixels();++n) stripR.setPixelColor(n,col);
    stripL.show(); stripR.show(); delay(10);
  }
}
static void startupWipe(){
  fillAmbient(stripL); fillAmbient(stripR);
  uint16_t nL=stripL.numPixels(), nR=stripR.numPixels();
  for(uint16_t k=0;k<max(nL,nR);++k){
    if(k<nL) stripL.setPixelColor(k, toColor(ambientColor));
    if(k<nR) stripR.setPixelColor(k, toColor(ambientColor));
    stripL.show(); stripR.show(); delay(6);
  }
}
static void startupSparkle(){
  fillAmbient(stripL); fillAmbient(stripR);
  for(int t=0;t<180;t++){
    uint16_t iL = esp_random() % max<uint16_t>(1, stripL.numPixels());
    uint16_t iR = esp_random() % max<uint16_t>(1, stripR.numPixels());
    stripL.setPixelColor(iL, toColor(scaleRGB(ambientColor, 1.6f)));
    stripR.setPixelColor(iR, toColor(scaleRGB(ambientColor, 1.6f)));
    stripL.show(); stripR.show(); delay(8);
    stripL.setPixelColor(iL, toColor(ambientColor));
    stripR.setPixelColor(iR, toColor(ambientColor));
  }
}

static void runStartup(){
  switch(g_startMode){
    case START_FADE:    startupFade(); break;
    case START_WIPE:    startupWipe(); break;
    case START_SPARKLE: startupSparkle(); break;
    default: break;
  }
}

/************ ESP-NOW ************/
static void on_data_recv(const esp_now_recv_info *info, const uint8_t *incoming, int len){
  if(len==(int)sizeof(RxPacket)){
    RxPacket tmp; memcpy(&tmp,incoming,sizeof(tmp));
    portENTER_CRITICAL(&g_mux); g_lastPkt=tmp; g_hasPkt=true; portEXIT_CRITICAL(&g_mux);
    g_lastPktMs=millis();
    if(info && info->rx_ctrl) g_lastRSSI=(int)info->rx_ctrl->rssi;
    if(g_debugSerial){ Serial.printf("NOW seq=%lu L=%u R=%u RSSI=%d\n",(unsigned long)tmp.seq,tmp.bits.left_on,tmp.bits.right_on,g_lastRSSI); }
    if(g_pktTicks) Serial.print('.');
  }
}

static bool setupEspNow(){
  if(esp_now_init()!=ESP_OK) return false;
  esp_now_register_recv_cb(on_data_recv);
  return true;
}

/************ Web UI ************/
static void handleRoot(){
  char colorHex[16]; sprintf(colorHex, "#%02X%02X%02X", ambientColor.r, ambientColor.g, ambientColor.b);
  String html = F("<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'/>"
    "<style>body{background:#111;color:#eee;font-family:sans-serif;margin:20px}"
    ".grid{display:grid;grid-template-columns:1fr;gap:14px}"
    ".card{background:#1b1b1b;border-radius:12px;padding:16px;box-shadow:0 2px 10px rgba(0,0,0,.4)}"
    "label{display:block;margin:8px 0 6px} button{padding:8px 14px;border:0;border-radius:8px;background:#2e72ff;color:#fff}"
    "input[type=color]{width:120px;height:50px;border:0;background:none}"
    "input[type=range]{width:100%} input[type=number]{width:90px;padding:6px;border-radius:8px;border:0;}"
    "select{width:100%;padding:6px;border-radius:8px;border:0;background:#222;color:#eee}"
    ".row{display:flex;gap:10px;flex-wrap:wrap;align-items:center}"
    ".dot{display:inline-block;width:10px;height:10px;border-radius:50%;margin-left:6px;background:#f33}"
    "</style></head><body>");
  html += F("<div class='grid'>");

  // Ambient
  html += F("<div class='card'><h2>Ambient Lights</h2><form action='/set' method='get'>");
  html += F("<label>Color</label><input type='color' name='c' value='"); html += colorHex;
  html += F("'> <input type='checkbox' name='save' value='1' checked> Save");
  html += F("<label>Brightness</label><input type='range' min='0' max='255' name='b' id='b' value='");
  html += String(g_brightness);
  html += F("' oninput='bv.innerText=this.value'><div>Value: <b id='bv'>"); html += String(g_brightness); html += F("</b></div>");
  html += F("<div style='margin-top:12px'><button type='submit'>Apply</button></div></form></div>");

  // Strip & Tail
  html += F("<div class='card'><h2>Strip & Tail</h2><form action='/lengths' method='get'>");
  html += F("<label>Left strip LEDs</label><input type='number' min='1' max='600' name='ll' value='"); html += String(g_lenLeft); html += F("'>");
  html += F("<label>Right strip LEDs</label><input type='number' min='1' max='600' name='lr' value='"); html += String(g_lenRight); html += F("'>");
  html += F("<label>Left tail LEDs</label><input type='number' min='0' max='255' name='tl' value='"); html += String(g_tailLeft); html += F("'>");
  html += F("<label>Right tail LEDs</label><input type='number' min='0' max='255' name='tr' value='"); html += String(g_tailRight); html += F("'>");
  html += F("<div style='margin-top:12px'><button type='submit'>Apply</button></div></form></div>");

  // Effects
  html += F("<div class='card'><h2>Displayed Effects</h2><form action='/effects' method='get'>");
  html += F("<label><input type='checkbox' name='alerts' value='1' "); if(g_eff.alerts) html += F("checked"); html += F("> Enable Alerts (master)</label>");
  html += F("<label><input type='checkbox' name='turn' value='1' "); if(g_eff.turn) html += F("checked"); html += F("> Turn Signals</label>");
  html += F("<label><input type='checkbox' name='chg' value='1' "); if(g_eff.charging) html += F("checked"); html += F("> Charging</label>");
  html += F("<label><input type='checkbox' name='ap' value='1' "); if(g_eff.ap) html += F("checked"); html += F("> Autopilot</label>");
  html += F("<label><input type='checkbox' name='trk' value='1' "); if(g_eff.track) html += F("checked"); html += F("> Track Mode</label>");
  html += F("<label><input type='checkbox' name='plg' value='1' "); if(g_eff.plugged) html += F("checked"); html += F("> Plugged</label>");
  html += F("<div style='margin-top:12px'><button type='submit'>Save</button></div></form></div>");

  // Startup, Pins, Responsiveness
  html += F("<div class='card'><h2>Startup, Pins & Responsiveness</h2><form action='/startpins' method='get'>");
  html += F("<label>Startup Animation</label><select name='sm'>");
  html += F("<option value='0'"); if(g_startMode==START_NONE) html += F(" selected"); html += F(">None</option>");
  html += F("<option value='1'"); if(g_startMode==START_FADE) html += F(" selected"); html += F(">Fade In</option>");
  html += F("<option value='2'"); if(g_startMode==START_WIPE) html += F(" selected"); html += F(">Wipe</option>");
  html += F("<option value='3'"); if(g_startMode==START_SPARKLE) html += F(" selected"); html += F(">Sparkle</option>");
  html += F("</select>");
  html += F("<label>Left Pin</label><input type='number' min='0' max='39' name='pl' value='"); html += String(g_pinLeft); html += F("'>");
  html += F("<label>Right Pin</label><input type='number' min='0' max='39' name='pr' value='"); html += String(g_pinRight); html += F("'>");
  html += F("<label>Responsiveness</label><select name='rp'>");
  html += F("<option value='0'"); if(g_respPreset==RESP_CHILL) html += F(" selected"); html += F(">Chill</option>");
  html += F("<option value='1'"); if(g_respPreset==RESP_NORMAL) html += F(" selected"); html += F(">Normal</option>");
  html += F("<option value='2'"); if(g_respPreset==RESP_SNAPPY) html += F(" selected"); html += F(">Snappy</option>");
  html += F("</select>");
  html += F("<div style='margin-top:12px'><button type='submit'>Apply</button></div><small>Pins take effect immediately (strips re-init).</small></form></div>");

  // Test Panel
  html += F("<div class='card'><h2>Test Panel</h2><form action='/test' method='get'>");
  html += F("<div class='row'><label><input type='checkbox' name='tm' value='1' "); if(g_testMode) html += F("checked"); html += F("> Enable Test Mode (override TX)</label></div>");
  html += F("<div class='row'><label><input type='checkbox' name='l' value='1' "); if(g_testBits.left_on) html += F("checked"); html += F("> Left</label>");
  html += F("<label><input type='checkbox' name='r' value='1' "); if(g_testBits.right_on) html += F("checked"); html += F("> Right</label>");
  html += F("<label><input type='checkbox' name='ap' value='1' "); if(g_testBits.ap_on) html += F("checked"); html += F("> Autopilot</label>");
  html += F("<label><input type='checkbox' name='chg' value='1' "); if(g_testBits.charging) html += F("checked"); html += F("> Charging</label>");
  html += F("<label><input type='checkbox' name='plg' value='1' "); if(g_testBits.plugged) html += F("checked"); html += F("> Plugged</label>");
  html += F("<label><input type='checkbox' name='trk' value='1' "); if(g_testBits.track_on) html += F("checked"); html += F("> Track</label></div>");
  html += F("<div style='margin-top:12px'><button type='submit'>Apply</button></div>");
  html += F("<small>When Test Mode is ON, the UI selections above are used instead of the TX packet.</small></form></div>");

  // Status & Debug
  html += F("<div class='card'><h2>Status <span id='aliveDot' class='dot'></span></h2><pre id='stat'>loading...</pre>"
            "<button onclick='fetchStatus()'>Refresh</button></div>"
            "<div class='card'><h2>Debug</h2>"
            "<button onclick='toggleDebug(1)'>Enable Serial Debug</button> "
            "<button onclick='toggleDebug(0)'>Disable</button> "
            "<button onclick='togglePkt(1)'>Packet Ticks ON</button> "
            "<button onclick='togglePkt(0)'>Packet Ticks OFF</button></div>");

  html += F("</div><script>"
            "async function fetchStatus(){"
            "  let r=await fetch('/status'); let t=await r.text();"
            "  document.getElementById('stat').textContent=t;"
            "  let alive = t.includes('txAlive: yes');"
            "  document.getElementById('aliveDot').style.background = alive ? '#0f0' : '#f33';"
            "}"
            "fetchStatus(); setInterval(fetchStatus, 1000);"
            "async function toggleDebug(en){await fetch('/debug?en='+en);}"
            "async function togglePkt(en){await fetch('/pktlog?en='+en);}"
            "</script></body></html>");
  server.send(200, "text/html", html);
}

static void handleSet(){
  bool wantSave = server.hasArg("save");
  bool updated=false;
  if (server.hasArg("c")) {
    String hex=server.arg("c"); if(hex.startsWith("#")) hex.remove(0,1);
    if(hex.length()==6){ long v=strtol(hex.c_str(),NULL,16);
      ambientColor.r=(v>>16)&0xFF; ambientColor.g=(v>>8)&0xFF; ambientColor.b=v&0xFF; updated=true;
      if(g_debugSerial) Serial.printf("Ambient set R=%u G=%u B=%u\n",ambientColor.r,ambientColor.g,ambientColor.b);
    }
  }
  if (server.hasArg("b")) {
    int b=server.arg("b").toInt(); b=constrain(b,0,255); g_brightness=(uint8_t)b; applyBrightness(); updated=true;
    if(g_debugSerial) Serial.printf("Brightness %u\n",g_brightness);
  }
  if (wantSave && updated){
    prefs.begin("ambient", false);
    prefs.putUChar("r", ambientColor.r);
    prefs.putUChar("g", ambientColor.g);
    prefs.putUChar("b", ambientColor.b);
    prefs.putUChar("br", g_brightness);
    prefs.end();
  }
  handleRoot();
}

static void handleLengths(){
  uint16_t ll=g_lenLeft, lr=g_lenRight; uint8_t tl=g_tailLeft, tr=g_tailRight;
  if(server.hasArg("ll")) ll=(uint16_t)constrain(server.arg("ll").toInt(),1,600);
  if(server.hasArg("lr")) lr=(uint16_t)constrain(server.arg("lr").toInt(),1,600);
  if(server.hasArg("tl")) tl=(uint8_t)constrain(server.arg("tl").toInt(),0,255);
  if(server.hasArg("tr")) tr=(uint8_t)constrain(server.arg("tr").toInt(),0,255);
  bool changed=(ll!=g_lenLeft)||(lr!=g_lenRight)||(tl!=g_tailLeft)||(tr!=g_tailRight);
  if(changed){
    g_lenLeft=ll; g_lenRight=lr; g_tailLeft=tl; g_tailRight=tr;
    applyStripConfig();
    if(g_debugSerial) Serial.printf("Lengths L=%u R=%u tails TL=%u TR=%u\n",g_lenLeft,g_lenRight,g_tailLeft,g_tailRight);
    prefs.begin("ambient", false);
    prefs.putUShort("lenL", g_lenLeft); prefs.putUShort("lenR", g_lenRight);
    prefs.putUChar("tailL", g_tailLeft); prefs.putUChar("tailR", g_tailRight);
    prefs.end();
  }
  handleRoot();
}

static void handleEffects(){
  g_eff.alerts   = server.hasArg("alerts");
  g_eff.turn     = server.hasArg("turn");
  g_eff.charging = server.hasArg("chg");
  g_eff.ap       = server.hasArg("ap");
  g_eff.track    = server.hasArg("trk");
  g_eff.plugged  = server.hasArg("plg");

  prefs.begin("ambient", false);
  prefs.putBool("eAlerts", g_eff.alerts);
  prefs.putBool("eTurn",   g_eff.turn);
  prefs.putBool("eChg",    g_eff.charging);
  prefs.putBool("eAP",     g_eff.ap);
  prefs.putBool("eTrk",    g_eff.track);
  prefs.putBool("ePlg",    g_eff.plugged);
  prefs.end();

  handleRoot();
}

static void handleStartPins(){
  // Startup mode
  if(server.hasArg("sm")){
    uint8_t m=(uint8_t)constrain(server.arg("sm").toInt(),0,3);
    g_startMode=(StartupMode)m;
    prefs.begin("ambient", false); prefs.putUChar("startMode", m); prefs.end();
  }
  // Pins
  bool pinChanged=false;
  if(server.hasArg("pl")){
    uint8_t pl=(uint8_t)constrain(server.arg("pl").toInt(),0,39);
    if(pl!=g_pinLeft){ g_pinLeft=pl; pinChanged=true; }
  }
  if(server.hasArg("pr")){
    uint8_t pr=(uint8_t)constrain(server.arg("pr").toInt(),0,39);
    if(pr!=g_pinRight){ g_pinRight=pr; pinChanged=true; }
  }
  if(pinChanged){
    prefs.begin("ambient", false);
    prefs.putUChar("pinL", g_pinLeft);
    prefs.putUChar("pinR", g_pinRight);
    prefs.end();
    applyStripConfig();
    if(g_debugSerial) Serial.printf("Pins updated L=%u R=%u\n", g_pinLeft, g_pinRight);
  }
  // Responsiveness preset
  if(server.hasArg("rp")){
    uint8_t rp=(uint8_t)constrain(server.arg("rp").toInt(),0,2);
    applyResponsiveness(rp);
    prefs.begin("ambient", false); prefs.putUChar("resp", g_respPreset); prefs.end();
    if(g_debugSerial) Serial.printf("Responsiveness preset %u applied\n", g_respPreset);
  }

  handleRoot();
}

static void handleTest(){
  // Test mode enable
  bool tm = server.hasArg("tm");
  g_testMode = tm;
  // States (if omitted, off)
  g_testBits.left_on  = server.hasArg("l");
  g_testBits.right_on = server.hasArg("r");
  g_testBits.ap_on    = server.hasArg("ap");
  g_testBits.charging = server.hasArg("chg");
  g_testBits.plugged  = server.hasArg("plg");
  g_testBits.track_on = server.hasArg("trk");

  // Persist
  prefs.begin("ambient", false);
  prefs.putBool("tMode", g_testMode);
  prefs.putUChar("tL", g_testBits.left_on);
  prefs.putUChar("tR", g_testBits.right_on);
  prefs.putUChar("tAP", g_testBits.ap_on);
  prefs.putUChar("tCH", g_testBits.charging);
  prefs.putUChar("tPL", g_testBits.plugged);
  prefs.putUChar("tTR", g_testBits.track_on);
  prefs.end();

  handleRoot();
}

static void handleStatus(){
  RxPacket snap; bool has; portENTER_CRITICAL(&g_mux); snap=g_lastPkt; has=g_hasPkt; portEXIT_CRITICAL(&g_mux);
  uint32_t now=millis(); uint32_t age=(g_lastPktMs==0)?0:(now-g_lastPktMs); bool txAlive=(g_lastPktMs!=0)&&(age<=TX_ALIVE_WINDOW_MS);
  char out[850];
  snprintf(out,sizeof(out),
    "ambient: R=%u G=%u B=%u\n"
    "brightness: %u\n"
    "lengths: left=%u right=%u  tails: left=%u right=%u\n"
    "pins: left=%u right=%u\n"
    "effects: master=%s turn=%s chg=%s ap=%s trk=%s plg=%s\n"
    "startup: %u  responsiveness: %u (turn=%ums status=%ums frame=%ums)\n"
    "testMode: %s  testBits: L=%u R=%u AP=%u CHG=%u PLG=%u TRK=%u\n"
    "hasPacket: %s  seq: %lu  bits: L=%u R=%u AP=%u CHG=%u PLUG=%u TRK=%u\n"
    "rssi: %d dBm  txAlive: %s  lastPktAge: %lu ms\n"
    "uptime: %lu ms  debugSerial: %s  pktTicks: %s\n",
    ambientColor.r, ambientColor.g, ambientColor.b,
    g_brightness,
    g_lenLeft, g_lenRight, g_tailLeft, g_tailRight,
    g_pinLeft, g_pinRight,
    g_eff.alerts?"on":"off", g_eff.turn?"on":"off", g_eff.charging?"on":"off", g_eff.ap?"on":"off", g_eff.track?"on":"off", g_eff.plugged?"on":"off",
    (unsigned)g_startMode, (unsigned)g_respPreset, g_turnPulseMs, g_statusPulseMs, g_frameMs,
    g_testMode?"on":"off", g_testBits.left_on, g_testBits.right_on, g_testBits.ap_on, g_testBits.charging, g_testBits.plugged, g_testBits.track_on,
    has?"yes":"no", (unsigned long)snap.seq, snap.bits.left_on, snap.bits.right_on, snap.bits.ap_on, snap.bits.charging, snap.bits.plugged, snap.bits.track_on,
    g_lastRSSI, txAlive?"yes":"no", (unsigned long)age,
    (unsigned long)now, g_debugSerial?"on":"off", g_pktTicks?"on":"off"
  );
  server.send(200,"text/plain",out);
}

static void handleDebug(){ if(server.hasArg("en")) g_debugSerial=(server.arg("en")=="1"); handleStatus(); }
static void handlePktLog(){ if(server.hasArg("en")) g_pktTicks=(server.arg("en")=="1"); handleStatus(); }

/************ Setup & Loop ************/
static void setupAPAndWeb(){
  WiFi.mode(WIFI_AP);
  bool ok=WiFi.softAP(AP_SSID, AP_PASSWORD, AP_CHANNEL);
  delay(100);
  IPAddress ip=WiFi.softAPIP();
  Serial.printf("AP %s %s on ch %u IP %s\n", ok?"started":"FAILED", AP_SSID, AP_CHANNEL, ip.toString().c_str());

  if(!setupEspNow()) Serial.println("ESP-NOW init failed!");

  // Load prefs
  prefs.begin("ambient", true);
  ambientColor.r = prefs.getUChar("r", ambientColor.r);
  ambientColor.g = prefs.getUChar("g", ambientColor.g);
  ambientColor.b = prefs.getUChar("b", ambientColor.b);
  g_brightness   = prefs.getUChar("br", DEFAULT_BRIGHTNESS);
  g_lenLeft      = prefs.getUShort("lenL", DEFAULT_LEDS_LEFT);
  g_lenRight     = prefs.getUShort("lenR", DEFAULT_LEDS_RIGHT);
  g_tailLeft     = prefs.getUChar("tailL", DEFAULT_TAIL_LEFT);
  g_tailRight    = prefs.getUChar("tailR", DEFAULT_TAIL_RIGHT);
  g_pinLeft      = prefs.getUChar("pinL", g_pinLeft);
  g_pinRight     = prefs.getUChar("pinR", g_pinRight);
  g_startMode    = (StartupMode)prefs.getUChar("startMode", (uint8_t)g_startMode);
  g_eff.alerts   = prefs.getBool("eAlerts", true);
  g_eff.turn     = prefs.getBool("eTurn",   true);
  g_eff.charging = prefs.getBool("eChg",    true);
  g_eff.ap       = prefs.getBool("eAP",     true);
  g_eff.track    = prefs.getBool("eTrk",    true);
  g_eff.plugged  = prefs.getBool("ePlg",    true);
  g_respPreset   = prefs.getUChar("resp", (uint8_t)g_respPreset);
  g_testMode     = prefs.getBool("tMode", false);
  g_testBits.left_on  = prefs.getUChar("tL", 0);
  g_testBits.right_on = prefs.getUChar("tR", 0);
  g_testBits.ap_on    = prefs.getUChar("tAP",0);
  g_testBits.charging = prefs.getUChar("tCH",0);
  g_testBits.plugged  = prefs.getUChar("tPL",0);
  g_testBits.track_on = prefs.getUChar("tTR",0);
  prefs.end();

  applyResponsiveness(g_respPreset);
  applyStripConfig();

  // Web handlers
  server.on("/",          handleRoot);
  server.on("/set",       handleSet);
  server.on("/lengths",   handleLengths);
  server.on("/effects",   handleEffects);
  server.on("/startpins", handleStartPins);
  server.on("/test",      handleTest);
  server.on("/status",    handleStatus);
  server.on("/debug",     handleDebug);
  server.on("/pktlog",    handlePktLog);
  server.begin();
  Serial.println("Web server started");

  runStartup();
}

void setup(){
  Serial.begin(115200);
  stripL.begin(); stripR.begin(); applyBrightness(); stripL.show(); stripR.show();
  setupAPAndWeb();
}

void loop(){
  server.handleClient();

  uint32_t now=millis();
  static uint32_t lastFrameMs=0;
  if(now-lastFrameMs<g_frameMs) return;
  lastFrameMs=now;

  // Snapshot of bits: either from test mode or from last packet
  StateBits srcBits;
  bool has;
  if (g_testMode) {
    srcBits = g_testBits;
    has = true;
  } else {
    RxPacket snap;
    portENTER_CRITICAL(&g_mux); snap=g_lastPkt; has=g_hasPkt; portEXIT_CRITICAL(&g_mux);
    srcBits = snap.bits;
  }

  if(!has){
    fillAmbient(stripL); fillAmbient(stripR);
    stripL.show(); stripR.show();
    return;
  }

  updateTurnSide(g_leftTurn,  srcBits.left_on,  now);
  updateTurnSide(g_rightTurn, srcBits.right_on, now);

  uint8_t prL=pickPriority(g_leftTurn.active, srcBits);
  uint8_t prR=pickPriority(g_rightTurn.active, srcBits);

  applySidePulse(stripL, prL, now, g_tailLeft);
  applySidePulse(stripR, prR, now, g_tailRight);

  stripL.show(); stripR.show();
}
