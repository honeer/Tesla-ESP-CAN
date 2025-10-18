/*
  ESP32 WS2812B (NeoPixel) Controller — 5 Strips
  ------------------------------------------------
  - Starts a Wi‑Fi hotspot (AP mode) with an onboard web portal
  - Control per‑strip color, presets, animations, startup animation
  - Settings panel for pins, lengths, direction, brightness (per strip)
  - Status/Debug dots (placeholders), Testing utilities
  - Designed to be easy to extend with more animations & UI later

  Hardware:
  - ESP32 (any common dev board)
  - Five WS2812B strips (5V) — use a common GND with ESP32
  - Level shifter recommended for data lines (3.3V->5V)

  Library Dependencies (install via Library Manager):
  - ESP Async WebServer by me-no-dev (and AsyncTCP)
  - Adafruit NeoPixel by Adafruit
  - ArduinoJson by Benoit Blanchon

  Optional, but recommended:
  - Enable PSRAM if available for larger pages.

  Notes on the Logo:
  - By default, the portal loads /logo.png from SPIFFS.
  - To replace the logo, upload your PNG to SPIFFS as /logo.png (Tools → ESP32 Sketch Data Upload, or any SPIFFS uploader).
  - If /logo.png is missing, the page shows a text fallback.
*/

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Adafruit_NeoPixel.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <SPIFFS.h>

// ===================== USER‑TWEAKABLE DEFAULTS =====================
// AP (hotspot) credentials
static const char* AP_SSID     = "ESP32-LED-Controller";
static const char* AP_PASSWORD = "esp32leds";   // 8+ chars required

// Friendly names for the 5 strips (used in UI)
static const char* STRIP_NAMES[5] = {
  "Dashboard Left",
  "Dashboard Right",
  "Left Door",
  "Right Door",
  "Extra"
};

// Reasonable default settings (can be changed in the UI and saved)
struct StripSettings {
  int pin = 2;           // default pin
  uint16_t length = 30;  // number of LEDs
  bool reverse = false;  // direction (false=normal, true=reversed)
  uint8_t brightness = 50; // 0-100 percent
};

struct ControllerSettings {
  StripSettings strips[5];
  uint8_t  startupAnimationId = 0;   // which animation to run at boot
  uint16_t startupDurationSec = 5;   // how long to run startup animation
  uint16_t startupFadeMs      = 1000; // fade time from startup anim to ambient
};

// Initial pin suggestions matching the names above — change as needed
static const int DEFAULT_PINS[5] = { 16, 17, 18, 19, 21 };

// ===================== GLOBALS =====================
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
Preferences prefs;

// We create the Adafruit_NeoPixel objects *dynamically* so we can rebuild them
// if the user changes length or pin from the Settings panel.
Adafruit_NeoPixel* strips[5] = {nullptr, nullptr, nullptr, nullptr, nullptr};

// Runtime color (per strip). Stored as packed 0xRRGGBB.
uint32_t runtimeColor[5] = {0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF};

// Per-strip selected animation ID (index into registry below)
uint8_t activeAnimation[5] = {0, 0, 0, 0, 0};

// Startup control & fade management
bool     startupActive = false;
uint32_t startupDeadline = 0;   // when to end startup animation
uint8_t  fadePhase = 0;         // 0=none,1=down,2=up
uint32_t fadeStart = 0;         // millis when current fade phase started
uint8_t  globalDimmer = 100;    // extra global brightness % applied during fades

// Animation timekeeping/state per strip
struct AnimState {
  uint32_t lastUpdate = 0; // millis of last frame
  uint32_t step = 0;       // generic step/index
};
AnimState animState[5];

// Current settings (loaded/saved from NVS + SPIFFS JSON for human readability)
ControllerSettings settings;

// ===================== UTILS =====================
static inline uint32_t packColor(uint8_t r, uint8_t g, uint8_t b) {
  return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}
static inline uint8_t R(uint32_t c){ return (c >> 16) & 0xFF; }
static inline uint8_t G(uint32_t c){ return (c >> 8) & 0xFF; }
static inline uint8_t B(uint32_t c){ return c & 0xFF; }

// Map logical LED index → physical index based on direction
static inline uint16_t mapIndex(uint8_t s, uint16_t i) {
  if (!strips[s]) return 0;
  uint16_t len = strips[s]->numPixels();
  return settings.strips[s].reverse ? (len - 1 - i) : i;
}

// Apply brightness percent (0-100) to a color
static inline uint32_t applyBrightness(uint32_t c, uint8_t pct){
  // Combine per-strip brightness with global dimmer (for fades)
  uint16_t eff = (uint16_t)pct * (uint16_t)globalDimmer / 100;
  uint16_t r = (uint16_t)R(c) * eff / 100;
  uint16_t g = (uint16_t)G(c) * eff / 100;
  uint16_t b = (uint16_t)B(c) * eff / 100;
  return packColor(r, g, b);
}

// ===================== STRIP MANAGEMENT =====================
void destroyStrips(){
  for (int i=0;i<5;i++){
    if (strips[i]){ delete strips[i]; strips[i] = nullptr; }
  }
}

void buildStrips(){
  destroyStrips();
  for (int i=0;i<5;i++){
    strips[i] = new Adafruit_NeoPixel(settings.strips[i].length, settings.strips[i].pin, NEO_GRB + NEO_KHZ800);
    strips[i]->begin();
    strips[i]->setBrightness(map(settings.strips[i].brightness, 0, 100, 0, 255));
    strips[i]->show();
  }
}

void fillStripSolid(uint8_t s, uint32_t color){
  if (!strips[s]) return;
  uint16_t n = strips[s]->numPixels();
  uint32_t c = applyBrightness(color, settings.strips[s].brightness);
  for (uint16_t i=0;i<n;i++){
    strips[s]->setPixelColor(mapIndex(s,i), strips[s]->Color(R(c), G(c), B(c)));
  }
  strips[s]->show();
}

// ===================== ANIMATION SYSTEM =====================
// Every animation conforms to this signature
typedef void (*AnimFn)(uint8_t s, AnimState& st, uint32_t now);

// --- Animation helpers ---
void clearStrip(uint8_t s){
  if (!strips[s]) return;
  for (uint16_t i=0;i<strips[s]->numPixels();i++) strips[s]->setPixelColor(i, 0);
  strips[s]->show();
}

// 0) Solid Color (uses runtimeColor)
void animSolid(uint8_t s, AnimState& st, uint32_t now){
  // only refresh if brightness or color changed elsewhere; cheap redraw every 500ms
  if (now - st.lastUpdate < 500) return;
  st.lastUpdate = now;
  fillStripSolid(s, runtimeColor[s]);
}

// 1) Color Wipe
void animColorWipe(uint8_t s, AnimState& st, uint32_t now){
  if (now - st.lastUpdate < 25) return;
  st.lastUpdate = now;
  if (!strips[s]) return;
  uint16_t n = strips[s]->numPixels();
  uint16_t idx = st.step % (n+5);
  uint32_t c = applyBrightness(runtimeColor[s], settings.strips[s].brightness);
  if (idx == 0) clearStrip(s);
  if (idx < n){
    strips[s]->setPixelColor(mapIndex(s, idx), strips[s]->Color(R(c),G(c),B(c)));
    strips[s]->show();
  }
  st.step++;
}

// 2) Theater Chase
void animTheaterChase(uint8_t s, AnimState& st, uint32_t now){
  if (now - st.lastUpdate < 80) return;
  st.lastUpdate = now;
  if (!strips[s]) return;
  uint16_t n = strips[s]->numPixels();
  uint8_t q = st.step % 3;
  uint32_t c = applyBrightness(runtimeColor[s], settings.strips[s].brightness);
  for (uint16_t i=0;i<n;i++){
    bool on = ((i + q) % 3) == 0;
    uint32_t px = on ? strips[s]->Color(R(c),G(c),B(c)) : 0;
    strips[s]->setPixelColor(mapIndex(s,i), px);
  }
  strips[s]->show();
  st.step++;
}

// 3) Rainbow (non-blocking)
uint32_t wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return packColor(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return packColor(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return packColor(WheelPos * 3, 255 - WheelPos * 3, 0);
}

void animRainbow(uint8_t s, AnimState& st, uint32_t now){
  if (now - st.lastUpdate < 20) return;
  st.lastUpdate = now;
  if (!strips[s]) return;
  uint16_t n = strips[s]->numPixels();
  for (uint16_t i=0;i<n;i++){
    uint32_t c = wheel((i + st.step) & 255);
    c = applyBrightness(c, settings.strips[s].brightness);
    strips[s]->setPixelColor(mapIndex(s,i), strips[s]->Color(R(c),G(c),B(c)));
  }
  strips[s]->show();
  st.step++;
}

// 4) Scanner (Cylon)
void animScanner(uint8_t s, AnimState& st, uint32_t now){
  if (now - st.lastUpdate < 20) return;
  st.lastUpdate = now;
  if (!strips[s]) return;
  uint16_t n = strips[s]->numPixels();
  uint16_t pos = st.step % (2*(n-1));
  if (pos >= n) pos = 2*(n-1) - pos; // bounce back
  uint32_t c = applyBrightness(runtimeColor[s], settings.strips[s].brightness);
  // fade all
  for (uint16_t i=0;i<n;i++){
    uint32_t existing = strips[s]->getPixelColor(mapIndex(s,i));
    uint8_t r = (uint8_t)(R(existing) * 0.6);
    uint8_t g = (uint8_t)(G(existing) * 0.6);
    uint8_t b = (uint8_t)(B(existing) * 0.6);
    strips[s]->setPixelColor(mapIndex(s,i), strips[s]->Color(r,g,b));
  }
  strips[s]->setPixelColor(mapIndex(s,pos), strips[s]->Color(R(c),G(c),B(c)));
  strips[s]->show();
  st.step++;
}

// Animation registry — add your own by appending here
struct AnimEntry { const char* name; AnimFn fn; };
AnimEntry ANIMS[] = {
  {"Solid",          animSolid},
  {"Color Wipe",     animColorWipe},
  {"Theater Chase",  animTheaterChase},
  {"Rainbow",        animRainbow},
  {"Scanner",        animScanner}
};
const uint8_t NUM_ANIMS = sizeof(ANIMS)/sizeof(ANIMS[0]);

// ===================== SETTINGS LOAD/SAVE =====================
const char* CFG_PATH = "/config.json";

void loadDefaults(){
  for (int i=0;i<5;i++){
    settings.strips[i].pin = DEFAULT_PINS[i];
    settings.strips[i].length = 30;
    settings.strips[i].reverse = false;
    settings.strips[i].brightness = 50;
  }
  settings.startupAnimationId = 0;
  settings.startupDurationSec = 5;
  settings.startupFadeMs = 1000;
}

void saveConfigToFS(){
  DynamicJsonDocument doc(4096);
  for (int i=0;i<5;i++){
    JsonObject s = doc.createNestedObject(String(i));
    s["name"] = STRIP_NAMES[i];
    s["pin"] = settings.strips[i].pin;
    s["length"] = settings.strips[i].length;
    s["reverse"] = settings.strips[i].reverse;
    s["brightness"] = settings.strips[i].brightness;
    s["lastColor"] = runtimeColor[i];
  }
  doc["startupAnimationId"] = settings.startupAnimationId;
  doc["startupDurationSec"] = settings.startupDurationSec;
  doc["startupFadeMs"] = settings.startupFadeMs;

  File f = SPIFFS.open(CFG_PATH, FILE_WRITE);
  if (f){ serializeJsonPretty(doc, f); f.close(); }
}

void loadConfigFromFS(){
  if (!SPIFFS.exists(CFG_PATH)) { saveConfigToFS(); return; }
  File f = SPIFFS.open(CFG_PATH, FILE_READ);
  if (!f) return;
  DynamicJsonDocument doc(6144);
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) return;
  for (int i=0;i<5;i++){
    JsonObject s = doc[String(i)];
    if (s.isNull()) continue;
    settings.strips[i].pin = s["pin"] | settings.strips[i].pin;
    settings.strips[i].length = s["length"] | settings.strips[i].length;
    settings.strips[i].reverse = s["reverse"] | settings.strips[i].reverse;
    settings.strips[i].brightness = s["brightness"] | settings.strips[i].brightness;
    runtimeColor[i] = s["lastColor"] | runtimeColor[i];
  }
  settings.startupAnimationId = doc["startupAnimationId"] | settings.startupAnimationId;
  settings.startupDurationSec = doc["startupDurationSec"] | settings.startupDurationSec;
  settings.startupFadeMs = doc["startupFadeMs"] | settings.startupFadeMs;
}

// ===================== WEBSERVER & UI =====================
const char* INDEX_HTML = R"HTML(
<!doctype html>
<html>
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>Honeer Automotive - CAN BUS</title>
  <style>
    :root { --bg:#0f1115; --panel:#171923; --text:#e6e6e6; --muted:#9aa4b2; --accent:#4f8cff; --ok:#21c55d; --warn:#f59e0b; --bad:#ef4444; }
    *{box-sizing:border-box} body{margin:0;background:var(--bg);color:var(--text);font:14px/1.5 system-ui,Segoe UI,Roboto,Helvetica,Arial}
    header{display:flex;align-items:center;gap:12px;padding:14px 16px;border-bottom:1px solid #222}
    header img{height:36px}
    header .fallback{font-weight:700;opacity:.85}
    .wrap{max-width:1100px;margin:0 auto;padding:16px}
    .grid{display:grid;grid-template-columns:repeat(auto-fit, minmax(260px,1fr));gap:16px}
    .card{background:var(--panel);border:1px solid #242636;border-radius:14px;padding:14px}
    .card h2{margin:0 0 10px;font-size:16px}
    .row{display:flex;align-items:center;gap:8px;flex-wrap:wrap;margin:8px 0}
    label{opacity:.9}
    select, input[type="number"], input[type="text"]{background:#10121a;border:1px solid #2a2d3a;border-radius:8px;color:var(--text);padding:6px 8px}
    .btn{background:var(--accent);border:none;color:white;padding:8px 12px;border-radius:10px;cursor:pointer}
    .btn.flat{background:#2a2d3a}
    .chips{display:flex;gap:8px;flex-wrap:wrap}
    .chip{padding:6px 10px;border-radius:999px;background:#222633;border:1px solid #2a2f3f;cursor:pointer}
    .chip input{margin-right:6px}
    .dot{width:10px;height:10px;border-radius:50%;display:inline-block;margin-right:6px}
    .dot.ok{background:var(--ok)} .dot.warn{background:var(--warn)} .dot.bad{background:var(--bad)} .dot.off{background:#3a3f55}
    .muted{color:var(--muted)}
    .cols{display:grid;grid-template-columns:1fr 1fr;gap:10px}
    .divider{height:1px;background:#242636;margin:10px 0}
    .strip-list{display:grid;gap:8px}
    .strip-tile{background:#131623;border:1px solid #22273a;border-radius:10px;padding:10px}
    .strip-tile h3{margin:0 0 6px;font-size:14px}
    .right{margin-left:auto}
  </style>
</head>
<body>
  <header>
    <img id="logo" alt="logo" onerror="this.replaceWith(Object.assign(document.createElement('div'),{className:'fallback',innerText:'ESP32 LED Controller'}))" />
    <div class="muted">Hotspot portal</div>
    <div class="right"></div>
  </header>
  <div class="wrap">
    <div class="grid">
      <section class="card">
        <h2>Color Panel</h2>
        <div class="row">
          <input type="color" id="colorPicker" value="#ffffff" />
          <select id="presetSelect">
            <option value="">Presets…</option>
            <option value="#ff0000">Red</option>
            <option value="#00ff00">Green</option>
            <option value="#0000ff">Blue</option>
            <option value="#ffffff">White</option>
            <option value="#ff7f00">Orange</option>
            <option value="#ff00ff">Magenta</option>
            <option value="#00ffff">Cyan</option>
            <option value="#ffd700">Warm White</option>
          </select>
          <button class="btn" id="applyColor">Apply to Selected</button>
        </div>
        <div class="strip-list" id="colorTargets"></div>
      </section>

      <section class="card">
        <h2>Animations</h2>
        <div class="row">
          <select id="animSelect"></select>
          <button class="btn" id="applyAnim">Set for Selected</button>
        </div>
        <div class="strip-list" id="animTargets"></div>
      </section>

      <section class="card">
        <h2>Startup</h2>
        <div class="row">
          <select id="startupSelect"></select>
          <input type="number" id="startupDuration" min="0" value="5" title="Duration (seconds)" />
          <input type="number" id="startupFade" min="0" value="1000" title="Fade (ms)" />
          <button class="btn" id="saveStartup">Save Startup</button>
        </div>
        <div class="muted">Choose which animation runs when the ESP32 powers up, how long it runs, and the fade time into your ambient color.</div>
      </section>

      <section class="card">
        <h2>Settings</h2>
        <div class="strip-list" id="settingsList"></div>
        <div class="divider"></div>
        <div class="row">
          <button class="btn" id="saveSettings">Save Settings</button>
          <button class="btn flat" id="revertSettings">Revert (reload)</button>
        </div>
      </section>

      <section class="card">
        <h2>Status & Debug</h2>
        <div class="row"><span class="dot off" id="dbg-anim"></span>Animation Loop</div>
        <div class="row"><span class="dot off" id="dbg-ws"></span>WebSocket</div>
        <div class="row"><span class="dot off" id="dbg-save"></span>Last Save</div>
      </section>

      <section class="card">
        <h2>Testing</h2>
        <div class="row chips">
          <label class="chip"><input type="radio" name="test" value="blink"> Blink All</label>
          <label class="chip"><input type="radio" name="test" value="fill"> Fill Random</label>
          <label class="chip"><input type="radio" name="test" value="rainbow"> Rainbow All</label>
          <label class="chip"><input type="radio" name="test" value="clear"> Clear</label>
        </div>
        <div class="row">
          <button class="btn" id="runTest">Run Test</button>
        </div>
      </section>
    </div>
  </div>

<script>
  const byId = (id)=>document.getElementById(id);
  const ws = new WebSocket(`ws://${location.host}/ws`);
  let meta = null; // populated on connect

  function setDot(id, cls){
    const el = byId(id);
    el.className = 'dot ' + cls;
  }

  ws.addEventListener('open', ()=> setDot('dbg-ws','ok'));
  ws.addEventListener('close', ()=> setDot('dbg-ws','bad'));
  ws.addEventListener('message', (ev)=>{
    const msg = JSON.parse(ev.data);
    if (msg.type === 'hello'){
      meta = msg.data;
      initUI(meta);
    } else if (msg.type === 'saved'){
      setDot('dbg-save','ok');
      setTimeout(()=>setDot('dbg-save','off'), 1500);
    }
  });

  function initUI(meta){
    // logo
    byId('logo').src = '/logo.png';

    // color targets
    const colorTargets = byId('colorTargets');
    colorTargets.innerHTML = meta.strips.map((n, i)=>
      `<div class="strip-tile"><h3>${n}</h3>
        <label><input type="checkbox" class="colorTarget" value="${i}"> Selected</label>
      </div>`).join('');

    // animation targets
    const animTargets = byId('animTargets');
    animTargets.innerHTML = meta.strips.map((n, i)=>
      `<div class="strip-tile"><h3>${n}</h3>
        <label><input type="checkbox" class="animTarget" value="${i}"> Selected</label>
      </div>`).join('');

    // animation options
    const animSel = byId('animSelect');
    animSel.innerHTML = meta.animations.map((a, i)=>`<option value="${i}">${a}</option>`).join('');

    // startup options
    const startupSel = byId('startupSelect');
    startupSel.innerHTML = meta.animations.map((a, i)=>`<option value="${i}">${a}</option>`).join('');
    startupSel.value = meta.startupAnimationId;
    byId('startupDuration').value = (meta.startupDurationSec ?? 5);
    byId('startupFade').value = (meta.startupFadeMs ?? 1000);

    // settings list
    renderSettings(meta.settings);
  }

  function renderSettings(s){
    const holder = byId('settingsList');
    holder.innerHTML = '';
    s.forEach((st, i)=>{
      const row = document.createElement('div');
      row.className = 'strip-tile';
      row.innerHTML = `
        <h3>${st.name}</h3>
        <div class="cols">
          <label>Pin<br><input type="number" value="${st.pin}" data-i="${i}" data-k="pin"></label>
          <label>Length<br><input type="number" value="${st.length}" data-i="${i}" data-k="length"></label>
          <label>Direction<br>
            <select data-i="${i}" data-k="reverse">
              <option value="false" ${!st.reverse?'selected':''}>Normal</option>
              <option value="true" ${st.reverse?'selected':''}>Reversed</option>
            </select>
          </label>
          <label>Brightness (%)<br><input type="number" min="0" max="100" value="${st.brightness}" data-i="${i}" data-k="brightness"></label>
        </div>`;
      holder.appendChild(row);
    });
  }

  // Color apply
  byId('applyColor').addEventListener('click', ()=>{
    const hex = byId('colorPicker').value || '#ffffff';
    const targets = [...document.querySelectorAll('.colorTarget:checked')].map(x=>+x.value);
    send({type:'setColor', color:hex, targets});
  });
  byId('presetSelect').addEventListener('change', (e)=>{
    if (e.target.value){ byId('colorPicker').value = e.target.value; e.target.value=''; }
  });

  // Animation apply
  byId('applyAnim').addEventListener('click', ()=>{
    const animId = +byId('animSelect').value;
    const targets = [...document.querySelectorAll('.animTarget:checked')].map(x=>+x.value);
    send({type:'setAnimation', animId, targets});
  });

  // Startup save
  byId('saveStartup').addEventListener('click', ()=>{
    const animId = +byId('startupSelect').value;
    const durationSec = Math.max(0, +byId('startupDuration').value || 0);
    const fadeMs = Math.max(0, +byId('startupFade').value || 0);
    send({type:'setStartup', animId, durationSec, fadeMs});
  });

  // Settings save/revert
  byId('saveSettings').addEventListener('click', ()=>{
    const rows = [...byId('settingsList').querySelectorAll('[data-i]')];
    const payload = [];
    for (let i=0;i<meta.settings.length;i++) payload[i] = {};
    rows.forEach(el=>{
      const i = +el.dataset.i, k = el.dataset.k; let v = el.value;
      if (k === 'reverse') v = (v === 'true');
      if (k === 'pin' || k === 'length' || k === 'brightness') v = +v;
      payload[i][k] = v;
    });
    send({type:'saveSettings', settings: payload});
  });
  byId('revertSettings').addEventListener('click', ()=> location.reload());

  // Tests
  byId('runTest').addEventListener('click', ()=>{
    const r = document.querySelector('input[name="test"]:checked');
    if (!r) return;
    send({type:'runTest', test:r.value});
  });

  function send(o){ ws.send(JSON.stringify(o)); }
</script>
</body>
</html>
)HTML";

// Serve /logo.png (from SPIFFS). If not present, client will fallback.
void setupWeb(){
  // Index
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* req){
    AsyncWebServerResponse *res = req->beginResponse_P(200, "text/html", INDEX_HTML);
    res->addHeader("Cache-Control", "no-cache");
    req->send(res);
  });

  // Logo passthrough
  server.on("/logo.png", HTTP_GET, [](AsyncWebServerRequest* req){
    if (!SPIFFS.exists("/logo.png")){
      req->send(404, "text/plain", "No logo uploaded");
      return;
    }
    // Use FS-based send() overload to avoid ambiguous File overload
    req->send(SPIFFS, "/logo.png", "image/png");
  });

  // WebSocket for realtime control
  ws.onEvent([](AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t * data, size_t len){
    if (type == WS_EVT_CONNECT){
      // Send hello/meta
      DynamicJsonDocument doc(1024);
      doc["type"] = "hello";
      JsonObject d = doc.createNestedObject("data");
      JsonArray names = d.createNestedArray("strips");
      for (int i=0;i<5;i++) names.add(STRIP_NAMES[i]);
      JsonArray anims = d.createNestedArray("animations");
      for (int i=0;i<NUM_ANIMS;i++) anims.add(ANIMS[i].name);
      d["startupAnimationId"] = settings.startupAnimationId;
      d["startupDurationSec"] = settings.startupDurationSec;
      d["startupFadeMs"] = settings.startupFadeMs;
      JsonArray st = d.createNestedArray("settings");
      for (int i=0;i<5;i++){
        JsonObject s = st.createNestedObject();
        s["name"] = STRIP_NAMES[i];
        s["pin"] = settings.strips[i].pin;
        s["length"] = settings.strips[i].length;
        s["reverse"] = settings.strips[i].reverse;
        s["brightness"] = settings.strips[i].brightness;
      }
      String out; serializeJson(doc, out);
      client->text(out);
    }
    else if (type == WS_EVT_DATA){
      AwsFrameInfo * info = (AwsFrameInfo*)arg;
      if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
        DynamicJsonDocument msg(1024);
        DeserializationError err = deserializeJson(msg, data, len);
        if (err) return;
        String t = msg["type"] | "";

        if (t == "setColor"){
          const char* hex = msg["color"] | "#ffffff";
          JsonArray targets = msg["targets"].as<JsonArray>();
          // Parse hex #rrggbb
          long rgb = strtol(hex+1, nullptr, 16);
          uint32_t col = ((rgb >> 16) & 0xFF) << 16 | ((rgb >> 8) & 0xFF) << 8 | (rgb & 0xFF);
          for (uint8_t s : targets){
            if (s < 5){ runtimeColor[s] = col; activeAnimation[s] = 0; animState[s] = {}; fillStripSolid(s, runtimeColor[s]); }
          }
          // Persist colors as the ambient state
          saveConfigToFS();
        }
        else if (t == "setAnimation"){
          uint8_t animId = msg["animId"] | 0;
          JsonArray targets = msg["targets"].as<JsonArray>();
          for (uint8_t s : targets){ if (s < 5){ activeAnimation[s] = animId; animState[s] = {}; } }
        }
        else if (t == "setStartup"){
          uint8_t animId = msg["animId"] | 0;
          settings.startupAnimationId = min<uint8_t>(animId, NUM_ANIMS-1);
          settings.startupDurationSec = msg["durationSec"] | settings.startupDurationSec;
          settings.startupFadeMs = msg["fadeMs"] | settings.startupFadeMs;
          saveConfigToFS();
          DynamicJsonDocument ok(64); ok["type"] = "saved"; String out; serializeJson(ok, out); ws.textAll(out);
        }
        else if (t == "saveSettings"){
          JsonArray arr = msg["settings"].as<JsonArray>();
          if (arr.size() == 5){
            for (int i=0;i<5;i++){
              JsonObject s = arr[i];
              if (s.containsKey("pin")) settings.strips[i].pin = s["pin"].as<int>();
              if (s.containsKey("length")) settings.strips[i].length = s["length"].as<uint16_t>();
              if (s.containsKey("reverse")) settings.strips[i].reverse = s["reverse"].as<bool>();
              if (s.containsKey("brightness")) settings.strips[i].brightness = (uint8_t) constrain(s["brightness"].as<int>(), 0, 100);
            }
            saveConfigToFS();
            buildStrips();
            DynamicJsonDocument ok(64); ok["type"] = "saved"; String out; serializeJson(ok, out); ws.textAll(out);
          }
        }
        else if (t == "runTest"){
          String test = msg["test"] | "";
          if (test == "blink"){
            // blink all strips quickly
            for (int k=0;k<3;k++){
              for (int i=0;i<5;i++) fillStripSolid(i, packColor(255,255,255));
              delay(120);
              for (int i=0;i<5;i++) fillStripSolid(i, packColor(0,0,0));
              delay(120);
            }
          } else if (test == "fill"){
            for (int i=0;i<5;i++){
              runtimeColor[i] = packColor(random(256), random(256), random(256));
              activeAnimation[i] = 0; animState[i] = {}; fillStripSolid(i, runtimeColor[i]);
            }
          } else if (test == "rainbow"){
            for (int i=0;i<5;i++){ activeAnimation[i] = 3; animState[i] = {}; }
          } else if (test == "clear"){
            for (int i=0;i<5;i++){ clearStrip(i); }
          }
        }
      }
    }
  });
  server.addHandler(&ws);

  // Static file fallbacks if you want to add more assets later
  server.serveStatic("/", SPIFFS, "/");

  server.begin();
}

// ===================== SETUP/LOOP =====================
void setup(){
  Serial.begin(115200);
  delay(100);

  // Filesystem
  SPIFFS.begin(true); // format on fail

  // Settings defaults -> load saved overrides
  loadDefaults();
  loadConfigFromFS();

  // Build strips from settings
  buildStrips();

  // WiFi AP
  WiFi.mode(WIFI_AP);
  bool ap = WiFi.softAP(AP_SSID, AP_PASSWORD);
  Serial.printf("AP %s on %s\n", ap?"started":"failed", WiFi.softAPIP().toString().c_str());

  // Web
  setupWeb();

  // Startup animation preset
  for (int i=0;i<5;i++){
    activeAnimation[i] = min<uint8_t>(settings.startupAnimationId, NUM_ANIMS-1);
    animState[i] = {};
  }
  // Initialize startup timer and fade state
  startupActive = (settings.startupDurationSec > 0);
  startupDeadline = millis() + (uint32_t)settings.startupDurationSec * 1000UL;
  globalDimmer = 100; fadePhase = 0; fadeStart = 0;
}

void loop(){
  uint32_t now = millis();
  // Handle startup timeout and smooth fade to ambient solid
  if (startupActive){
    if (settings.startupFadeMs == 0){
      if (now >= startupDeadline){
        for (int i=0;i<5;i++){ activeAnimation[i] = 0; animState[i] = {}; }
        startupActive = false; globalDimmer = 100; fadePhase = 0;
      }
    } else {
      if (now >= startupDeadline && fadePhase == 0){
        // begin fade down
        fadePhase = 1; fadeStart = now;
      }
      if (fadePhase == 1){
        uint32_t half = settings.startupFadeMs / 2;
        uint32_t dt = now - fadeStart;
        if (dt >= half){
          globalDimmer = 0;
          // switch to solid ambient
          for (int i=0;i<5;i++){ activeAnimation[i] = 0; animState[i] = {}; }
          // begin fade up
          fadePhase = 2; fadeStart = now;
        } else {
          float p = (half==0)?1.0f: (float)dt / (float)half; // 0..1
          globalDimmer = (uint8_t)(100.0f * (1.0f - p));
        }
      } else if (fadePhase == 2){
        uint32_t half = settings.startupFadeMs - (settings.startupFadeMs / 2);
        uint32_t dt = now - fadeStart;
        if (dt >= half){
          globalDimmer = 100; startupActive = false; fadePhase = 0;
        } else {
          float p = (half==0)?1.0f: (float)dt / (float)half; // 0..1
          globalDimmer = (uint8_t)(100.0f * p);
        }
      }
    }
  }

  // Drive animations (non-blocking)
  for (int i=0;i<5;i++){
    uint8_t id = min<uint8_t>(activeAnimation[i], NUM_ANIMS-1);
    ANIMS[id].fn(i, animState[i], now);
  }
  // Housekeeping
  ws.cleanupClients();
}

// ===================== HOW TO EXTEND =====================
// 1) Add a new animation:
//    - Write a function: void animMyEffect(uint8_t s, AnimState& st, uint32_t now) { ... }
//    - Append {"My Effect", animMyEffect} to ANIMS[] above.
//    - It will automatically appear in the Animations & Startup dropdowns.
//
// 2) Add more testing actions:
//    - Handle additional values in the "runTest" branch inside the WebSocket handler.
//
// 3) Add debug signals/dots:
//    - Send a ws message with type "dbg" and update the UI to toggle dot classes.
//
// 4) Add more settings:
//    - Extend ControllerSettings / StripSettings, expose them in the UI (renderSettings) and handle in saveSettings.
//
// 5) Replace the logo:
//    - Upload your PNG to SPIFFS as /logo.png. The page automatically loads it.
