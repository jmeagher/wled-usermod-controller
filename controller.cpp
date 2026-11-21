// WLED usermod: touch controller for this WLED and every other WLED on the LAN.
//
// Hardware: Waveshare ESP32-C6-Touch-LCD-1.47 (JD9853 172x320 LCD, AXS5106L capacitive touch).
//
// Main screen (portrait):
//   Device row      tap -> full-screen device list
//   Power row       tap -> toggle
//   Preset row      tap -> full-screen preset list
//   Brightness      touch or drag anywhere in the area to set the level
// Lists: drag up/down to scroll, tap an item to choose it, tap X (top right) to go back.
//
// Threading: everything that touches WLED state (the local device, the Nodes list) runs in
// loop() on the main task. HTTP to other devices runs in one worker task that never touches WLED
// globals; the two share `devs` and `reqs`, both guarded by `mtx`.
#include "wled.h"

#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <WiFiClient.h>
#include <algorithm>
#include <vector>

#include "ctl_logic.h"      // pure logic (slider, touch decode, scrolling), unit-tested on the host

#ifndef ARDUINO_ARCH_ESP32
  #error "usermod controller needs an ESP32"
#endif

// Pin defaults are the Waveshare ESP32-C6-Touch-LCD-1.47; override with -D CTL_PIN_*=n. Every pin is also
// editable in the WLED UI (restart to apply). A touch pin of -1 disables touch.
#ifndef CTL_PIN_MOSI
  #define CTL_PIN_MOSI 2
  #define CTL_PIN_SCLK 1
  #define CTL_PIN_CS   14
  #define CTL_PIN_DC   15
  #define CTL_PIN_RST  22
  #define CTL_PIN_BL   23
  #define CTL_PIN_TSDA 18
  #define CTL_PIN_TSCL 19
  #define CTL_PIN_TRST 20
  #define CTL_PIN_TINT 21
#endif
// The panel is a JD9853 (ST7789-compatible, needs vendor registers). -D CTL_LCD_PLAIN_ST7789 skips them.
#ifndef CTL_LCD_PLAIN_ST7789
  #define CTL_LCD_JD9853 1
  #include "jd9853_init.h"
#endif

#ifdef CTL_DEBUG
  #define CTL_LOG(...) Serial.printf(__VA_ARGS__)
#else
  #define CTL_LOG(...) do {} while (0)
#endif

namespace {

constexpr int kW = 172, kH = 320;                 // portrait
constexpr int kHeaderH = 30, kRowH = 62, kBriH = kH - kHeaderH - 3 * kRowH;   // 104
constexpr int kDevY = kHeaderH, kPowY = kDevY + kRowH, kPsY = kPowY + kRowH, kBriY = kPsY + kRowH;
constexpr int kPickTitleH = 36, kItemH = 44, kVisible = (kH - kPickTitleH) / kItemH;   // 6
constexpr int kTrackX = 10, kTrackW = kW - 20, kTrackY = 34, kTrackH = 50;
constexpr uint32_t kPollMs = 2500, kMdnsMs = 30000, kPresetRetryMs = 10000;
constexpr uint32_t kSweepIdleMs = 60000;    // pause between unicast sweeps of the local /24
constexpr uint16_t kSweepProbeMs = 350;   // connect timeout per address; a device at -88 dBm needs more than 200 ms
constexpr uint8_t kMdnsMaxMissed = 3;   // a device that answers mDNS only some of the time (weak WiFi) stays listed
constexpr uint32_t kBriDebounceMs = 150, kPickIdleMs = 20000;
constexpr uint8_t kTouchAddr = 0x63, kTouchIdReg = 0x08, kTouchDataReg = 0x01;

enum Screen : uint8_t { SCR_MAIN, SCR_PICK_DEVICE, SCR_PICK_PRESET };
enum Region : int8_t { R_NONE = -1, R_DEVICE, R_POWER, R_PRESET, R_BRI };
enum TouchEv : uint8_t { T_DOWN, T_MOVE, T_UP };

struct Preset { uint8_t id; char name[24]; bool scene; };

struct Dev {
  uint32_t ip = 0;               // 0 = this device
  bool local = false;
  char name[33] = {};
  bool on = false;
  uint8_t bri = 128;             // brightness to show (the remembered one while off)
  uint8_t ps = 0;                // active preset id, 0 = none
  bool stateKnown = false;
  bool reachable = true;
  uint32_t lastCmdAt = 0;        // ignore polled state right after a command (it may be stale)
  std::vector<Preset> presets;
  bool presetsLoaded = false;
  bool presetsFetching = false;
  uint32_t presetsAt = 0;        // last attempt
};

// One pending request per remote device; later requests for the same device merge in.
struct Req {
  uint32_t ip = 0;
  uint8_t power = 0;             // 0 none, 1 on, 2 off, 3 toggle
  int16_t bri = -1;
  int16_t ps = -1;
  bool poll = false;
  bool presets = false;
  uint32_t briDueAt = 0;         // debounce: a lone brightness request waits until this time
};

struct MdnsHit { uint32_t ip; char name[33]; uint8_t missed; };   // missed: consecutive scans that did not see it

struct PickItem { char label[34]; char sub[20]; uint32_t ip; uint8_t id; bool current; bool header; };

// RGB565 helper
constexpr uint16_t rgb(uint8_t r, uint8_t g, uint8_t b) {
  return uint16_t(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}
constexpr uint16_t C_BG = rgb(0, 0, 0), C_TEXT = rgb(235, 235, 235), C_DIM = rgb(130, 135, 145),
                   C_PRESS = rgb(28, 52, 110), C_CUR = rgb(18, 38, 86), C_ACCENT = rgb(255, 150, 0),
                   C_ON = rgb(40, 190, 90), C_OFF = rgb(120, 40, 40), C_BAR = rgb(60, 140, 255),
                   C_TRACK = rgb(50, 55, 65), C_ERR = rgb(255, 90, 80);

// setColRowStart() is protected in the Adafruit driver; expose it for the panel's RAM offset.
class Panel : public Adafruit_ST7789 {
 public:
  using Adafruit_ST7789::Adafruit_ST7789;
  void setOffsets(int8_t col, int8_t row) { setColRowStart(col, row); }
};

String ipStr(uint32_t ip) { return IPAddress(ip).toString(); }

volatile bool gTouchInt = false;
void IRAM_ATTR onTouchInt() { gTouchInt = true; }

}  // namespace

class ControllerUsermod : public Usermod {
 public:
  // --- configuration (persisted) ------------------------------------------------------------
  bool enabled = true;
  int8_t pinMosi = CTL_PIN_MOSI, pinSclk = CTL_PIN_SCLK, pinCs = CTL_PIN_CS, pinDc = CTL_PIN_DC,
         pinRst = CTL_PIN_RST, pinBl = CTL_PIN_BL;
  int8_t pinTsda = CTL_PIN_TSDA, pinTscl = CTL_PIN_TSCL, pinTrst = CTL_PIN_TRST, pinTint = CTL_PIN_TINT;
  uint16_t screenTimeoutS = 60;      // 0 = never

  void setup() override {
    if (!enabled) return;
    mtx = xSemaphoreCreateMutex();
    devs.emplace_back();
    devs[0].local = true;

    managed_pin_type pins[10];
    size_t np = 0;
    for (int8_t p : {pinMosi, pinSclk, pinCs, pinDc, pinRst, pinBl, pinTsda, pinTscl, pinTrst})
      if (p >= 0) pins[np++] = {p, true};
    if (pinTint >= 0) pins[np++] = {pinTint, false};
    if (!PinManager::allocateMultiplePins(pins, np, PinOwner::UM_Unspecified)) {
      DEBUG_PRINTLN(F("[ctl] display/touch pins are in use; usermod disabled"));
      enabled = false;
      return;
    }

    pinMode(pinBl, OUTPUT);
    digitalWrite(pinBl, LOW);
    SPI.begin(pinSclk, -1, pinMosi, pinCs);
    tft = new Panel(&SPI, pinCs, pinDc, pinRst);
    tft->init(kW, kH);
#ifdef CTL_LCD_JD9853
    runJd9853Init();               // JD9853 needs vendor registers on top of the ST7789 init
#endif
    tft->setOffsets(34, 0);        // this 172-wide panel sits at column 34 of the 240-wide controller RAM
    tft->setRotation(0);
#ifdef CTL_LCD_JD9853
    { const uint8_t madctl = 0x00; tft->sendCommand(0x36, &madctl, 1); }   // Waveshare's demo orientation
#endif
    tft->fillScreen(C_BG);
    canvas = new GFXcanvas16(kW, kBriH);       // tallest region; every other region is shorter
    if (!canvas || !canvas->getBuffer()) {
      DEBUG_PRINTLN(F("[ctl] out of memory for the frame buffer; usermod disabled"));
      enabled = false;
      return;
    }
    digitalWrite(pinBl, HIGH);
    lastActivity = millis();
    touchOk = touchInit();
    reloadLocalPresets();

    xTaskCreate(&ControllerUsermod::workerEntry, "ctl-http", 10240, this, 1, nullptr);
    ready = true;
  }

  void loop() override {
    if (!ready) return;
    uint32_t now = millis();

    pollTouch(now);

    // Screen timeout; a list is also closed when left idle.
    if (awake && screenTimeoutS && ctl::elapsedMs(now, lastActivity, screenTimeoutS * 1000UL)) sleepScreen();
    if (screen != SCR_MAIN && ctl::elapsedMs(now, lastActivity, kPickIdleMs)) closePicker();

    if (now - lastHousekeeping >= 500) {   // device list, local state, polling
      lastHousekeeping = now;
      refreshLocal();
      refreshDevices(now);
      schedulePolls(now);
    }
    flushBrightness(now);
    if (awake && now - lastDraw >= 40) { lastDraw = now; redraw(); }
  }

  void connected() override { lastMdns = 0; }

  uint16_t getId() override { return USERMOD_ID_UNSPECIFIED; }

  void addToConfig(JsonObject& root) override {
    JsonObject top = root.createNestedObject(F("Controller"));
    top[F("enabled")] = enabled;
    top[F("screen timeout s")] = screenTimeoutS;
    top[F("pin mosi")] = pinMosi;
    top[F("pin sclk")] = pinSclk;
    top[F("pin cs")] = pinCs;
    top[F("pin dc")] = pinDc;
    top[F("pin rst")] = pinRst;
    top[F("pin backlight")] = pinBl;
    top[F("pin touch sda")] = pinTsda;
    top[F("pin touch scl")] = pinTscl;
    top[F("pin touch rst")] = pinTrst;
    top[F("pin touch int")] = pinTint;
  }

  bool readFromConfig(JsonObject& root) override {
    JsonObject top = root[F("Controller")];
    bool complete = !top.isNull();
    complete &= getJsonValue(top[F("enabled")], enabled, true);
    complete &= getJsonValue(top[F("screen timeout s")], screenTimeoutS, 60);
    complete &= getJsonValue(top[F("pin mosi")], pinMosi, CTL_PIN_MOSI);
    complete &= getJsonValue(top[F("pin sclk")], pinSclk, CTL_PIN_SCLK);
    complete &= getJsonValue(top[F("pin cs")], pinCs, CTL_PIN_CS);
    complete &= getJsonValue(top[F("pin dc")], pinDc, CTL_PIN_DC);
    complete &= getJsonValue(top[F("pin rst")], pinRst, CTL_PIN_RST);
    complete &= getJsonValue(top[F("pin backlight")], pinBl, CTL_PIN_BL);
    complete &= getJsonValue(top[F("pin touch sda")], pinTsda, CTL_PIN_TSDA);
    complete &= getJsonValue(top[F("pin touch scl")], pinTscl, CTL_PIN_TSCL);
    complete &= getJsonValue(top[F("pin touch rst")], pinTrst, CTL_PIN_TRST);
    complete &= getJsonValue(top[F("pin touch int")], pinTint, CTL_PIN_TINT);
    return complete;
  }

  void appendConfigData() override {
    oappend(F("addInfo('Controller:pin mosi',1,'restart needed after changing pins');"));
  }

 private:
  // --- state shared with the worker (guard with mtx) ------------------------------------------
  SemaphoreHandle_t mtx = nullptr;
  std::vector<Dev> devs;               // [0] is this device, the rest sorted by name
  Req reqs[4];
  std::vector<MdnsHit> mdnsHits;
  // Unicast fallback. Some APs/mesh nodes never deliver multicast or broadcast to their clients, which silences
  // both the WLED node list (UDP broadcast) and mDNS. Unicast HTTP still works there, so when neither source has
  // found anything the worker probes the local /24 for WLED devices. missed counts whole sweeps, not scans.
  std::vector<MdnsHit> sweepHits;
  volatile uint8_t nodeCount = 0;   // Nodes.size(), published from loop() because Nodes is WLED state
  uint16_t sweepHost = 0;           // next last-octet to probe; 0 = no sweep in progress
  uint32_t sweepDoneAt = 0;         // when the last sweep ended

  struct Lock {
    SemaphoreHandle_t m;
    explicit Lock(SemaphoreHandle_t m) : m(m) { xSemaphoreTake(m, portMAX_DELAY); }
    ~Lock() { xSemaphoreGive(m); }
  };

#ifdef CTL_LCD_JD9853
  // Replays kJd9853Init: [cmd, n, data x n, delay/5ms]... terminated by 0xFF 0xFF.
  void runJd9853Init() {
    for (const uint8_t* p = kJd9853Init; !(p[0] == 0xFF && p[1] == 0xFF);) {
      const uint8_t cmd = p[0], n = p[1];
      tft->sendCommand(cmd, n ? p + 2 : nullptr, n);
      const uint8_t d = p[2 + n];
      if (d) delay(d * 5);
      p += 3 + n;
    }
  }
#endif

  // --- main-task state ----------------------------------------------------------------------
  bool ready = false, touchOk = false;
  Panel* tft = nullptr;
  GFXcanvas16* canvas = nullptr;

  bool awake = true;
  uint32_t lastActivity = 0, lastDraw = 0, lastHousekeeping = 0, lastPoll = 0, lastMdns = 0;
  uint32_t selIp = 0;                  // selected device (0 = this one)
  Screen screen = SCR_MAIN;
  bool localPresetsDirty = false;
  char statusMsg[40] = {};
  uint32_t statusUntil = 0;

  // pending (debounced) remote brightness
  uint32_t briSendAt = 0; uint32_t briSendIp = 0; int16_t briSendVal = -1;

  char wifiSsid[33] = {}, wifiIp[16] = {};   // cached in refreshLocal (WiFi.SSID() allocates)
  uint32_t regionSig[5] = {};          // header + 4 rows; skip redraw when unchanged
  uint32_t pickSig = 0;

  // list state
  std::vector<PickItem> pickItems;
  int pickFirst = 0;                   // index of the first visible item
  uint32_t lastPickBuild = 0;

  // --- touch ----------------------------------------------------------------------------------
  bool touching = false, tIgnore = false, tMoved = false, tSlider = false;
  int tStartX = 0, tStartY = 0, tLastX = 0, tLastY = 0, tAccum = 0;
  Region tRegion = R_NONE;             // row being pressed (highlight + tap target)
  uint32_t lastTouchRead = 0, touchDownAt = 0, lastTouchReset = 0;
  uint8_t touchFails = 0;

  int i2cErr = 0;                      // last failure: Wire.endTransmission() code, or 100 + bytes received

  bool i2cRead(uint8_t reg, uint8_t* out, uint8_t n) {
    Wire.beginTransmission(kTouchAddr);
    Wire.write(reg);
    if ((i2cErr = Wire.endTransmission()) != 0) return false;
    const uint8_t got = Wire.requestFrom(kTouchAddr, n);
    if (got != n) { i2cErr = 100 + got; return false; }
    for (uint8_t i = 0; i < n; i++) out[i] = Wire.read();
    i2cErr = 0;
    return true;
  }

  void touchReset() {                  // reset sequence from Waveshare's driver
    if (pinTrst < 0) return;
    CTL_LOG("[ctl] %lu touch reset begin\n", (unsigned long)millis());
    pinMode(pinTrst, OUTPUT);
    digitalWrite(pinTrst, LOW);  delay(200);
    digitalWrite(pinTrst, HIGH); delay(300);
    CTL_LOG("[ctl] %lu touch reset end\n", (unsigned long)millis());
  }

  bool touchInit() {
    if (pinTsda < 0 || pinTscl < 0) return false;
    Wire.begin(pinTsda, pinTscl, 400000);
    touchReset();
    if (pinTint >= 0) {
      pinMode(pinTint, INPUT_PULLUP);
      attachInterrupt(pinTint, onTouchInt, FALLING);
    }
    uint8_t id[3] = {};
    bool ok = i2cRead(kTouchIdReg, id, 3);
    CTL_LOG("[ctl] touch %s id=%02x %02x %02x\n", ok ? "found" : "NOT FOUND", id[0], id[1], id[2]);
    return ok;
  }

  // Read the controller when it signalled (INT edge), while INT is held low (the chip's INT is a latched level,
  // so an edge can be missed), while a finger is down, and every 150 ms while the screen is dark. Emits DOWN/MOVE/UP.
  // If the chip stops answering it is reset (at most every 10 s).
  void pollTouch(uint32_t now) {
    if (!touchOk) return;
    const bool intLow = pinTint >= 0 && digitalRead(pinTint) == LOW;
    const bool due = gTouchInt || touching || intLow || (!awake && now - lastTouchRead >= 150);
    if (!due || now - lastTouchRead < 12) return;
    lastTouchRead = now;
    const bool flag = gTouchInt;
    gTouchInt = false;
    uint8_t d[14] = {};
    const bool readOk = i2cRead(kTouchDataReg, d, 14);
#ifdef CTL_DEBUG
    {   // one line per change of state (not per read), so the dark-screen poll does not flood the log
      const uint32_t sig = readOk | (intLow << 1) | (flag << 2) | (awake << 3) | (touching << 4) |
                           (uint32_t(readOk ? d[1] : 255) << 8) | (uint32_t(i2cErr) << 16);
      static uint32_t lastSig = 0xFFFFFFFF;
      if (sig != lastSig) {
        lastSig = sig;
        CTL_LOG("[ctl] %lu read ok=%d err=%d int=%d flag=%d awake=%d touching=%d raw=%02x %02x %02x %02x %02x %02x\n",
                (unsigned long)now, readOk, i2cErr, intLow, flag, awake, touching, d[0], d[1], d[2], d[3], d[4], d[5]);
      }
    }
#endif
    if (!readOk) {
      if (++touchFails >= 5 && now - lastTouchReset > 10000) {
        CTL_LOG("[ctl] touch not answering, resetting it\n");
        lastTouchReset = now;
        touchFails = 0;
        touchReset();
      }
      return;
    }
    touchFails = 0;
    const ctl::TouchPoint pt = ctl::decodeTouch(d, kW, kH);
    if (pt.n) {
      const int x = pt.x, y = pt.y;
      if (!touching) { touching = true; touchDownAt = now; onTouch(T_DOWN, x, y, now); }
      else if (x != tLastX || y != tLastY) onTouch(T_MOVE, x, y, now);
    } else if (touching) {
      touching = false;
      onTouch(T_UP, tLastX, tLastY, now);
    }
    if (touching && now - touchDownAt > 15000) {          // safety: never stay "down" forever
      touching = false;
      onTouch(T_UP, tLastX, tLastY, now);
    }
  }

  static Region regionAt(int y) {
    if (y < kDevY) return R_NONE;
    if (y < kPowY) return R_DEVICE;
    if (y < kPsY) return R_POWER;
    if (y < kBriY) return R_PRESET;
    return R_BRI;
  }

  void onTouch(TouchEv ev, int x, int y, uint32_t now) {
    CTL_LOG("[ctl] touch %s x=%d y=%d\n", ev == T_DOWN ? "down" : ev == T_MOVE ? "move" : "up", x, y);
    if (ev == T_DOWN) {
      const bool wasAwake = awake;
      wake(now);
      tIgnore = !wasAwake;             // the touch that wakes the screen does nothing else
      tMoved = false; tSlider = false; tRegion = R_NONE; tAccum = 0;
      tStartX = x; tStartY = y;
      if (!tIgnore) {
        lastActivity = now;
        if (screen == SCR_MAIN) {
          tRegion = regionAt(y);
          if (tRegion == R_BRI) { tSlider = true; setBrightnessFromX(x, now); }
        }
      }
    } else if (!tIgnore) {
      lastActivity = now;
      if (screen == SCR_MAIN) {
        if (tSlider) setBrightnessFromX(x, now);
        else if (abs(y - tStartY) > 14 || abs(x - tStartX) > 14) tRegion = R_NONE;   // dragged off: cancel the tap
      } else {
        if (abs(y - tStartY) > 10) tMoved = true;
        if (tMoved) scrollBy(y - tLastY);
      }
    }
    if (ev == T_UP) {
      if (!tIgnore) {
        if (screen == SCR_MAIN) {
          if (!tSlider && tRegion != R_NONE && tRegion == regionAt(y)) activate(tRegion, now);
        } else if (!tMoved) {
          pickAt(x, y, now);
        }
      }
      tIgnore = false; tSlider = false; tRegion = R_NONE;
    }
    tLastX = x; tLastY = y;
  }

  // --- device list ----------------------------------------------------------------------------
  int selIndexLocked() {               // call with mtx held
    for (size_t i = 0; i < devs.size(); i++) if (devs[i].ip == selIp) return int(i);
    selIp = 0;
    return 0;
  }

  // The local device mirrors WLED's globals; presets are reloaded on demand.
  void refreshLocal() {
    Lock l(mtx);
    Dev& d = devs[0];
    strlcpy(d.name, serverDescription, sizeof(d.name));
    d.on = bri > 0;
    d.bri = bri ? bri : briLast;
    d.ps = currentPreset;
    d.stateKnown = true;
    d.reachable = true;
    if (WLED_CONNECTED) {
      strlcpy(wifiSsid, WiFi.SSID().c_str(), sizeof(wifiSsid));
      strlcpy(wifiIp, WLEDNetwork.localIP().toString().c_str(), sizeof(wifiIp));
    } else {
      wifiSsid[0] = wifiIp[0] = 0;
    }
  }

  void refreshDevices(uint32_t now) {
    if (!WLED_CONNECTED) {
      Lock l(mtx);
      if (devs.size() > 1) devs.resize(1);   // nothing else is reachable
      selIndexLocked();
      return;
    }
    const uint32_t self = (uint32_t)WLEDNetwork.localIP();
    nodeCount = Nodes.size() > 255 ? 255 : Nodes.size();
    struct Cand { uint32_t ip; String name; bool hasOn; bool on; };
    std::vector<Cand> cands;
    for (auto& kv : Nodes) {
      uint32_t ip = (uint32_t)kv.second.ip;
      if (ip && ip != self) cands.push_back({ip, kv.second.nodeName, true, bool(kv.second.on)});
    }
    Lock l(mtx);
    for (auto& h : mdnsHits) {
      bool dup = h.ip == self;
      for (auto& c : cands) dup |= c.ip == h.ip;
      if (!dup) cands.push_back({h.ip, String(h.name), false, false});
    }
    for (auto& h : sweepHits) {
      bool dup = h.ip == self;
      for (auto& c : cands) dup |= c.ip == h.ip;
      if (!dup) cands.push_back({h.ip, String(h.name), false, false});
    }
    std::vector<Dev> next;
    next.reserve(cands.size() + 1);
    next.push_back(std::move(devs[0]));
    for (auto& c : cands) {
      Dev d;
      for (auto& old : devs) if (old.ip == c.ip) { d = std::move(old); break; }
      d.ip = c.ip;
      strlcpy(d.name, c.name.length() ? c.name.c_str() : ipStr(c.ip).c_str(), sizeof(d.name));
      if (!d.stateKnown && c.hasOn) d.on = c.on;   // best guess until the first poll
      next.push_back(std::move(d));
    }
    std::sort(next.begin() + 1, next.end(), [](const Dev& a, const Dev& b) {
      int c = strcasecmp(a.name, b.name);
      return c ? c < 0 : a.ip < b.ip;
    });
    devs.swap(next);
    selIndexLocked();
#ifdef CTL_DEBUG
    {
      static uint32_t lastSig = 0;
      uint32_t sig = devs.size();
      for (auto& d : devs) sig = sig * 31 + d.ip;
      if (sig != lastSig) {
        lastSig = sig;
        CTL_LOG("[ctl] device list (%u):", (unsigned)devs.size());
        for (auto& d : devs) CTL_LOG(" %s=%s", d.name, ipStr(d.ip).c_str());
        CTL_LOG("\n");
      }
    }
#endif
  }

  void schedulePolls(uint32_t now) {
    if (!awake || !WLED_CONNECTED) return;
    Lock l(mtx);
    Dev& d = devs[selIndexLocked()];
    if (d.local) return;
    // Until the first reply the controls are blank, so ask again every 500 ms instead of every kPollMs.
    const uint32_t interval = d.stateKnown ? kPollMs : 500;
    if (now - lastPoll < interval) return;
    lastPoll = now;
    Req* r = slotFor(d.ip);
    if (r) r->poll = true;
  }

  // Fetch the selected remote's presets the first time it is selected, and retry after failures.
  void ensurePresetsLocked(Dev& d, uint32_t now, bool force) {
    if (d.local || !WLED_CONNECTED || d.presetsFetching) return;
    if (d.presetsLoaded && !force) return;
    if (!force && d.presetsAt && now - d.presetsAt < kPresetRetryMs) return;
    Req* r = slotFor(d.ip);
    if (!r) return;
    r->presets = true;
    d.presetsFetching = true;
    d.presetsAt = now;
  }

  Req* slotFor(uint32_t ip) {          // mtx held
    Req* freeSlot = nullptr;
    for (auto& r : reqs) {
      if (r.ip == ip) return &r;
      if (!r.ip && !freeSlot) freeSlot = &r;
    }
    if (freeSlot) { *freeSlot = Req(); freeSlot->ip = ip; }
    return freeSlot;                    // null: worker is saturated, the caller drops the request
  }

  // --- local presets ------------------------------------------------------------------------
  // Parse a preset from a JSON object and populate the Preset struct.
  void parsePreset(JsonObject value, uint8_t id, Preset& p) {
    p.id = id;
    p.scene = ctl::isScene(value["playlist"].is<JsonObject>(), value["ql"] | "");
    const char* n = value["n"] | "";
    if (*n) strlcpy(p.name, n, sizeof(p.name));
    else snprintf(p.name, sizeof(p.name), "Preset %d", id);
  }

  void reloadLocalPresets() {
    if (!requestJSONBufferLock(JSON_LOCK_UNKNOWN)) { localPresetsDirty = true; return; }
    std::vector<Preset> list;
    if (readObjectFromFile(getPresetsFileName(), nullptr, pDoc)) {
      for (JsonPair kv : pDoc->as<JsonObject>()) {
        int id = atoi(kv.key().c_str());
        if (id <= 0 || id > 250) continue;
        Preset p{};
        parsePreset(kv.value().as<JsonObject>(), id, p);
        list.push_back(p);
      }
    }
    releaseJSONBufferLock();
    ctl::orderPresets(list);
    Lock l(mtx);
    devs[0].presets.swap(list);
    devs[0].presetsLoaded = true;
    localPresetsDirty = false;
  }

  // --- actions ------------------------------------------------------------------------------
  void say(const char* msg, uint32_t now, uint32_t ms = 2500) {
    strlcpy(statusMsg, msg, sizeof(statusMsg));
    statusUntil = now + ms;
  }

  void wake(uint32_t now) {
    lastActivity = now;
    if (!awake) { awake = true; digitalWrite(pinBl, HIGH); resetDrawCache(); CTL_LOG("[ctl] %lu screen wake\n", (unsigned long)millis()); }
  }

  void sleepScreen() {
    awake = false;
    screen = SCR_MAIN;
    digitalWrite(pinBl, LOW);
    CTL_LOG("[ctl] %lu screen sleep\n", (unsigned long)millis());
  }

  void resetDrawCache() {
    memset(regionSig, 0, sizeof(regionSig));
    pickSig = 0;
  }

  void activate(Region r, uint32_t now) {
    switch (r) {
      case R_POWER: if (selStateKnown()) togglePower(now); break;
      case R_DEVICE: openPicker(SCR_PICK_DEVICE, now); break;
      case R_PRESET: openPicker(SCR_PICK_PRESET, now); break;
      default: break;
    }
  }

  bool selStateKnown() {
    Lock l(mtx);
    return devs[selIndexLocked()].stateKnown;
  }

  void openPicker(Screen s, uint32_t now) {
    if (s == SCR_PICK_PRESET) {
      if (selIp == 0) reloadLocalPresets();   // picks up presets saved since boot
      Lock l(mtx);
      ensurePresetsLocked(devs[selIndexLocked()], now, true);   // refresh so edits made elsewhere show up
    }
    screen = s;
    pickFirst = 0;
    buildPickItems(true);
    if (s == SCR_PICK_PRESET) {            // start the list scrolled to the active preset
      for (size_t i = 0; i < pickItems.size(); i++)
        if (pickItems[i].current) { pickFirst = std::min<int>(i, maxFirst()); break; }
    }
    resetDrawCache();
    tft->fillScreen(C_BG);
  }

  void closePicker() {
    screen = SCR_MAIN;
    resetDrawCache();
    tft->fillScreen(C_BG);
  }

  int maxFirst() const { return std::max(0, int(pickItems.size()) - kVisible); }

  void scrollBy(int dy) {
    ctl::scrollList(pickFirst, tAccum, dy, kItemH, maxFirst());
  }

  void pickAt(int x, int y, uint32_t now) {
    if (y < kPickTitleH) {
      if (x >= kW - 56) closePicker();                   // the X in the title bar
      return;
    }
    int idx = pickFirst + (y - kPickTitleH) / kItemH;
    if (idx < 0 || idx >= int(pickItems.size())) return;
    const PickItem it = pickItems[idx];
    if (it.header) return;              // ignore taps on header rows
    if (screen == SCR_PICK_DEVICE) {
      selIp = it.ip;
      lastPoll = 0;                    // poll the newly selected device on the next housekeeping pass
      if (it.ip == 0) localPresetsDirty = true;
      else { Lock l(mtx); ensurePresetsLocked(devs[selIndexLocked()], now, false); }
    } else {
      applyPresetTo(it.id, now);
    }
    closePicker();
  }

  // Rebuild the list contents from the shared device state. `force` also resets nothing else.
  void buildPickItems(bool force) {
    uint32_t now = millis();
    if (!force && now - lastPickBuild < 200) return;
    lastPickBuild = now;
    std::vector<PickItem> items;
    Lock l(mtx);
    int si = selIndexLocked();
    if (screen == SCR_PICK_DEVICE) {
      items.reserve(devs.size());
      for (auto& d : devs) {
        PickItem it{};
        strlcpy(it.label, d.name, sizeof(it.label));
        strlcpy(it.sub, d.local ? "this device" : (d.reachable ? ipStr(d.ip).c_str() : "unreachable"), sizeof(it.sub));
        it.ip = d.ip; it.current = d.ip == selIp;
        items.push_back(it);
      }
    } else {
      Dev& d = devs[si];
      items.reserve(d.presets.size() + 2);  // +2 for potential header rows

      // Count scenes to decide whether to show headers
      size_t sceneCount = 0;
      for (auto& p : d.presets) if (p.scene) sceneCount++;

      // If there are scenes, emit headers
      if (sceneCount > 0) {
        // SCENES header
        PickItem hdr{};
        strlcpy(hdr.label, "SCENES", sizeof(hdr.label));
        hdr.header = true;
        items.push_back(hdr);

        // Add scenes
        for (auto& p : d.presets) {
          if (!p.scene) continue;
          PickItem it{};
          snprintf(it.label, sizeof(it.label), "%s", p.name);
          snprintf(it.sub, sizeof(it.sub), "scene %u", p.id);
          it.id = p.id; it.current = p.id == d.ps;
          items.push_back(it);
        }

        // PRESETS header, only when there are component presets to put under it
        if (sceneCount < d.presets.size()) {
          PickItem presetHdr{};
          strlcpy(presetHdr.label, "PRESETS", sizeof(presetHdr.label));
          presetHdr.header = true;
          items.push_back(presetHdr);
        }
      }

      // Add non-scene presets
      for (auto& p : d.presets) {
        if (p.scene) continue;
        PickItem it{};
        snprintf(it.label, sizeof(it.label), "%s", p.name);
        snprintf(it.sub, sizeof(it.sub), "preset %u", p.id);
        it.id = p.id; it.current = p.id == d.ps;
        items.push_back(it);
      }
    }
    pickItems.swap(items);
    pickFirst = constrain(pickFirst, 0, maxFirst());
  }

  void togglePower(uint32_t now) {
    Lock l(mtx);
    Dev& d = devs[selIndexLocked()];
    d.lastCmdAt = now;
    if (d.local) {
      toggleOnOff();
      stateUpdated(CALL_MODE_DIRECT_CHANGE);
    } else if (Req* r = slotFor(d.ip)) {
      r->power = 3;
      d.on = !d.on;
    } else say("busy, try again", now);
  }

  void applyPresetTo(uint8_t id, uint32_t now) {
    {
      Lock l(mtx);
      Dev& d = devs[selIndexLocked()];
      d.lastCmdAt = now;
      if (!d.local) {
        if (Req* r = slotFor(d.ip)) { r->ps = id; d.ps = id; d.on = true; }
        else say("busy, try again", now);
        return;
      }
    }
    ::applyPreset(id, CALL_MODE_DIRECT_CHANGE);   // outside the lock: WLED does real work here
  }

  // Slider: x across the track maps to 1..255 (never 0: Power is the off switch).
  void setBrightnessFromX(int x, uint32_t now) {
    const int v = ctl::briFromTrackX(x, kTrackX, kTrackW);
    Lock l(mtx);
    Dev& d = devs[selIndexLocked()];
    if (!d.stateKnown) return;         // blank controls: do not overwrite a device we have not heard from
    if (d.bri == v && d.on) return;
    d.bri = v;
    d.lastCmdAt = now;
    if (d.local) { bri = v; stateUpdated(CALL_MODE_DIRECT_CHANGE); }
    else {
      briSendIp = d.ip; briSendVal = v; d.on = true;
      if (briSendVal >= 0 && int32_t(now - briSendAt) >= 0) briSendAt = now + kBriDebounceMs;   // rate-limit, not debounce
    }
  }

  // Remote brightness goes out at most every kBriDebounceMs while dragging, always with the latest value.
  void flushBrightness(uint32_t now) {
    if (briSendVal < 0 || int32_t(now - briSendAt) < 0) return;
    Lock l(mtx);
    if (Req* r = slotFor(briSendIp)) {
      r->bri = briSendVal;
      r->briDueAt = 0;
      briSendVal = -1;
    }                                   // else keep it and retry on the next loop
  }

  // --- drawing ------------------------------------------------------------------------------
  struct View {
    char name[33]; uint32_t ip; bool local, on, reachable, stateKnown; uint8_t bri, ps;
    char psName[24]; bool presetsLoaded, presetsFetching; size_t presetCount;
    size_t devCount, devIndex;
  };

  View snapshot() {
    View v{};
    Lock l(mtx);
    int si = selIndexLocked();
    Dev& d = devs[si];
    strlcpy(v.name, d.name, sizeof(v.name));
    v.ip = d.ip; v.local = d.local; v.on = d.on; v.reachable = d.reachable; v.stateKnown = d.stateKnown;
    v.bri = d.bri; v.ps = d.ps;
    v.presetsLoaded = d.presetsLoaded; v.presetsFetching = d.presetsFetching; v.presetCount = d.presets.size();
    v.devCount = devs.size(); v.devIndex = si;
    for (auto& p : d.presets)
      if (p.id == d.ps) snprintf(v.psName, sizeof(v.psName), "%s", p.name);
    if (!v.psName[0] && d.ps) snprintf(v.psName, sizeof(v.psName), "Preset %u", d.ps);
    return v;
  }

  static uint32_t hashBytes(const void* p, size_t n, uint32_t h = 2166136261u) {
    const uint8_t* b = (const uint8_t*)p;
    while (n--) h = (h ^ *b++) * 16777619u;
    return h;
  }

  // Draw `s` in the current font at x, truncating with ".." so it fits maxW pixels.
  static void fitText(GFXcanvas16& c, const char* s, int x, int baseline, int maxW, uint16_t color) {
    char buf[40];
    strlcpy(buf, s, sizeof(buf));
    int16_t x1, y1; uint16_t w, h;
    for (;;) {
      c.getTextBounds(buf, 0, 0, &x1, &y1, &w, &h);
      size_t n = strlen(buf);
      if (int(w) <= maxW || n <= 2) break;
      buf[n - 1] = 0;
      if (n > 3) { buf[n - 2] = '.'; buf[n - 3] = '.'; }
    }
    c.setTextColor(color);
    c.setCursor(x, baseline);
    c.print(buf);
  }

  void redraw() {
    uint32_t now = millis();
    if (statusMsg[0] && int32_t(now - statusUntil) > 0) statusMsg[0] = 0;
    if (localPresetsDirty) reloadLocalPresets();
    if (screen != SCR_MAIN) { redrawPicker(); return; }

    View v = snapshot();
    const bool wifi = WLED_CONNECTED;
    uint32_t base = hashBytes(&v, sizeof(v));
    uint32_t sigs[5];
    uint32_t hdr[3] = {uint32_t(wifi), uint32_t(v.devCount), uint32_t(touchOk)};
    sigs[0] = hashBytes(hdr, sizeof(hdr), hashBytes(statusMsg, sizeof(statusMsg)));
    sigs[0] = hashBytes(wifiSsid, sizeof(wifiSsid), hashBytes(wifiIp, sizeof(wifiIp), sigs[0]));
    for (int r = 0; r < 4; r++) {
      uint32_t flag = tRegion == r ? 1u : 0u;
      sigs[1 + r] = hashBytes(&flag, sizeof(flag), base);
    }
    if (sigs[0] != regionSig[0]) { drawHeader(v, wifi); regionSig[0] = sigs[0]; }
    for (int r = 0; r < 4; r++)
      if (sigs[1 + r] != regionSig[1 + r]) { drawRow(Region(r), v); regionSig[1 + r] = sigs[1 + r]; }
  }

  void drawHeader(const View& v, bool wifi) {
    canvas->fillScreen(C_BG);
    canvas->setFont(nullptr);
    canvas->setTextSize(1);
    char line[48];
    if (statusMsg[0]) { canvas->setTextColor(C_ERR); canvas->setCursor(4, 11); canvas->print(statusMsg); }
    else {
      // Three 8 px lines: network name, address, device count. 28 characters fit across the 172 px panel.
      if (wifi) snprintf(line, sizeof(line), "WiFi: %.22s", wifiSsid[0] ? wifiSsid : "(unknown)");
      else snprintf(line, sizeof(line), apActive ? "AP mode - local only" : "No WiFi - local only");
      canvas->setTextColor(wifi ? C_ON : C_ERR);
      canvas->setCursor(4, 2);
      canvas->print(line);
      canvas->setTextColor(C_DIM);
      canvas->setCursor(4, 12);
      canvas->print(wifi ? wifiIp : "-");
      snprintf(line, sizeof(line), touchOk ? "%u device%s" : "touch not found | %u device%s",
               unsigned(v.devCount), v.devCount == 1 ? "" : "s");
      canvas->setTextColor(touchOk ? C_DIM : C_ERR);
      canvas->setCursor(4, 22);
      canvas->print(line);
    }
    tft->drawRGBBitmap(0, 0, canvas->getBuffer(), kW, kHeaderH);
  }

  void drawRow(Region r, const View& v) {
    const bool pressed = (tRegion == r);
    const int h = (r == R_BRI) ? kBriH : kRowH;
    canvas->fillScreen(pressed ? C_PRESS : C_BG);
    canvas->setFont(&FreeSans9pt7b);
    canvas->setTextSize(1);
    char buf[48];
    const int x = 12, maxW = kW - x - 8;

    switch (r) {
      case R_DEVICE: {
        snprintf(buf, sizeof(buf), "DEVICE  %u of %u  >", unsigned(v.devIndex + 1), unsigned(v.devCount));
        fitText(*canvas, buf, x, 17, maxW, C_DIM);
        canvas->setFont(&FreeSansBold12pt7b);
        fitText(*canvas, v.name, x, 42, maxW, C_TEXT);
        canvas->setFont(nullptr);
        canvas->setTextColor(v.reachable ? C_DIM : C_ERR);
        canvas->setCursor(x, 50);
        canvas->print(v.local ? "this device" : (v.reachable ? ipStr(v.ip).c_str() : "unreachable"));
        break;
      }
      case R_POWER: {
        fitText(*canvas, "POWER", x, 17, maxW, C_DIM);
        if (!v.stateKnown) {            // blank pill until the real state arrives
          canvas->fillRoundRect(x, 24, 96, 34, 17, C_TRACK);
          break;
        }
        canvas->fillRoundRect(x, 24, 96, 34, 17, v.on ? C_ON : C_OFF);
        canvas->setFont(&FreeSansBold12pt7b);
        fitText(*canvas, v.on ? "ON" : "OFF", x + (v.on ? 34 : 27), 48, 80, C_TEXT);
        break;
      }
      case R_PRESET: {
        fitText(*canvas, "PRESET  >", x, 17, maxW, C_DIM);
        const char* shown = v.psName[0] ? v.psName : "-";
        if (!v.stateKnown) shown = v.reachable ? "loading..." : "unreachable";   // active preset is part of the state
        else if (v.presetsFetching && !v.presetsLoaded) shown = "loading...";
        else if (v.presetsLoaded && !v.presetCount) shown = "no presets";
        canvas->setFont(&FreeSansBold12pt7b);
        fitText(*canvas, shown, x, 46, maxW, C_TEXT);
        break;
      }
      case R_BRI: {
        fitText(*canvas, "BRIGHTNESS", x, 22, maxW, C_DIM);
        canvas->fillRoundRect(kTrackX, kTrackY, kTrackW, kTrackH, 12, C_TRACK);
        if (!v.stateKnown) {            // empty track and a message: never draw the placeholder level
          canvas->setFont(&FreeSans9pt7b);
          const char* msg = v.reachable ? "loading..." : "unreachable";
          int16_t mx, my; uint16_t mw, mh;
          canvas->getTextBounds(msg, 0, 0, &mx, &my, &mw, &mh);
          canvas->setTextColor(v.reachable ? C_DIM : C_ERR);
          canvas->setCursor(kTrackX + (kTrackW - int(mw)) / 2 - mx, kTrackY + (kTrackH - int(mh)) / 2 - my);
          canvas->print(msg);
          break;
        }
        // Proportional fill. The corner radius shrinks for narrow fills (a fixed radius needs >= 24 px of width),
        // so 2% looks like 2% instead of a rounded blob a third of the track wide.
        const int fillW = ctl::fillWidth(v.bri, kTrackW);
        canvas->fillRoundRect(kTrackX, kTrackY, fillW, kTrackH, ctl::fillRadius(fillW), v.on ? C_BAR : C_DIM);
        // The level as a percentage, centred on the bar.
        snprintf(buf, sizeof(buf), "%d%%", ctl::percentOf(v.bri));
        canvas->setFont(&FreeSansBold12pt7b);
        int16_t bx, by; uint16_t bw, bh;
        canvas->getTextBounds(buf, 0, 0, &bx, &by, &bw, &bh);
        canvas->setTextColor(C_TEXT);
        canvas->setCursor(kTrackX + (kTrackW - int(bw)) / 2 - bx, kTrackY + (kTrackH - int(bh)) / 2 - by);
        canvas->print(buf);
        break;
      }
      default: break;
    }
    const int y = r == R_DEVICE ? kDevY : r == R_POWER ? kPowY : r == R_PRESET ? kPsY : kBriY;
    tft->drawRGBBitmap(0, y, canvas->getBuffer(), kW, h);
  }

  void redrawPicker() {
    buildPickItems(false);
    uint32_t sig = hashBytes(&pickFirst, sizeof(pickFirst), hashBytes(&screen, 1));
    for (auto& it : pickItems) sig = hashBytes(&it, sizeof(it), sig);
    const bool fetching = [&] { Lock l(mtx); return devs[selIndexLocked()].presetsFetching; }();
    sig = hashBytes(&fetching, 1, sig);
    if (sig == pickSig) return;
    pickSig = sig;

    // title bar
    canvas->fillScreen(C_BG);
    canvas->setFont(&FreeSansBold12pt7b);
    canvas->setTextSize(1);
    fitText(*canvas, screen == SCR_PICK_DEVICE ? "Devices" : "Presets", 10, 25, kW - 70, C_TEXT);
    canvas->fillRoundRect(kW - 46, 4, 38, 28, 8, C_TRACK);
    fitText(*canvas, "X", kW - 35, 25, 30, C_ACCENT);
    canvas->drawFastHLine(0, kPickTitleH - 1, kW, C_TRACK);
    tft->drawRGBBitmap(0, 0, canvas->getBuffer(), kW, kPickTitleH);

    for (int i = 0; i < kVisible; i++) {
      const int idx = pickFirst + i;
      canvas->fillScreen(C_BG);
      if (idx < int(pickItems.size())) {
        const PickItem& it = pickItems[idx];
        if (it.header) {
          // Render header row: dim, small, centered separator style
          canvas->setFont(nullptr);
          canvas->setTextSize(1);
          canvas->setTextColor(C_DIM);
          int16_t x1, y1; uint16_t w, h;
          canvas->getTextBounds(it.label, 0, 0, &x1, &y1, &w, &h);
          int xc = (kW - int(w)) / 2;
          canvas->setCursor(xc, 20);
          canvas->print(it.label);
          canvas->drawFastHLine(0, kItemH / 2 + 2, kW, C_DIM);
        } else {
          // Render regular item
          if (it.current) { canvas->fillRect(0, 0, kW, kItemH, C_CUR); canvas->fillRect(0, 0, 4, kItemH, C_ACCENT); }
          canvas->setFont(&FreeSans9pt7b);
          fitText(*canvas, it.label, 12, 18, kW - 24, C_TEXT);
          canvas->setFont(nullptr);
          canvas->setTextColor(C_DIM);
          canvas->setCursor(12, 28);
          canvas->print(it.sub);
          canvas->drawFastHLine(0, kItemH - 1, kW, C_TRACK);
        }
      } else if (pickItems.empty() && i == 0) {
        canvas->setFont(&FreeSans9pt7b);
        fitText(*canvas, fetching ? "loading..." : "nothing here", 12, 26, kW - 24, C_DIM);
      }
      tft->drawRGBBitmap(0, kPickTitleH + i * kItemH, canvas->getBuffer(), kW, kItemH);
    }
    const int usedH = kPickTitleH + kVisible * kItemH;
    if (usedH < kH) tft->fillRect(0, usedH, kW, kH - usedH, C_BG);
    if (int(pickItems.size()) > kVisible) {            // scroll thumb
      const int trackH = kVisible * kItemH;
      const int thumbH = std::max(16, trackH * kVisible / int(pickItems.size()));
      const int thumbY = kPickTitleH + (trackH - thumbH) * pickFirst / std::max(1, maxFirst());
      tft->fillRect(kW - 4, thumbY, 3, thumbH, C_DIM);
    }
  }

  // --- worker: HTTP to other WLED devices -----------------------------------------------------
  static void workerEntry(void* self) { static_cast<ControllerUsermod*>(self)->workerLoop(); }

  bool takeReq(Req& out) {
    Lock l(mtx);
    uint32_t now = millis();
    for (auto& r : reqs) {
      if (!r.ip) continue;
      bool actionable = r.power || r.ps >= 0 || r.poll || r.presets;
      if (!actionable && r.bri < 0) { r = Req(); continue; }
      if (!actionable && int32_t(now - r.briDueAt) < 0) continue;
      out = r;
      r = Req();
      return true;
    }
    return false;
  }

  void workerLoop() {
    for (;;) {
      if (!WLED_CONNECTED) { vTaskDelay(pdMS_TO_TICKS(500)); continue; }
      Req r;
      if (takeReq(r)) { doRequest(r); continue; }
      uint32_t now = millis();
      if (!lastMdns || now - lastMdns >= kMdnsMs) { lastMdns = now ? now : 1; scanMdns(); }
      sweepStep(now);
      vTaskDelay(pdMS_TO_TICKS(30));
    }
  }

  // Returns true and fills `doc` when the device answered with HTTP 200 and valid JSON.
  bool http(uint32_t ip, bool post, const char* path, const String& body, JsonDocument& doc, JsonDocument* filter,
            uint16_t connectMs = 1500) {
    WiFiClient client;
    HTTPClient h;
    h.setConnectTimeout(connectMs);
    h.setTimeout(3000);
    h.setReuse(false);
    if (!h.begin(client, ipStr(ip), 80, path)) return false;
    int code;
    if (post) { h.addHeader("Content-Type", "application/json"); code = h.POST(body); }
    else code = h.GET();
    bool ok = false;
    if (code == HTTP_CODE_OK) {
      DeserializationError err;
      if (filter && h.getSize() > 0) err = deserializeJson(doc, h.getStream(), DeserializationOption::Filter(*filter));
      else if (filter) err = deserializeJson(doc, h.getString(), DeserializationOption::Filter(*filter));
      else err = deserializeJson(doc, h.getStream());
      ok = !err;
    }
    h.end();
    return ok;
  }

  void applyRemoteState(uint32_t ip, JsonObject st, bool fromCommand) {
    if (st.isNull()) return;
    Lock l(mtx);
    for (auto& d : devs) {
      if (d.ip != ip) continue;
      // A poll that was in flight while the user clicked would flash the old value; skip it.
      if (!fromCommand && millis() - d.lastCmdAt < 1200) return;
      d.on = st["on"] | d.on;
      d.bri = st["bri"] | d.bri;
      d.ps = st["ps"] | 0;
      if (st["ps"].isNull() || d.ps == 255) d.ps = 0;
      d.stateKnown = true;
      return;
    }
  }

  void setReachable(uint32_t ip, bool ok) {
    Lock l(mtx);
    for (auto& d : devs) if (d.ip == ip) { d.reachable = ok; return; }
  }

  void doRequest(const Req& r) {
    bool ok = true;
    if (r.power || r.bri >= 0 || r.ps >= 0) {
      StaticJsonDocument<128> body;
      StaticJsonDocument<192> resp, filter;
      filter["on"] = true; filter["bri"] = true; filter["ps"] = true;
      if (r.power == 1) body["on"] = true;
      else if (r.power == 2) body["on"] = false;
      else if (r.power == 3) body["on"] = "t";
      if (r.bri >= 0) body["bri"] = r.bri;
      if (r.ps >= 0) body["ps"] = r.ps;
      body["v"] = true;                 // ask for the resulting state in the response
      String s;
      serializeJson(body, s);
      if (http(r.ip, true, "/json/state", s, resp, &filter)) applyRemoteState(r.ip, resp.as<JsonObject>(), true);
      else ok = false;
    }
    if (ok && r.poll) {
      StaticJsonDocument<192> resp, filter;
      filter["on"] = true; filter["bri"] = true; filter["ps"] = true;
      if (http(r.ip, false, "/json/state", String(), resp, &filter)) applyRemoteState(r.ip, resp.as<JsonObject>(), false);
      else ok = false;
    }
    if (r.presets) {
      DynamicJsonDocument resp(24576);   // ~70 bytes per preset (250 max)
      // Each filter key takes a slot. 64 bytes held only n + playlist.ps; adding "ql" overflowed it, and ArduinoJson then
      // silently drops the keys that did not fit, so scenes marked by a label never showed. Keep headroom and check.
      StaticJsonDocument<256> filter;
      filter["*"]["n"] = true;
      filter["*"]["playlist"]["ps"] = true;   // only its presence matters; "ps" is always set in a playlist
      filter["*"]["ql"] = true;               // quick-load label: marks a plain preset as a scene
      if (filter.overflowed()) CTL_LOG("[ctl] preset filter overflowed: scenes will be incomplete\n");
      bool got = resp.capacity() && http(r.ip, false, "/presets.json", String(), resp, &filter);
      std::vector<Preset> list;
      if (got) {
        for (JsonPair kv : resp.as<JsonObject>()) {
          int id = atoi(kv.key().c_str());
          if (id <= 0 || id > 250) continue;
          Preset p{};
          parsePreset(kv.value().as<JsonObject>(), id, p);
          list.push_back(p);
        }
        ctl::orderPresets(list);
      }
      Lock l(mtx);
      for (auto& d : devs) if (d.ip == r.ip) {
        d.presetsFetching = false;
        if (got) { d.presets.swap(list); d.presetsLoaded = true; }
      }
      ok &= got;
    }
    setReachable(r.ip, ok);
  }

  // One probe of the unicast sweep, run between requests so control stays responsive. It runs regardless of what
  // the node list or mDNS found: on an AP that delivers multicast only some of the time they return a partial
  // list (or none), and nothing says how many devices to expect. A dead address costs one connect timeout, so a
  // /24 pass takes about a minute and then idles for kSweepIdleMs.
  void sweepStep(uint32_t now) {
    if (!sweepHost) {
      if (sweepDoneAt && now - sweepDoneAt < kSweepIdleMs) return;
      sweepHost = 1;
      Lock l(mtx);
      for (auto& h : sweepHits) h.missed++;
    }
    const uint32_t self = (uint32_t)WLEDNetwork.localIP();
    IPAddress base(self);
    base[3] = (uint8_t)sweepHost;
    const uint32_t ip = (uint32_t)base;
    const uint8_t host = (uint8_t)sweepHost;
    if (++sweepHost > 254) {
      sweepHost = 0;
      sweepDoneAt = now ? now : 1;
      Lock l(mtx);
      sweepHits.erase(std::remove_if(sweepHits.begin(), sweepHits.end(),
                                     [](const MdnsHit& h) { return h.missed >= 2; }), sweepHits.end());
      CTL_LOG("[ctl] sweep done: %u WLED found\n", (unsigned)sweepHits.size());
    }
    if (ip == self || host == 255) return;
    {
      Lock l(mtx);
      for (auto& h : mdnsHits) if (h.ip == ip) return;   // already known through mDNS
    }
    StaticJsonDocument<64> filter;
    filter["brand"] = true;
    filter["name"] = true;
    StaticJsonDocument<192> doc;
    if (!http(ip, false, "/json/info", String(), doc, &filter, kSweepProbeMs)) return;
    if (strcmp(doc["brand"] | "", "WLED") != 0) return;
    MdnsHit hit{};
    hit.ip = ip;
    strlcpy(hit.name, doc["name"] | "", sizeof(hit.name));
    Lock l(mtx);
    for (auto& h : sweepHits) {
      if (h.ip != ip) continue;
      h.missed = 0;
      strlcpy(h.name, hit.name, sizeof(h.name));
      return;
    }
    if (sweepHits.size() < 16) {
      sweepHits.push_back(hit);
      CTL_LOG("[ctl] sweep found %s %s\n", hit.name, ipStr(ip).c_str());
    }
  }

  // mDNS finds WLED devices that have the node list broadcast turned off.
  void scanMdns() {
    const int n = MDNS.queryService("wled", "tcp");
    std::vector<MdnsHit> seen;
    for (int i = 0; i < n && i < 16; i++) {
      MdnsHit h{};
      h.ip = (uint32_t)MDNS.address(i);
      if (!h.ip) continue;
      strlcpy(h.name, MDNS.hostname(i).c_str(), sizeof(h.name));
      seen.push_back(h);
    }
    Lock l(mtx);
    // One mDNS query is lossy (multicast has no retries, and a device at the edge of WiFi range misses some).
    // Replacing the whole list each scan made such devices flicker out; instead merge, and drop a device only
    // after it has been missed kMdnsMaxMissed scans in a row.
    for (auto& old : mdnsHits) old.missed++;
    for (auto& h : seen) {
      bool known = false;
      for (auto& old : mdnsHits) {
        if (old.ip != h.ip) continue;
        old.missed = 0;
        strlcpy(old.name, h.name, sizeof(old.name));
        known = true;
        break;
      }
      if (!known && mdnsHits.size() < 16) mdnsHits.push_back(h);
    }
    mdnsHits.erase(std::remove_if(mdnsHits.begin(), mdnsHits.end(),
                                  [](const MdnsHit& h) { return h.missed >= kMdnsMaxMissed; }),
                   mdnsHits.end());
    CTL_LOG("[ctl] mDNS: query=%d seen=%u kept=%u nodes=%u sweep=%u\n", n, (unsigned)seen.size(),
            (unsigned)mdnsHits.size(), (unsigned)nodeCount, (unsigned)sweepHits.size());
  }
};

static ControllerUsermod controllerUsermod;
REGISTER_USERMOD(controllerUsermod);
