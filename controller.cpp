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
// Threading: everything that touches WLED state (the local device, the Nodes list, WLEDNetwork / WLED_CONNECTED,
// the config) runs in loop() on the main task. HTTP and mDNS for other devices run in one worker task that never
// touches WLED globals: the main task publishes what the worker needs (network address and mask, whether it is
// connected, whether the screen is on, the node list's addresses) in `net` / `nodeIps`. The two tasks share those,
// `devs`, `reqs`, the discovery lists and `workerMsg`, all guarded by `mtx`. Config written from the web server task
// (`sweepEnabled`) is atomic.
#include "wled.h"

#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <Wire.h>
#include <WiFiClient.h>
#include <mdns.h>
#include <algorithm>
#include <atomic>
#include <vector>

#include "ctl_logic.h"      // pure logic (slider, touch decode, scrolling, device list, HTTP head), host-tested
#include "ctl_json.h"       // preset list parsing, host-tested against ArduinoJson 6.18.1

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
constexpr uint32_t kPollMs = 2500, kMdnsMs = 30000, kMdnsQueryMs = 3000;
constexpr uint32_t kSweepIdleMs = 60000;    // pause between unicast sweeps of the local subnet
constexpr uint16_t kSweepProbeMs = 350;     // connect timeout per address; a device at -88 dBm needs more than 200 ms
constexpr uint32_t kSweepProbeTotalMs = 1000;   // whole probe, connect to parsed reply
constexpr uint8_t kMdnsMaxMissed = 3;   // a device that answers mDNS only some of the time (weak WiFi) stays listed
constexpr size_t kMaxHits = 64;         // per discovery source (mDNS, sweep); more is logged and ignored
constexpr uint32_t kBriDebounceMs = 150, kPickIdleMs = 20000;
constexpr uint8_t kTouchAddr = 0x63, kTouchIdReg = 0x08, kTouchDataReg = 0x01;
// Remote HTTP: connect timeout, a deadline for the whole exchange, and a byte cap for the body.
constexpr uint16_t kConnectMs = 1500;
constexpr uint32_t kStateTotalMs = 4000, kPresetsTotalMs = 15000;
constexpr size_t kStateMaxBytes = 32768;    // /json/state with "v": every segment of a 32-segment setup
constexpr size_t kInfoMaxBytes = 8192;      // /json/info is ~1.5 KB
constexpr size_t kPresetsMaxBytes = 262144; // presets.json is streamed through the filter, never held whole

enum Screen : uint8_t { SCR_MAIN, SCR_PICK_DEVICE, SCR_PICK_PRESET };
enum Region : int8_t { R_NONE = -1, R_DEVICE, R_POWER, R_PRESET, R_BRI };
enum TouchEv : uint8_t { T_DOWN, T_MOVE, T_UP };

struct Preset { uint8_t id; char name[24]; bool scene; };

struct Dev {
  uint32_t ip = 0;               // 0 = this device
  bool local = false;
  char name[33] = {};
  bool dupName = false;          // another device has the same name: show the address with it
  bool listed = true;            // false: kept only because it is selected, no discovery source reports it
  bool on = false;
  uint8_t bri = 128;             // brightness to show (the remembered one while off)
  uint8_t ps = 0;                // active preset id, 0 = none
  bool stateKnown = false;
  bool reachable = true;
  uint8_t fails = 0;             // consecutive failed requests (ctl::reachableAfter)
  uint32_t lastCmdAt = 0;        // ignore polled state right after a command (it may be stale)
  std::vector<Preset> presets;
  bool presetsLoaded = false;
  bool presetsPartial = false;   // the list did not fit in memory: only the first part is shown
  bool presetsFetching = false;
  uint8_t presetFails = 0;       // consecutive failed fetches, for the retry backoff
  uint32_t presetsAt = 0;        // last attempt
};

// One pending request per remote device; later requests for the same device merge in.
struct Req {
  uint32_t ip = 0;
  uint8_t power = 0;             // 0 none, 1 on, 2 off
  int16_t bri = -1;
  int16_t ps = -1;
  bool poll = false;
  bool presets = false;
  uint32_t briDueAt = 0;         // debounce: a lone brightness request waits until this time
};

struct MdnsHit { uint32_t ip; char name[33]; uint8_t missed; };   // missed: consecutive scans that did not see it

struct PickItem { char label[40]; char sub[32]; uint32_t ip; uint8_t id; bool current; bool header; };

// What the main task publishes for the worker (under mtx).
// mdnsReady: connected AND WLED has (re)started its mDNS responder for this connection (usermod connected() runs right
// after that), so a query is not started on a responder that is about to be torn down.
struct Net { bool up = false, mdnsReady = false; uint32_t self = 0, mask = 0; bool awake = true; };

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

// Dotted quad without a String allocation. `ip` is IPAddress's raw value (first octet in the low byte).
void ipText(uint32_t ip, char* out, size_t n) {
  const IPAddress a(ip);
  snprintf(out, n, "%u.%u.%u.%u", a[0], a[1], a[2], a[3]);
}
uint32_t hostOrder(uint32_t raw) {
  const IPAddress a(raw);
  return uint32_t(a[0]) << 24 | uint32_t(a[1]) << 16 | uint32_t(a[2]) << 8 | a[3];
}
uint32_t rawOrder(uint32_t host) {
  return uint32_t(IPAddress(uint8_t(host >> 24), uint8_t(host >> 16), uint8_t(host >> 8), uint8_t(host)));
}

// Name as shown: a device that shares its name with another gets its last address octet added.
void displayName(const Dev& d, char* out, size_t n) {
  if (d.dupName && !d.local) snprintf(out, n, "%s (.%u)", d.name, IPAddress(d.ip)[3]);
  else snprintf(out, n, "%s", d.name);
}

// The response body for ArduinoJson: ends at the byte limit, at the deadline, or when the server closes. read()
// waits for data itself (WiFiClient::read never blocks), and the caller sets setTimeout(0) so Stream::timedRead does
// not add a second wait on top. The old code had only HTTPClient's idle timeout, which restarts on every byte, and
// read bodies without a Content-Length into one unbounded String: a slow or broken host stalled the worker forever.
class BodyStream : public Stream {
 public:
  BodyStream(WiFiClient& c, long limit, uint32_t deadline) : c_(c), left_(limit), deadline_(deadline) {}
  int available() override { return left_ > 0 ? int(std::min<long>(c_.available(), left_)) : 0; }
  int read() override {
    if (!waitData()) return -1;
    const int ch = c_.read();
    if (ch >= 0) left_--;
    return ch;
  }
  int peek() override { return waitData() ? c_.peek() : -1; }
  size_t write(uint8_t) override { return 0; }
  bool timedOut() const { return timedOut_; }

 private:
  bool waitData() {
    while (left_ > 0) {
      if (c_.available() > 0) return true;
      if (!c_.connected()) return false;
      if (int32_t(millis() - deadline_) >= 0) { timedOut_ = true; return false; }
      vTaskDelay(pdMS_TO_TICKS(2));
    }
    return false;
  }
  WiFiClient& c_;
  long left_;
  uint32_t deadline_;
  bool timedOut_ = false;
};

enum HttpResult : uint8_t { HTTP_OK, HTTP_UNREACHABLE, HTTP_TIMEOUT, HTTP_BAD_REPLY, HTTP_TOO_BIG, HTTP_NO_MEMORY };

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
  std::atomic<bool> sweepEnabled{true};   // unicast sweep of the subnet (written by the web server task)

  void setup() override {
    if (!enabled) return;
    if (!begin()) {
      // A runtime flag, not `enabled = false`: that would be saved with the next config write and keep the usermod
      // off after the conflict is fixed.
      failed = true;
      DEBUG_PRINTF_P(PSTR("[ctl] setup failed: %s\n"), failReason);
      return;
    }
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
      refreshDevices();
      schedulePolls(now);
      takeWorkerMessage(now);
    }
    flushBrightness(now);
    if (awake && now - lastDraw >= 40) { lastDraw = now; redraw(); }
  }

  void connected() override {
    mdnsKick = true;                     // scan right away instead of within kMdnsMs
    cachedIp = 0xFFFFFFFF;               // re-read SSID and address
    connectedSeen = true;                // WLED has just restarted mDNS (initInterfaces) for this connection
  }

  uint16_t getId() override { return USERMOD_ID_UNSPECIFIED; }

  void addToConfig(JsonObject& root) override {
    JsonObject top = root.createNestedObject(F("Controller"));
    top[F("enabled")] = enabled;
    top[F("screen timeout s")] = screenTimeoutS;
    top[F("lan sweep")] = sweepEnabled.load();
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
    bool sweep = sweepEnabled;
    complete &= getJsonValue(top[F("enabled")], enabled, true);
    complete &= getJsonValue(top[F("screen timeout s")], screenTimeoutS, 60);
    complete &= getJsonValue(top[F("lan sweep")], sweep, true);
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
    sweepEnabled = sweep;
    return complete;
  }

  void appendConfigData() override {
    oappend(F("addInfo('Controller:pin mosi',1,'restart needed after changing pins');"));
    oappend(F("addInfo('Controller:lan sweep',1,'probe every address of the subnet for WLED (finds devices "
              "when multicast is blocked)');"));
    if (failed && failReason) {
      char buf[160];
      snprintf(buf, sizeof(buf), "addInfo('Controller:enabled',1,'<b>not running: %s</b>');", failReason);
      oappend(buf);
    }
  }

 private:
  // --- state shared with the worker (guard with mtx) ------------------------------------------
  SemaphoreHandle_t mtx = nullptr;
  std::vector<Dev> devs;               // [0] is this device, the rest sorted by name
  Req reqs[4];
  std::vector<MdnsHit> mdnsHits;
  // Unicast fallback. Some APs/mesh nodes never deliver multicast or broadcast to their clients, which silences
  // both the WLED node list (UDP broadcast) and mDNS. Unicast HTTP still works there, so the worker probes the
  // subnet for WLED devices. missed counts whole sweeps, not scans.
  std::vector<MdnsHit> sweepHits;
  uint32_t hitsGen = 0;                // bumped whenever mdnsHits/sweepHits change, so the list is rebuilt
  std::vector<uint32_t> nodeIps;       // addresses in WLED's node list (the sweep skips them)
  Net net;
  char workerMsg[40] = {};             // a message for the header, set by the worker
  volatile uint8_t nodeCount = 0;      // Nodes.size(), for the debug log
  std::atomic<bool> mdnsKick{true};

  // --- worker-only state ------------------------------------------------------------------------
  uint32_t lastMdns = 0;
  mdns_search_once_t* mdnsSearch = nullptr;
  bool sweeping = false;
  uint32_t sweepNext = 0, sweepEnd = 0, sweepSelf = 0;   // host byte order
  uint32_t sweepDoneAt = 0;            // when the last sweep ended

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
  bool ready = false, failed = false, touchOk = false;
  const char* failReason = nullptr;
  const char* touchNote = nullptr;     // why touch is off, when it is not just "not found"
  Panel* tft = nullptr;
  GFXcanvas16* canvas = nullptr;

  bool awake = true;
  bool connectedSeen = false;          // connected() ran since WiFi last came up
  uint32_t lastActivity = 0, lastDraw = 0, lastHousekeeping = 0, lastPoll = 0;
  uint32_t selIp = 0;                  // selected device (0 = this one)
  Screen screen = SCR_MAIN;
  bool localPresetsDirty = false;
  char statusMsg[40] = {};
  uint32_t statusUntil = 0;
  uint32_t devSig = 0;                 // signature of the discovery sources the device list was built from
  bool devSigValid = false;

  // pending (debounced) remote brightness
  uint32_t briSendAt = 0; uint32_t briSendIp = 0; int16_t briSendVal = -1;

  char wifiSsid[33] = {}, wifiIp[16] = {};   // cached in refreshLocal (WiFi.SSID() allocates)
  uint32_t cachedIp = 0xFFFFFFFF;            // address wifiSsid/wifiIp were read for
  uint32_t regionSig[5] = {};          // header + 4 rows; skip redraw when unchanged
  uint32_t pickSig = 0;

  // list state
  std::vector<PickItem> pickItems;
  int pickFirst = 0;                   // index of the first visible item
  uint32_t lastPickBuild = 0;

  // --- setup --------------------------------------------------------------------------------
  bool fail(const char* why) { failReason = why; return false; }

  bool begin() {
    mtx = xSemaphoreCreateMutex();
    if (!mtx) return fail("out of memory");
    devs.emplace_back();
    devs[0].local = true;

    // WLED's own global SPI / I2C (Config > Usermods, "global" pins) are started before usermods. If they use our
    // pins, share them; if they use other pins, a second begin() would be silently ignored by the core.
    const ctl::BusUse spiUse = ctl::busUse(spi_sclk, spi_mosi, pinSclk, pinMosi);
    if (spiUse == ctl::BUS_CONFLICT) return fail("WLED global SPI is set to other pins than the display");
    bool touchPins = pinTsda >= 0 && pinTscl >= 0;
    const ctl::BusUse i2cUse = touchPins ? ctl::busUse(i2c_sda, i2c_scl, pinTsda, pinTscl) : ctl::BUS_OWN;
    if (i2cUse == ctl::BUS_CONFLICT) { touchPins = false; touchNote = "touch: I2C pin clash"; }

    managed_pin_type pins[10];
    size_t np = 0;
    auto add = [&](int8_t p, bool out) { if (p >= 0) pins[np++] = {p, out}; };
    if (spiUse == ctl::BUS_OWN) { add(pinMosi, true); add(pinSclk, true); }   // shared: WLED owns them
    add(pinCs, true); add(pinDc, true); add(pinRst, true); add(pinBl, true);
    if (touchPins) {
      if (i2cUse == ctl::BUS_OWN) { add(pinTsda, true); add(pinTscl, true); }
      add(pinTrst, true);
      add(pinTint, false);
    }
    if (!PinManager::allocateMultiplePins(pins, np, PinOwner::UM_Unspecified))
      return fail("display/touch pins are in use");

    canvas = new GFXcanvas16(kW, kBriH);       // tallest region; every other region is shorter
    if (!canvas || !canvas->getBuffer()) {
      delete canvas; canvas = nullptr;
      PinManager::deallocateMultiplePins(pins, np, PinOwner::UM_Unspecified);
      return fail("out of memory for the frame buffer");
    }
    pinMode(pinBl, OUTPUT);
    digitalWrite(pinBl, LOW);
    if (spiUse == ctl::BUS_OWN) SPI.begin(pinSclk, -1, pinMosi, pinCs);
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
    digitalWrite(pinBl, HIGH);
    lastActivity = millis();
    touchOk = touchPins && touchInit(i2cUse == ctl::BUS_OWN);
    reloadLocalPresets();

    if (xTaskCreate(&ControllerUsermod::workerEntry, "ctl-http", 10240, this, 1, nullptr) != pdPASS) {
      showFailure("out of memory for the HTTP task");
      return fail("out of memory for the HTTP task");
    }
    return true;
  }

  void showFailure(const char* why) {   // the display works but the usermod will not run: say so on it
    tft->fillScreen(C_BG);
    tft->setFont(nullptr);
    tft->setTextColor(C_ERR);
    tft->setCursor(4, 4);
    tft->print("controller not running:");
    tft->setCursor(4, 16);
    tft->print(why);
  }

  // --- touch ----------------------------------------------------------------------------------
  bool touching = false, tIgnore = false, tMoved = false, tSlider = false;
  int tStartX = 0, tStartY = 0, tLastX = 0, tLastY = 0, tAccum = 0;
  Region tRegion = R_NONE;             // row being pressed (highlight + tap target)
  uint32_t lastTouchRead = 0, touchDownAt = 0, lastTouchReset = 0;
  uint8_t touchFails = 0;
  ctl::ResetPulse resetPulse;

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

  // Reset pulse from Waveshare's driver. In setup() it may block (the chip ID is read right after); in loop() it
  // runs as ctl::ResetPulse, stepped by pollTouch, so WLED is never frozen for half a second.
  void startTouchReset(uint32_t now) {
    if (pinTrst < 0) return;
    CTL_LOG("[ctl] %lu touch reset begin\n", (unsigned long)now);
    pinMode(pinTrst, OUTPUT);
    digitalWrite(pinTrst, LOW);
    resetPulse.start(now);
  }

  bool touchInit(bool startBus) {
    if (startBus) Wire.begin(pinTsda, pinTscl, 400000);
    if (pinTrst >= 0) {
      pinMode(pinTrst, OUTPUT);
      digitalWrite(pinTrst, LOW);  delay(ctl::ResetPulse::kLowMs);
      digitalWrite(pinTrst, HIGH); delay(ctl::ResetPulse::kBootMs);
    }
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
    if (resetPulse.busy()) {
      const ctl::ResetPulse::Action a = resetPulse.step(now);
      if (a == ctl::ResetPulse::DRIVE_HIGH) digitalWrite(pinTrst, HIGH);
      if (a == ctl::ResetPulse::FINISHED) CTL_LOG("[ctl] %lu touch reset end\n", (unsigned long)now);
      return;                          // the chip is not readable during the pulse
    }
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
#else
    (void)flag;
#endif
    if (!readOk) {
      if (++touchFails >= 5 && now - lastTouchReset > 10000) {
        CTL_LOG("[ctl] touch not answering, resetting it\n");
        lastTouchReset = now;
        touchFails = 0;
        if (touching) { touching = false; onTouch(T_UP, tLastX, tLastY, now); }
        startTouchReset(now);
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
  // The selected device, or null. refreshDevices keeps the selected device in `devs` (ctl::mergeDevices), so null
  // is only a transient; callers then do nothing. They must never fall back to devs[0]: that sent the user's
  // command to THIS device when a remote one briefly dropped out of discovery.
  Dev* selDevLocked() {                // call with mtx held
    const int i = ctl::findDevice(devs, selIp);
    return i < 0 ? nullptr : &devs[i];
  }

  // The local device mirrors WLED's globals; presets are reloaded on demand. Also publishes what the worker needs.
  void refreshLocal() {
    const bool up = WLED_CONNECTED;
    const uint32_t self = up ? uint32_t(WLEDNetwork.localIP()) : 0;
    const uint32_t mask = up ? uint32_t(WLEDNetwork.subnetMask()) : 0;
    if (self != cachedIp) {            // SSID / address strings allocate: read them only when the address changes
      cachedIp = self;
      if (up) {
        strlcpy(wifiSsid, WiFi.SSID().c_str(), sizeof(wifiSsid));
        ipText(self, wifiIp, sizeof(wifiIp));
      } else {
        wifiSsid[0] = wifiIp[0] = 0;
      }
    }
    if (!up) connectedSeen = false;
    Lock l(mtx);
    net.up = up; net.mdnsReady = up && connectedSeen; net.self = self; net.mask = mask; net.awake = awake;
    Dev& d = devs[0];
    strlcpy(d.name, serverDescription, sizeof(d.name));
    d.on = bri > 0;
    d.bri = bri ? bri : briLast;
    d.ps = currentPreset;
    d.stateKnown = true;
    d.reachable = true;
  }

  // Rebuild the device list from WLED's node list, mDNS and the sweep, but only when one of them (or the network, or
  // the selection) changed: rebuilding every 500 ms allocated a String per node and a new vector each time.
  void refreshDevices() {
    const bool up = WLED_CONNECTED;
    const uint32_t self = up ? uint32_t(WLEDNetwork.localIP()) : 0;
    const uint32_t mask = up ? uint32_t(WLEDNetwork.subnetMask()) : 0;
    ctl::Hasher sig;
    sig.add(up).add(self).add(mask).add(selIp);
    if (up) {
      nodeCount = Nodes.size() > 255 ? 255 : Nodes.size();
      for (auto& kv : Nodes) sig.add(uint32_t(kv.second.ip)).str(kv.second.nodeName.c_str()).add(bool(kv.second.on));
    }
    Lock l(mtx);
    sig.add(hitsGen);
    if (devSigValid && sig.h == devSig) return;
    devSig = sig.h;
    devSigValid = true;

    std::vector<ctl::DevCand> cands;
    nodeIps.clear();
    if (up) {
      cands.reserve(Nodes.size() + mdnsHits.size() + sweepHits.size());
      for (auto& kv : Nodes) {
        ctl::DevCand c;
        c.ip = uint32_t(kv.second.ip);
        if (!c.ip) continue;
        strlcpy(c.name, kv.second.nodeName.c_str(), sizeof(c.name));
        c.hasOn = true;
        c.on = kv.second.on;
        cands.push_back(c);
        nodeIps.push_back(c.ip);
      }
      for (const auto* list : {&mdnsHits, &sweepHits})
        for (const MdnsHit& h : *list) {
          ctl::DevCand c;
          c.ip = h.ip;
          strlcpy(c.name, h.name, sizeof(c.name));
          cands.push_back(c);
        }
      for (auto& c : cands) if (!c.name[0]) ipText(c.ip, c.name, sizeof(c.name));
    }
    ctl::mergeDevices(devs, cands, selIp, self, mask);
    if (!up) {                         // nothing remote is reachable; the selected device stays, blank
      for (size_t i = 1; i < devs.size(); i++) { devs[i].reachable = false; devs[i].stateKnown = false; }
    }
    if (ctl::findDevice(devs, selIp) < 0) selIp = 0;   // never listed (cannot happen through the picker)
#ifdef CTL_DEBUG
    CTL_LOG("[ctl] device list (%u):", (unsigned)devs.size());
    for (auto& d : devs) {
      char ip[16];
      ipText(d.ip, ip, sizeof(ip));
      CTL_LOG(" %s=%s%s", d.name, ip, d.listed ? "" : "(kept)");
    }
    CTL_LOG("\n");
#endif
  }

  void schedulePolls(uint32_t now) {
    if (!awake || !WLED_CONNECTED) return;
    Lock l(mtx);
    Dev* d = selDevLocked();
    if (!d || d->local) return;
    // Until the first reply the controls are blank, so ask again every 500 ms instead of every kPollMs; but not
    // while the device does not answer, which would keep the worker in connect timeouts.
    const uint32_t interval = d->stateKnown || !d->reachable ? kPollMs : 500;
    if (now - lastPoll < interval) return;
    lastPoll = now;
    Req* r = slotFor(d->ip);
    if (r) r->poll = true;
  }

  // Fetch the selected remote's presets the first time it is selected, and retry after failures with a backoff.
  void ensurePresetsLocked(Dev& d, uint32_t now, bool force) {
    if (d.local || !WLED_CONNECTED || d.presetsFetching) return;
    if (d.presetsLoaded && !force) return;
    if (!force && d.presetsAt && now - d.presetsAt < ctl::presetRetryMs(d.presetFails)) return;
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

  void takeWorkerMessage(uint32_t now) {
    char msg[sizeof(workerMsg)];
    {
      Lock l(mtx);
      if (!workerMsg[0]) return;
      memcpy(msg, workerMsg, sizeof(msg));
      workerMsg[0] = 0;
    }
    say(msg, now, 4000);
  }

  // --- local presets ------------------------------------------------------------------------
  void reloadLocalPresets() {
    if (!requestJSONBufferLock(JSON_LOCK_UNKNOWN)) { localPresetsDirty = true; return; }
    std::vector<Preset> list;
    StaticJsonDocument<256> filter;    // same filter as for remote lists: big playlists must not fill pDoc
    ctl::presetFilter(filter);
    if (readObjectFromFile(getPresetsFileName(), nullptr, pDoc, &filter))
      ctl::presetsFromJson(pDoc->as<JsonObjectConst>(), list);
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
      case R_POWER: togglePower(now); break;
      case R_DEVICE: openPicker(SCR_PICK_DEVICE, now); break;
      case R_PRESET: openPicker(SCR_PICK_PRESET, now); break;
      default: break;
    }
  }

  void openPicker(Screen s, uint32_t now) {
    if (s == SCR_PICK_PRESET) {
      if (selIp == 0) reloadLocalPresets();   // picks up presets saved since boot
      Lock l(mtx);
      if (Dev* d = selDevLocked()) ensurePresetsLocked(*d, now, true);   // refresh so edits made elsewhere show up
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
      Lock l(mtx);
      const int di = ctl::findDevice(devs, it.ip);        // the list may be up to 200 ms old
      if (di < 0) {
        say("device is gone", now);
      } else {
        selIp = it.ip;
        lastPoll = 0;                  // poll the newly selected device on the next housekeeping pass
        if (it.ip == 0) localPresetsDirty = true;
        else ensurePresetsLocked(devs[di], now, false);
      }
    } else {
      applyPresetTo(it.ip, it.id, now);
    }
    closePicker();
  }

  // Rebuild the list contents from the shared device state.
  void buildPickItems(bool force) {
    uint32_t now = millis();
    if (!force && now - lastPickBuild < 200) return;
    lastPickBuild = now;
    std::vector<PickItem> items;
    Lock l(mtx);
    if (screen == SCR_PICK_DEVICE) {
      items.reserve(devs.size());
      for (auto& d : devs) {
        PickItem it{};
        displayName(d, it.label, sizeof(it.label));
        if (d.local) {
          strlcpy(it.sub, "this device", sizeof(it.sub));
        } else {
          char ip[16];
          ipText(d.ip, ip, sizeof(ip));
          snprintf(it.sub, sizeof(it.sub), "%s%s", ip, !d.reachable ? " unreachable" : !d.listed ? " not found" : "");
        }
        it.ip = d.ip; it.current = d.ip == selIp;
        items.push_back(it);
      }
    } else if (Dev* dp = selDevLocked()) {
      Dev& d = *dp;
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
          it.ip = d.ip; it.id = p.id; it.current = p.id == d.ps;
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
        it.ip = d.ip; it.id = p.id; it.current = p.id == d.ps;
        items.push_back(it);
      }
      if (d.presetsPartial) {              // the list did not fit in memory: say so at its end
        PickItem note{};
        strlcpy(note.label, "(list incomplete)", sizeof(note.label));
        note.header = true;
        items.push_back(note);
      }
    }
    pickItems.swap(items);
    pickFirst = constrain(pickFirst, 0, maxFirst());
  }

  void togglePower(uint32_t now) {
    Lock l(mtx);
    Dev* d = selDevLocked();
    if (!d || !d->stateKnown) return;
    d->lastCmdAt = now;
    if (d->local) {
      toggleOnOff();
      stateUpdated(CALL_MODE_DIRECT_CHANGE);
    } else if (Req* r = slotFor(d->ip)) {
      // Send the state the screen now shows, not a toggle: two taps before the worker runs merge into one
      // request, and a merged toggle flipped the device once while the screen flipped twice.
      d->on = !d->on;
      r->power = d->on ? 1 : 2;
    } else say("busy, try again", now);
  }

  // `ip` is the device the preset list was built for; it must still be the selected one.
  void applyPresetTo(uint32_t ip, uint8_t id, uint32_t now) {
    {
      Lock l(mtx);
      Dev* d = selDevLocked();
      if (!d || d->ip != ip) { say("device changed", now); return; }
      d->lastCmdAt = now;
      if (!d->local) {
        if (Req* r = slotFor(d->ip)) { r->ps = id; d->ps = id; d->on = true; }
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
    Dev* d = selDevLocked();
    if (!d || !d->stateKnown) return;   // blank controls: do not overwrite a device we have not heard from
    if (d->bri == v && d->on) return;
    d->bri = v;
    d->lastCmdAt = now;
    if (d->local) { bri = v; stateUpdated(CALL_MODE_DIRECT_CHANGE); }
    else {
      if (briSendVal >= 0 && briSendIp != d->ip) flushBrightnessLocked();   // never send one device's level to another
      briSendIp = d->ip; briSendVal = v; d->on = true;
      if (int32_t(now - briSendAt) >= 0) briSendAt = now + kBriDebounceMs;   // rate-limit, not debounce
    }
  }

  // Remote brightness goes out at most every kBriDebounceMs while dragging, always with the latest value.
  void flushBrightness(uint32_t now) {
    if (briSendVal < 0 || int32_t(now - briSendAt) < 0) return;
    Lock l(mtx);
    flushBrightnessLocked();
  }
  void flushBrightnessLocked() {
    if (Req* r = slotFor(briSendIp)) {
      r->bri = briSendVal;
      r->briDueAt = 0;
      briSendVal = -1;
    }                                   // else keep it and retry on the next loop
  }

  // --- drawing ------------------------------------------------------------------------------
  struct View {
    char name[40]; uint32_t ip; bool local, listed, on, reachable, stateKnown; uint8_t bri, ps;
    char psName[24]; bool presetsLoaded, presetsFetching; size_t presetCount;
    size_t devCount, devIndex;
  };

  // Hash of every field, one by one (never the struct's bytes: padding is unspecified).
  static uint32_t viewSig(const View& v) {
    ctl::Hasher h;
    h.str(v.name).add(v.ip).add(v.local).add(v.listed).add(v.on).add(v.reachable).add(v.stateKnown).add(v.bri)
     .add(v.ps).str(v.psName).add(v.presetsLoaded).add(v.presetsFetching).add(v.presetCount).add(v.devCount)
     .add(v.devIndex);
    return h.h;
  }

  View snapshot() {
    View v{};
    Lock l(mtx);
    const int si = ctl::findDevice(devs, selIp);
    v.devCount = devs.size();
    if (si < 0) {                       // transient (see selDevLocked): blank controls
      strlcpy(v.name, "-", sizeof(v.name));
      v.reachable = false;
      return v;
    }
    Dev& d = devs[si];
    displayName(d, v.name, sizeof(v.name));
    v.ip = d.ip; v.local = d.local; v.listed = d.listed; v.on = d.on; v.reachable = d.reachable;
    v.stateKnown = d.stateKnown; v.bri = d.bri; v.ps = d.ps;
    v.presetsLoaded = d.presetsLoaded; v.presetsFetching = d.presetsFetching; v.presetCount = d.presets.size();
    v.devIndex = si;
    for (auto& p : d.presets)
      if (p.id == d.ps) snprintf(v.psName, sizeof(v.psName), "%s", p.name);
    if (!v.psName[0] && d.ps) snprintf(v.psName, sizeof(v.psName), "Preset %u", d.ps);
    return v;
  }

  // Draw `s` in the current font at x, truncating with ".." so it fits maxW pixels.
  static void fitText(GFXcanvas16& c, const char* s, int x, int baseline, int maxW, uint16_t color) {
    char buf[48];
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
    const uint32_t base = viewSig(v);
    uint32_t sigs[5];
    sigs[0] = ctl::Hasher().add(wifi).add(v.devCount).add(touchOk).str(statusMsg).str(wifiSsid).str(wifiIp).h;
    for (int r = 0; r < 4; r++) sigs[1 + r] = ctl::Hasher().add(base).add(tRegion == r).h;
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
      if (touchOk) snprintf(line, sizeof(line), "%u device%s", unsigned(v.devCount), v.devCount == 1 ? "" : "s");
      else snprintf(line, sizeof(line), "%s | %u dev", touchNote ? touchNote : "touch not found", unsigned(v.devCount));
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
        if (v.local) canvas->print("this device");
        else {
          char ip[16];
          ipText(v.ip, ip, sizeof(ip));
          snprintf(buf, sizeof(buf), "%s%s", ip, !v.reachable ? " unreachable" : !v.listed ? " not found" : "");
          canvas->print(buf);
        }
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
    const bool fetching = [&] { Lock l(mtx); Dev* d = selDevLocked(); return d && d->presetsFetching; }();
    ctl::Hasher hs;
    hs.add(pickFirst).add(screen).add(fetching);
    for (auto& it : pickItems) hs.str(it.label).str(it.sub).add(it.ip).add(it.id).add(it.current).add(it.header);
    if (hs.h == pickSig) return;
    pickSig = hs.h;

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

  // --- worker: HTTP and mDNS for other WLED devices -------------------------------------------
  static void workerEntry(void* self) { static_cast<ControllerUsermod*>(self)->workerLoop(); }

  Net netSnapshot() { Lock l(mtx); return net; }

  bool takeReq(Req& out) {
    Lock l(mtx);
    for (auto& r : reqs)                // nothing left to do in a slot: free it
      if (r.ip && !r.power && r.ps < 0 && r.bri < 0 && !r.poll && !r.presets) r = Req();
    const int i = ctl::pickRequest(reqs, sizeof(reqs) / sizeof(reqs[0]), millis());
    if (i < 0) return false;
    out = reqs[i];
    reqs[i] = Req();
    return true;
  }

  void report(const char* fmt, uint32_t ip, const char* what) {   // a message for the header (any task)
    Lock l(mtx);
    const int i = ctl::findDevice(devs, ip);
    char name[40];
    if (i >= 0) displayName(devs[i], name, sizeof(name));
    else ipText(ip, name, sizeof(name));
    snprintf(workerMsg, sizeof(workerMsg), fmt, name, what);
  }

  // User requests are checked before every discovery step, and every step is bounded: the mDNS query is
  // asynchronous and a sweep probe takes at most kSweepProbeTotalMs, so a tap waits at most about a second.
  void workerLoop() {
    for (;;) {
      const Net n = netSnapshot();
      mdnsStep(n, millis());           // also collects a query still running after a disconnect
      if (!n.up) { vTaskDelay(pdMS_TO_TICKS(500)); continue; }
      Req r;
      if (takeReq(r)) { doRequest(r); continue; }
      sweepStep(n, millis());
      vTaskDelay(pdMS_TO_TICKS(30));
    }
  }

  // GET or POST `path` on ip:80 and parse the JSON reply through `filter` into `doc`. One deadline covers connect,
  // request, head and body; the body may not exceed maxBytes (a larger Content-Length is rejected unread, a body
  // without one is cut at maxBytes). HTTP/1.0, so the server sends neither chunked encoding nor keep-alive.
  HttpResult httpJson(uint32_t ip, const char* path, const char* body, JsonDocument& doc, const JsonDocument& filter,
                      size_t maxBytes, uint16_t connectMs, uint32_t totalMs) {
    const uint32_t deadline = millis() + totalMs;
    WiFiClient c;
    if (!c.connect(IPAddress(ip), 80, connectMs)) return HTTP_UNREACHABLE;
    char host[16];
    ipText(ip, host, sizeof(host));
    char head[192];
    const size_t bodyLen = body ? strlen(body) : 0;
    int n = snprintf(head, sizeof(head), "%s %s HTTP/1.0\r\nHost: %s\r\nConnection: close\r\n", body ? "POST" : "GET",
                     path, host);
    if (body) n += snprintf(head + n, sizeof(head) - n, "Content-Type: application/json\r\nContent-Length: %u\r\n",
                            unsigned(bodyLen));
    n += snprintf(head + n, sizeof(head) - n, "\r\n");
    if (n <= 0 || size_t(n) >= sizeof(head)) { c.stop(); return HTTP_BAD_REPLY; }
    if (c.write(reinterpret_cast<const uint8_t*>(head), n) != size_t(n) ||
        (bodyLen && c.write(reinterpret_cast<const uint8_t*>(body), bodyLen) != bodyLen)) {
      c.stop();
      return HTTP_UNREACHABLE;
    }
    ctl::HttpHead hh;
    for (bool more = true; more;) {
      if (c.available() > 0) { const int ch = c.read(); if (ch >= 0) more = hh.feed(char(ch)); continue; }
      if (!c.connected()) break;
      if (int32_t(millis() - deadline) >= 0) { c.stop(); return HTTP_TIMEOUT; }
      vTaskDelay(pdMS_TO_TICKS(2));
    }
    HttpResult res = HTTP_OK;
    const long limit = ctl::bodyLimit(hh.contentLength, maxBytes);
    if (hh.state != ctl::HttpHead::DONE || hh.status != 200 || hh.chunked) res = HTTP_BAD_REPLY;
    else if (limit < 0) res = HTTP_TOO_BIG;
    else {
      BodyStream s(c, limit, deadline);
      s.setTimeout(0);
      const DeserializationError err = deserializeJson(doc, s, DeserializationOption::Filter(filter));
      if (err == DeserializationError::NoMemory) res = HTTP_NO_MEMORY;   // doc holds the part that fit
      else if (err) res = s.timedOut() ? HTTP_TIMEOUT : HTTP_BAD_REPLY;
    }
    c.stop();
    return res;
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

  // Count a request's outcome. After ctl::kFailsUntilUnreachable failures in a row the device is unreachable and its
  // state is dropped, so the screen shows "unreachable" instead of the last power and level it heard.
  void recordResult(uint32_t ip, bool ok) {
    Lock l(mtx);
    for (auto& d : devs) {
      if (d.ip != ip || d.local) continue;
      d.fails = ctl::nextFails(d.fails, ok);
      d.reachable = ctl::reachableAfter(d.fails);
      if (!d.reachable) d.stateKnown = false;
      return;
    }
  }

  void doRequest(const Req& r) {
    bool ok = true;
    if (r.power || r.bri >= 0 || r.ps >= 0) {
      StaticJsonDocument<128> body;
      StaticJsonDocument<192> resp, filter;
      filter["on"] = true; filter["bri"] = true; filter["ps"] = true;
      if (r.power == 1) body["on"] = true;
      else if (r.power == 2) body["on"] = false;
      if (r.bri >= 0) body["bri"] = r.bri;
      if (r.ps >= 0) body["ps"] = r.ps;
      body["v"] = true;                 // ask for the resulting state in the response
      char s[128];
      serializeJson(body, s, sizeof(s));
      const HttpResult res = httpJson(r.ip, "/json/state", s, resp, filter, kStateMaxBytes, kConnectMs, kStateTotalMs);
      if (res == HTTP_OK) {
        applyRemoteState(r.ip, resp.as<JsonObject>(), true);
      } else {
        ok = false;
        // The screen already shows the optimistic result; it is not known to be true. Blank the controls until the
        // next poll says what the device is doing, and say that the command did not arrive.
        {
          Lock l(mtx);
          const int i = ctl::findDevice(devs, r.ip);
          if (i > 0) { devs[i].stateKnown = false; devs[i].lastCmdAt = 0; }
        }
        report("%.20s: %s", r.ip, res == HTTP_UNREACHABLE || res == HTTP_TIMEOUT ? "no answer" : "failed");
      }
    }
    if (ok && r.poll) {
      StaticJsonDocument<192> resp, filter;
      filter["on"] = true; filter["bri"] = true; filter["ps"] = true;
      if (httpJson(r.ip, "/json/state", nullptr, resp, filter, kStateMaxBytes, kConnectMs, kStateTotalMs) == HTTP_OK)
        applyRemoteState(r.ip, resp.as<JsonObject>(), false);
      else ok = false;
    }
    if (r.presets) {
      DynamicJsonDocument resp(ctl::kPresetDocBytes);
      StaticJsonDocument<256> filter;
      ctl::presetFilter(filter);
      if (filter.overflowed()) CTL_LOG("[ctl] preset filter overflowed: scenes will be incomplete\n");
      const HttpResult res = resp.capacity()
          ? httpJson(r.ip, "/presets.json", nullptr, resp, filter, kPresetsMaxBytes, kConnectMs, kPresetsTotalMs)
          : HTTP_NO_MEMORY;
      // NoMemory: keep the part that fit, mark the list incomplete, and do not retry on a timer (the next retry
      // would fail the same way at full cost). Opening the list again fetches it again.
      const bool got = res == HTTP_OK || (res == HTTP_NO_MEMORY && resp.capacity());
      std::vector<Preset> list;
      if (got) {
        ctl::presetsFromJson(resp.as<JsonObjectConst>(), list);
        ctl::orderPresets(list);
      }
      {
        Lock l(mtx);
        for (auto& d : devs) if (d.ip == r.ip) {
          d.presetsFetching = false;
          if (got) {
            d.presets.swap(list);
            d.presetsLoaded = true;
            d.presetsPartial = res != HTTP_OK;
            d.presetFails = 0;
          } else if (d.presetFails < 255) {
            d.presetFails++;
          }
        }
      }
      if (res == HTTP_NO_MEMORY) report("%.16s: %s", r.ip, resp.capacity() ? "presets cut short" : "no memory");
      if (res != HTTP_OK && res != HTTP_NO_MEMORY) ok = false;
    }
    recordResult(r.ip, ok);
  }

  // One probe of the unicast sweep, run between requests so control stays responsive. It covers the subnet from
  // the netmask (ctl::sweepRange: wider than /22 is clamped to this device's /24) and skips addresses the node list
  // or mDNS already report. It runs regardless of what they found: on an AP that delivers multicast only some of
  // the time they return a partial list, and nothing says how many devices to expect. It pauses while the screen is
  // dark (nobody is looking at the list) and can be turned off in the config ("lan sweep").
  void sweepStep(const Net& n, uint32_t now) {
    if (!sweepEnabled) {
      if (sweeping || !sweepHits.empty()) {
        sweeping = false;
        Lock l(mtx);
        if (!sweepHits.empty()) { sweepHits.clear(); hitsGen++; }
      }
      return;
    }
    if (!n.awake) return;
    const uint32_t selfH = hostOrder(n.self);
    if (sweeping && selfH != sweepSelf) sweeping = false;   // the address changed: start over on the new subnet
    if (!sweeping) {
      if (sweepDoneAt && !ctl::elapsedMs(now, sweepDoneAt, kSweepIdleMs)) return;
      const ctl::HostRange rg = ctl::sweepRange(selfH, hostOrder(n.mask));
      sweepDoneAt = now ? now : 1;
      if (!rg.count) return;
      sweepNext = rg.first;
      sweepEnd = rg.first + rg.count;
      sweepSelf = selfH;
      sweeping = true;
      Lock l(mtx);
      for (auto& h : sweepHits) h.missed++;
    }
    if (sweepNext >= sweepEnd) {
      sweeping = false;
      sweepDoneAt = now ? now : 1;
      Lock l(mtx);
      const size_t before = sweepHits.size();
      sweepHits.erase(std::remove_if(sweepHits.begin(), sweepHits.end(),
                                     [](const MdnsHit& h) { return h.missed >= 2; }), sweepHits.end());
      if (sweepHits.size() != before) hitsGen++;
      CTL_LOG("[ctl] sweep done: %u WLED found\n", (unsigned)sweepHits.size());
      return;
    }
    const uint32_t ip = rawOrder(sweepNext++);
    if (ip == n.self) return;
    {
      Lock l(mtx);                     // already known from the node list or mDNS: no need to probe
      for (auto& h : mdnsHits) if (h.ip == ip) return;
      for (uint32_t k : nodeIps) if (k == ip) return;
    }
    StaticJsonDocument<64> filter;
    filter["brand"] = true;
    filter["name"] = true;
    StaticJsonDocument<192> doc;
    if (httpJson(ip, "/json/info", nullptr, doc, filter, kInfoMaxBytes, kSweepProbeMs, kSweepProbeTotalMs) != HTTP_OK)
      return;
    if (strcmp(doc["brand"] | "", "WLED") != 0) return;
    MdnsHit hit{};
    hit.ip = ip;
    strlcpy(hit.name, doc["name"] | "", sizeof(hit.name));
    Lock l(mtx);
    for (auto& h : sweepHits) {
      if (h.ip != ip) continue;
      h.missed = 0;
      if (strcmp(h.name, hit.name)) { strlcpy(h.name, hit.name, sizeof(h.name)); hitsGen++; }
      return;
    }
    if (sweepHits.size() < kMaxHits) {
      sweepHits.push_back(hit);
      hitsGen++;
      CTL_LOG("[ctl] sweep found %s\n", hit.name);
    } else {
      CTL_LOG("[ctl] sweep: more than %u WLED devices, %s not listed\n", (unsigned)kMaxHits, hit.name);
    }
  }

  // mDNS finds WLED devices that have the node list broadcast turned off. The query is asynchronous: started here,
  // collected on a later call, so the worker never blocks for its 3 s (MDNS.queryService did, every 30 s).
  // On reconnect WLED calls MDNS.end(), which frees every query still RUNNING (a finished one stays ours to delete).
  // So a query is started only once WLED's responder is up for this connection, and after a disconnect the worker
  // waits here for the running query to finish (it has nothing else to do offline) before WLED can reconnect.
  void mdnsStep(const Net& n, uint32_t now) {
    if (mdnsSearch) {
      mdns_result_t* results = nullptr;
      uint8_t count = 0;
      const uint32_t wait = n.mdnsReady ? 0 : kMdnsQueryMs + 500;
      if (!mdns_query_async_get_results(mdnsSearch, wait, &results, &count)) return;   // still running
      mdns_query_async_delete(mdnsSearch);
      mdnsSearch = nullptr;
      if (n.mdnsReady) mergeMdns(results, n);
      if (results) mdns_query_results_free(results);
      return;
    }
    if (!n.mdnsReady) return;
    if (!mdnsKick.exchange(false) && lastMdns && !ctl::elapsedMs(now, lastMdns, kMdnsMs)) return;
    lastMdns = now ? now : 1;
    // null when mDNS is not running (WLED starts it only with an mDNS name set): tried again in kMdnsMs.
    mdnsSearch = mdns_query_async_new(nullptr, "_wled", "_tcp", MDNS_TYPE_PTR, kMdnsQueryMs, kMaxHits, nullptr);
  }

  void mergeMdns(mdns_result_t* results, const Net& n) {
    std::vector<MdnsHit> seen;
    size_t answers = 0, offSubnet = 0;
    for (mdns_result_t* r = results; r; r = r->next) {
      answers++;
      uint32_t ip = 0;
      for (mdns_ip_addr_t* a = r->addr; a && !ip; a = a->next)
        if (a->addr.type == ESP_IPADDR_TYPE_V4) ip = a->addr.u_addr.ip4.addr;
      if (!ip) continue;
      if (!ctl::inSubnet(ip, n.self, n.mask)) { offSubnet++; continue; }   // anyone can answer mDNS
      bool dup = false;
      for (auto& s : seen) dup |= s.ip == ip;
      if (dup) continue;
      MdnsHit h{};
      h.ip = ip;
      strlcpy(h.name, r->hostname ? r->hostname : (r->instance_name ? r->instance_name : ""), sizeof(h.name));
      seen.push_back(h);
    }
    Lock l(mtx);
    // One mDNS query is lossy (multicast has no retries, and a device at the edge of WiFi range misses some).
    // Replacing the whole list each scan made such devices flicker out; instead merge, and drop a device only
    // after it has been missed kMdnsMaxMissed scans in a row.
    bool changed = false;
    for (auto& old : mdnsHits) old.missed++;
    for (auto& h : seen) {
      bool known = false;
      for (auto& old : mdnsHits) {
        if (old.ip != h.ip) continue;
        old.missed = 0;
        if (strcmp(old.name, h.name)) { strlcpy(old.name, h.name, sizeof(old.name)); changed = true; }
        known = true;
        break;
      }
      if (known) continue;
      if (mdnsHits.size() < kMaxHits) { mdnsHits.push_back(h); changed = true; }
      else CTL_LOG("[ctl] mDNS: more than %u WLED devices, %s not listed\n", (unsigned)kMaxHits, h.name);
    }
    const size_t before = mdnsHits.size();
    mdnsHits.erase(std::remove_if(mdnsHits.begin(), mdnsHits.end(),
                                  [](const MdnsHit& h) { return h.missed >= kMdnsMaxMissed; }),
                   mdnsHits.end());
    if (changed || mdnsHits.size() != before) hitsGen++;
    CTL_LOG("[ctl] mDNS: answers=%u off-subnet=%u seen=%u kept=%u nodes=%u sweep=%u\n", (unsigned)answers,
            (unsigned)offSubnet, (unsigned)seen.size(), (unsigned)mdnsHits.size(), (unsigned)nodeCount,
            (unsigned)sweepHits.size());
  }
};

static ControllerUsermod controllerUsermod;
REGISTER_USERMOD(controllerUsermod);
