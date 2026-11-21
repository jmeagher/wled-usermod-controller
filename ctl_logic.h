// Pure logic of the controller usermod: no WLED, Arduino or hardware dependencies, so tests/test_logic.cpp can
// compile and run it on a desktop. controller.cpp uses these functions; keep behaviour changes here, with a test.
#pragma once
#include <algorithm>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <type_traits>
#include <vector>

namespace ctl {

// WLED brightness (0..255) as a whole percentage, rounded.
inline int percentOf(uint8_t bri) { return (bri * 100 + 127) / 255; }

// Brightness for a touch at `x` on a slider track starting at trackX and trackW pixels wide.
// Never 0 (the Power control is the off switch) and clamped at both ends of the track.
inline uint8_t briFromTrackX(int x, int trackX, int trackW) {
  return uint8_t(std::min(255, std::max(1, 1 + (x - trackX) * 254 / trackW)));
}

// Pixel width of the filled part of the bar for a brightness: proportional, at least 1 px. There must be no
// minimum "so the rounded end fits": it made every level below ~33% look like a third full.
inline int fillWidth(uint8_t bri, int trackW) { return std::max(1, trackW * bri / 255); }

// Corner radius for the fill. A fixed radius needs a fill at least twice as wide, so it shrinks for narrow fills.
inline int fillRadius(int fillW, int maxRadius = 12) { return std::min(maxRadius, fillW / 2); }

// One decoded report of the AXS5106L touch controller.
struct TouchPoint {
  uint8_t n = 0;   // number of fingers; 0 = nothing touching (a release report, or no data)
  int x = 0;       // panel coordinates, portrait, clamped to the panel
  int y = 0;
};

// Decode the 14 bytes read from register 0x01. Byte 1 is the finger count; for the first finger the X and Y values
// are 12 bits each (high nibble of bytes 2 / 4, low byte in 3 / 5; the high bits of byte 2 and 4 carry an event flag).
// X is mirrored for rotation 0, exactly as in Waveshare's driver (bsp_touch_get_coordinates).
inline TouchPoint decodeTouch(const uint8_t* d, int width, int height) {
  TouchPoint p;
  if (d[1] < 1 || d[1] > 5) return p;
  p.n = d[1];
  p.x = std::min(width - 1, std::max(0, width - 1 - (((d[2] & 0x0F) << 8) | d[3])));
  p.y = std::min(height - 1, std::max(0, ((d[4] & 0x0F) << 8) | d[5]));
  return p;
}

// Snap-scrolling for the lists: dragging dy pixels moves the first visible item by whole itemH steps, the remainder
// carries in `accum`. Dragging up shows later items. `first` stays within 0..maxFirst.
inline void scrollList(int& first, int& accum, int dy, int itemH, int maxFirst) {
  accum += dy;
  while (accum <= -itemH) { first++; accum += itemH; }
  while (accum >= itemH)  { first--; accum -= itemH; }
  first = std::min(maxFirst, std::max(0, first));
}

// A preset is a "scene" (a complete look to pick, listed first) if WLED saved it as a playlist, or it carries a
// non-empty quick-load label ("ql" in presets.json). WLED has no scene flag, and a plain preset like a device's boot
// look is still a scene, so the label is the opt-in; helper presets that only feed a playlist leave it unset.
inline bool isScene(bool hasPlaylist, const char* quickLoad) {
  return hasPlaylist || (quickLoad && *quickLoad);
}

// Order a list of presets so scenes come first (by id), then the other presets (by id).
// P needs `uint8_t id` and `bool scene`. Returns the number of scenes, which are the front group.
template <class P> size_t orderPresets(std::vector<P>& list) {
  std::sort(list.begin(), list.end(), [](const P& a, const P& b) {
    if (a.scene != b.scene) return a.scene > b.scene;
    return a.id < b.id;
  });
  return size_t(std::count_if(list.begin(), list.end(), [](const P& p) { return p.scene; }));
}

// True when more than `ms` milliseconds have passed since `since`, for millis() values that wrap at 2^32.
// Signed on purpose: `since` may be a few ms NEWER than `now` (a callback stamped millis() after the loop captured
// `now`). Unsigned `now - since` then wraps to ~4 billion and reads as "idle for 49 days": the screen woke by a touch
// went straight back to sleep in the same millisecond.
inline bool elapsedMs(uint32_t now, uint32_t since, uint32_t ms) { return int32_t(now - since) > int32_t(ms); }

// --- redraw signatures -----------------------------------------------------------------------------------------
// FNV-1a over explicitly listed fields. Hashing a whole struct also hashes its padding bytes, whose values C++ leaves
// unspecified (copies need not preserve them), so a signature could change with no visible change, or the reverse.
inline uint32_t fnv1a(const void* p, size_t n, uint32_t h = 2166136261u) {
  const uint8_t* b = static_cast<const uint8_t*>(p);
  while (n--) h = (h ^ *b++) * 16777619u;
  return h;
}

struct Hasher {
  uint32_t h = 2166136261u;
  template <class T> Hasher& add(T v) {   // arithmetic and enum values only: they have no padding
    static_assert(std::is_arithmetic<T>::value || std::is_enum<T>::value, "hash fields one by one");
    h = fnv1a(&v, sizeof(v), h);
    return *this;
  }
  Hasher& str(const char* s) {            // the characters and the terminator, never the bytes after it
    h = fnv1a(s, strlen(s) + 1, h);
    return *this;
  }
};

// --- network ---------------------------------------------------------------------------------------------------
// True when `ip` is another host on this device's subnet: same network part, not this device, not the network or
// broadcast address. Works on IPv4 addresses in any byte order, as long as all three use the same one. Discovered
// devices outside it are ignored: an mDNS answer (anyone on the LAN can send one) may point anywhere.
inline bool inSubnet(uint32_t ip, uint32_t self, uint32_t mask) {
  if (!ip || !mask || ip == self) return false;
  if ((ip & mask) != (self & mask)) return false;
  const uint32_t host = ip & ~mask;
  return host != 0 && host != (~mask);
}

// Range of addresses the unicast sweep probes, in HOST byte order (a.b.c.d = a << 24 | ...): every host address of
// the subnet, except that a subnet wider than /kSweepMinPrefix is clamped to the /24 around this device (a /16 is
// 65534 probes, hours per pass). A mask that is not a contiguous prefix is treated as /24.
constexpr int kSweepMinPrefix = 22;   // at most 1022 probes per pass
struct HostRange { uint32_t first = 0; uint32_t count = 0; };
inline int prefixLength(uint32_t maskHost) {
  const uint32_t inv = ~maskHost;
  if (inv & (inv + 1)) return -1;      // not of the form 1..10..0
  int n = 0;
  while (n < 32 && (maskHost & (0x80000000u >> n))) n++;
  return n;
}
inline HostRange sweepRange(uint32_t selfHost, uint32_t maskHost) {
  const int len = prefixLength(maskHost);
  uint32_t m = maskHost;
  if (len < kSweepMinPrefix) m = 0xFFFFFF00u;   // also non-contiguous (-1)
  const uint32_t hostBits = ~m;
  HostRange r;
  if (hostBits < 3) return r;                    // /31 and /32: no other host to find
  r.first = (selfHost & m) + 1;                  // skip the network address ...
  r.count = hostBits - 1;                        // ... and the broadcast address
  return r;
}

// What to do about a global WLED bus (I2C or SPI, set under Config > Usermods / hardware) that this usermod also
// needs. The Arduino core starts each bus once: a second begin() with other pins is silently ignored (SPIClass
// returns early, TwoWire logs "Bus already started"), so the display or touch would talk on WLED's pins.
enum BusUse : uint8_t { BUS_OWN, BUS_SHARED, BUS_CONFLICT };
inline BusUse busUse(int globalA, int globalB, int ourA, int ourB) {
  if (globalA < 0 && globalB < 0) return BUS_OWN;          // WLED did not start the bus: we do
  if (globalA == ourA && globalB == ourB) return BUS_SHARED;  // started on our pins already: just use it
  return BUS_CONFLICT;
}

// --- device list -----------------------------------------------------------------------------------------------
// One discovered device before it is merged into the list (from WLED's node list, mDNS or the sweep).
struct DevCand { uint32_t ip = 0; char name[33] = {}; bool hasOn = false; bool on = false; };

// Rebuild the device list from discovery results. devs[0] is this device and stays first; the others are sorted by
// name, then address, and keep their state (power, presets...) across rebuilds. Candidates outside the subnet,
// duplicates and this device are skipped. `keepIp` (the selected device) stays in the list even when no source
// reports it any more, with listed = false: dropping it used to fall the selection back to THIS device, so the next
// Power tap switched the wrong lights. Devices with the same name get dupName set (the UI adds the address).
// D needs: ip, local, name[33], on, stateKnown, listed, dupName; default-constructible and movable.
template <class D>
void mergeDevices(std::vector<D>& devs, const std::vector<DevCand>& cands, uint32_t keepIp, uint32_t self,
                  uint32_t mask) {
  if (devs.empty()) return;            // caller always has devs[0]
  std::vector<D> next;
  next.reserve(cands.size() + 2);
  next.push_back(std::move(devs[0]));
  auto takeOld = [&](uint32_t ip, D& out) {
    for (size_t i = 1; i < devs.size(); i++)
      if (devs[i].ip == ip) { out = std::move(devs[i]); devs[i].ip = 0; return true; }
    return false;
  };
  auto present = [&](uint32_t ip) {
    for (size_t i = 1; i < next.size(); i++) if (next[i].ip == ip) return true;
    return false;
  };
  for (const DevCand& c : cands) {
    if (!inSubnet(c.ip, self, mask) || present(c.ip)) continue;
    D d;
    takeOld(c.ip, d);
    d.ip = c.ip;
    d.local = false;
    snprintf(d.name, sizeof(d.name), "%s", c.name);
    d.listed = true;
    if (!d.stateKnown && c.hasOn) d.on = c.on;   // best guess until the first poll
    next.push_back(std::move(d));
  }
  if (keepIp && !present(keepIp)) {
    D d;
    if (takeOld(keepIp, d)) { d.listed = false; next.push_back(std::move(d)); }
  }
  std::sort(next.begin() + 1, next.end(), [](const D& a, const D& b) {
    const int c = strcasecmp(a.name, b.name);
    return c ? c < 0 : a.ip < b.ip;
  });
  for (size_t i = 1; i < next.size(); i++) {
    next[i].dupName = (i > 1 && !strcasecmp(next[i - 1].name, next[i].name)) ||
                      (i + 1 < next.size() && !strcasecmp(next[i + 1].name, next[i].name));
  }
  devs.swap(next);
}

// Index of the device with address ip, or -1.
template <class D> int findDevice(const std::vector<D>& devs, uint32_t ip) {
  for (size_t i = 0; i < devs.size(); i++) if (devs[i].ip == ip) return int(i);
  return -1;
}

// Consecutive failed requests before a remote device counts as unreachable and its shown state is dropped. One lost
// reply on weak WiFi should not blank the controls; a device that is gone must not keep showing stale power/level.
constexpr uint8_t kFailsUntilUnreachable = 2;
inline uint8_t nextFails(uint8_t fails, bool ok) { return ok ? 0 : uint8_t(fails < 255 ? fails + 1 : 255); }
inline bool reachableAfter(uint8_t fails) { return fails < kFailsUntilUnreachable; }

// Delay before fetching a device's preset list again after `fails` failed attempts in a row: 10 s doubling to 5 min,
// so a device whose list cannot be read does not cost a full download every 10 s forever.
inline uint32_t presetRetryMs(uint8_t fails) {
  uint32_t ms = 10000;
  for (uint8_t i = 1; i < fails && ms < 300000; i++) ms *= 2;
  return std::min<uint32_t>(ms, 300000);
}

// --- worker queue ----------------------------------------------------------------------------------------------
// Which pending request the worker runs next: -1 for none. User commands (power, preset, a due brightness) come
// before background work (state polls, preset lists), so a tap does not wait behind them.
// R needs: ip (0 = free slot), power, ps (-1 none), bri (-1 none), briDueAt, poll, presets.
template <class R> int pickRequest(const R* reqs, size_t n, uint32_t now) {
  int background = -1;
  for (size_t i = 0; i < n; i++) {
    const R& r = reqs[i];
    if (!r.ip) continue;
    const bool briDue = r.bri >= 0 && int32_t(now - r.briDueAt) >= 0;
    if (r.power || r.ps >= 0 || briDue) return int(i);
    if ((r.poll || r.presets) && background < 0) background = int(i);
  }
  return background;
}

// --- HTTP ------------------------------------------------------------------------------------------------------
// Incremental parser for an HTTP/1.x response head (status line + headers), fed one byte at a time. The worker reads
// the head itself so the whole exchange runs under one deadline and byte budget: HTTPClient's header loop restarts
// its timeout on every byte, so a host that drips data held the worker (and every user command behind it) forever.
struct HttpHead {
  enum State : uint8_t { STATUS, HEADERS, DONE, BAD };
  static constexpr size_t kMaxHead = 4096;     // WLED's heads are a few hundred bytes
  State state = STATUS;
  int status = 0;
  long contentLength = -1;                     // -1: not given (HTTP/1.0 body runs until the connection closes)
  bool chunked = false;
  size_t total = 0;
  char line[128] = {};                         // longer lines are truncated; only their start is ever looked at
  size_t len = 0;

  // Returns true while more head bytes are wanted (state STATUS or HEADERS).
  bool feed(char c) {
    if (state >= DONE) return false;
    if (++total > kMaxHead) { state = BAD; return false; }
    if (c == '\n') {
      if (len && line[len - 1] == '\r') len--;
      line[len] = 0;
      endLine();
      len = 0;
      return state < DONE;
    }
    if (len < sizeof(line) - 1) line[len++] = c;
    return true;
  }

 private:
  static bool startsWithCI(const char* s, const char* prefix) { return !strncasecmp(s, prefix, strlen(prefix)); }
  void endLine() {
    if (state == STATUS) {                     // "HTTP/1.1 200 OK"
      if (len < 12 || strncmp(line, "HTTP/1.", 7) || line[8] != ' ') { state = BAD; return; }
      char* end = nullptr;
      const long code = strtol(line + 9, &end, 10);
      if (end != line + 12 || code < 100 || code > 999) { state = BAD; return; }
      status = int(code);
      state = HEADERS;
      return;
    }
    if (!len) { state = DONE; return; }      // the blank line ends the head
    if (startsWithCI(line, "content-length:")) {
      const char* v = line + 15;
      while (*v == ' ' || *v == '\t') v++;
      char* end = nullptr;
      const long n = strtol(v, &end, 10);
      while (end && (*end == ' ' || *end == '\t')) end++;
      if (end == v || !end || *end || n < 0 || n > 100000000L) { state = BAD; return; }
      contentLength = n;
    } else if (startsWithCI(line, "transfer-encoding:")) {
      for (const char* p = line + 18; *p; p++) if (startsWithCI(p, "chunked")) chunked = true;
    }
  }
};

// Byte limit for a response body: the Content-Length when it fits maxBytes, maxBytes when the server sent none (the
// body ends at connection close, and a longer one is cut there and fails to parse), or -1 to reject it unread.
inline long bodyLimit(long contentLength, size_t maxBytes) {
  if (contentLength < 0) return long(maxBytes);
  return size_t(contentLength) <= maxBytes ? contentLength : -1;
}

// --- touch reset -----------------------------------------------------------------------------------------------
// Non-blocking reset pulse for the touch chip (Waveshare's driver: RST low 200 ms, then high and 300 ms to boot).
// It used to be two delay()s inside loop(), freezing WLED for half a second up to every 10 s while touch was dead.
struct ResetPulse {
  enum Phase : uint8_t { IDLE, HOLD_LOW, BOOTING };
  enum Action : uint8_t { NONE, DRIVE_HIGH, FINISHED };
  static constexpr uint32_t kLowMs = 200, kBootMs = 300;
  Phase phase = IDLE;
  uint32_t since = 0;
  bool busy() const { return phase != IDLE; }
  void start(uint32_t now) { phase = HOLD_LOW; since = now; }   // the caller drives RST low
  Action step(uint32_t now) {
    if (phase == HOLD_LOW && int32_t(now - since) >= int32_t(kLowMs)) { phase = BOOTING; since = now; return DRIVE_HIGH; }
    if (phase == BOOTING && int32_t(now - since) >= int32_t(kBootMs)) { phase = IDLE; return FINISHED; }
    return NONE;
  }
};

}  // namespace ctl
