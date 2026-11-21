// Desktop tests for ctl_logic.h and the JD9853 init table. Build and run with tests/run.sh.
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "../ctl_logic.h"
#include "../jd9853_init.h"

static int failures = 0;
#define CHECK(cond) do { if (!(cond)) { std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); failures++; } } while (0)
#define CHECK_EQ(a, b) do { long _a = (a), _b = (b); if (_a != _b) { \
  std::printf("FAIL %s:%d  %s == %s  (%ld vs %ld)\n", __FILE__, __LINE__, #a, #b, _a, _b); failures++; } } while (0)

constexpr int kW = 172, kH = 320, kTrackX = 10, kTrackW = kW - 20;   // same geometry as controller.cpp

// --- brightness bar ---------------------------------------------------------------------------------------
static void testPercent() {
  CHECK_EQ(ctl::percentOf(0), 0);
  CHECK_EQ(ctl::percentOf(255), 100);
  CHECK_EQ(ctl::percentOf(6), 2);      // the "bar" device: bri 6 is 2%
  CHECK_EQ(ctl::percentOf(128), 50);
  int prev = -1;
  for (int b = 0; b <= 255; b++) { int p = ctl::percentOf(uint8_t(b)); CHECK(p >= prev && p <= 100); prev = p; }
}

static void testFillIsProportional() {
  // Regression: a floor of the bar height (50 px of a 152 px track) drew 2% as a third full.
  CHECK(ctl::fillWidth(6, kTrackW) <= 4);
  CHECK_EQ(ctl::fillWidth(255, kTrackW), kTrackW);
  CHECK_EQ(ctl::fillWidth(0, kTrackW), 1);          // never zero width
  CHECK(ctl::fillWidth(128, kTrackW) >= kTrackW / 2 - 1 && ctl::fillWidth(128, kTrackW) <= kTrackW / 2 + 1);
  int prev = 0;
  for (int b = 0; b <= 255; b++) { int w = ctl::fillWidth(uint8_t(b), kTrackW); CHECK(w >= prev && w <= kTrackW); prev = w; }
  // fill as a fraction of the track must track the brightness fraction to within one pixel
  for (int b = 1; b <= 255; b++) {
    int w = ctl::fillWidth(uint8_t(b), kTrackW);
    int exact = kTrackW * b / 255;
    CHECK(w == exact || (exact == 0 && w == 1));
  }
}

static void testFillRadius() {
  CHECK_EQ(ctl::fillRadius(3), 1);
  CHECK_EQ(ctl::fillRadius(1), 0);
  CHECK_EQ(ctl::fillRadius(24), 12);
  CHECK_EQ(ctl::fillRadius(152), 12);
  for (int w = 1; w <= 152; w++) CHECK(2 * ctl::fillRadius(w) <= w);   // the corners always fit in the fill
}

static void testSliderMapping() {
  CHECK_EQ(ctl::briFromTrackX(kTrackX, kTrackX, kTrackW), 1);                    // left end
  CHECK_EQ(ctl::briFromTrackX(kTrackX + kTrackW, kTrackX, kTrackW), 255);        // right end
  CHECK_EQ(ctl::briFromTrackX(0, kTrackX, kTrackW), 1);                          // left of the track clamps
  CHECK_EQ(ctl::briFromTrackX(kW - 1, kTrackX, kTrackW), 255);                   // right of the track clamps
  int prev = 0;
  for (int x = 0; x < kW; x++) { int b = ctl::briFromTrackX(x, kTrackX, kTrackW); CHECK(b >= 1 && b <= 255 && b >= prev); prev = b; }
  // dragging to x must draw a bar that ends within 2 px of the finger
  for (int x = kTrackX + 2; x <= kTrackX + kTrackW; x++) {
    int w = ctl::fillWidth(ctl::briFromTrackX(x, kTrackX, kTrackW), kTrackW);
    CHECK(w >= (x - kTrackX) - 2 && w <= (x - kTrackX) + 2);
  }
}

// --- touch decoding: real reports captured from the board (serial log), with the coordinates firmware logged ---
static ctl::TouchPoint decode(std::initializer_list<uint8_t> first6) {
  uint8_t d[14] = {};
  int i = 0;
  for (uint8_t b : first6) d[i++] = b;
  return ctl::decodeTouch(d, kW, kH);
}

static void testTouchDecodeRealReports() {
  struct { uint8_t raw[6]; int x, y; } cases[] = {
    {{0x00, 0x01, 0x00, 0x6e, 0x00, 0x3a}, 61, 58},      // "touch down x=61 y=58"
    {{0x00, 0x01, 0x00, 0x74, 0x00, 0x77}, 55, 119},     // "touch down x=55 y=119"
    {{0x00, 0x01, 0x80, 0x87, 0x01, 0x39}, 36, 313},     // event flag 0x80 in byte 2 must not leak into X
    {{0x00, 0x01, 0x80, 0x22, 0x01, 0x39}, 137, 313},
    {{0x00, 0x01, 0x80, 0x07, 0x01, 0x39}, 164, 313},
  };
  for (auto& c : cases) {
    ctl::TouchPoint p = decode({c.raw[0], c.raw[1], c.raw[2], c.raw[3], c.raw[4], c.raw[5]});
    CHECK_EQ(p.n, 1);
    CHECK_EQ(p.x, c.x);
    CHECK_EQ(p.y, c.y);
  }
}

static void testTouchRelease() {
  CHECK_EQ(decode({0x00, 0x00, 0x40, 0x6e, 0x00, 0x3a}).n, 0);   // release report from the log
  CHECK_EQ(decode({0x00, 0x06, 0, 0, 0, 0}).n, 0);               // more fingers than the chip supports: ignore
  CHECK_EQ(decode({0, 0, 0, 0, 0, 0}).n, 0);
}

static void testTouchClamps() {
  ctl::TouchPoint p = decode({0x00, 0x01, 0x0f, 0xff, 0x0f, 0xff});   // 4095, 4095: far outside the panel
  CHECK_EQ(p.x, 0);                                                   // mirrored X clamps to the left edge
  CHECK_EQ(p.y, kH - 1);
  p = decode({0x00, 0x01, 0x00, 0x00, 0x00, 0x00});                   // raw 0,0
  CHECK_EQ(p.x, kW - 1);
  CHECK_EQ(p.y, 0);
}

// --- list scrolling ---------------------------------------------------------------------------------------
static void testScroll() {
  int first = 0, acc = 0;
  ctl::scrollList(first, acc, -44, 44, 10);          // drag up one item
  CHECK_EQ(first, 1); CHECK_EQ(acc, 0);
  ctl::scrollList(first, acc, 44, 44, 10);           // drag back down
  CHECK_EQ(first, 0);
  first = 0; acc = 0;
  for (int i = 0; i < 9; i++) ctl::scrollList(first, acc, -5, 44, 10);   // 45 px in small pieces = one item
  CHECK_EQ(first, 1);
  first = 3; acc = 0;
  ctl::scrollList(first, acc, 1000, 44, 10);         // far past the top clamps to 0
  CHECK_EQ(first, 0);
  ctl::scrollList(first, acc, -100000, 44, 10);      // far past the end clamps to maxFirst
  CHECK_EQ(first, 10);
  first = 0; acc = 0;
  ctl::scrollList(first, acc, -500, 44, 0);          // a list that fits on screen never scrolls
  CHECK_EQ(first, 0);
}

// --- idle timeout --------------------------------------------------------------------------------------------
static void testElapsed() {
  // Regression (seen in a serial capture: "screen wake" and "screen sleep" in the same millisecond): the activity
  // stamp can be NEWER than the loop's `now`. That must read as "not idle", not as 49 days of idleness.
  CHECK(!ctl::elapsedMs(1000, 1001, 60000));
  CHECK(!ctl::elapsedMs(1000, 1005, 60000));
  CHECK(!ctl::elapsedMs(1000, 1000, 0));
  CHECK(ctl::elapsedMs(2000, 1000, 500));
  CHECK(!ctl::elapsedMs(2000, 1000, 1000));        // strictly more than the timeout
  CHECK(ctl::elapsedMs(2001, 1000, 1000));
  // millis() wraps at 2^32 (after 49 days): 356 ms have passed between these two values
  CHECK(ctl::elapsedMs(100, 0xFFFFFF00u, 300));
  CHECK(!ctl::elapsedMs(100, 0xFFFFFF00u, 400));
  // the longest configurable timeout (uint16 seconds) still fits the signed comparison
  const uint32_t longest = 65535u * 1000u;
  CHECK(!ctl::elapsedMs(5000000, 5000000 - 1000, longest));
  CHECK(ctl::elapsedMs(5000000, 5000000 - longest - 1, longest));
}

// --- redraw signatures ------------------------------------------------------------------------------------
static void testHasher() {
  CHECK_EQ(ctl::fnv1a("", 0), 2166136261u);
  CHECK_EQ(ctl::fnv1a("a", 1), 0xe40c292cu);                 // published FNV-1a test vector
  // Strings hash by content: what follows the terminator in the buffer does not matter.
  char a[16], b[16];
  memset(a, 'x', sizeof(a)); memset(b, 'y', sizeof(b));
  strcpy(a, "Kitchen"); strcpy(b, "Kitchen");
  CHECK_EQ(ctl::Hasher().str(a).h, ctl::Hasher().str(b).h);
  // Field boundaries count: "ab"+"c" differs from "a"+"bc".
  CHECK(ctl::Hasher().str("ab").str("c").h != ctl::Hasher().str("a").str("bc").h);
  CHECK(ctl::Hasher().add(true).add(uint8_t(5)).h != ctl::Hasher().add(false).add(uint8_t(5)).h);
  CHECK(ctl::Hasher().add(uint32_t(1)).h != ctl::Hasher().add(uint32_t(2)).h);
}

// --- network ----------------------------------------------------------------------------------------------
static uint32_t ip4(int a, int b, int c, int d) { return uint32_t(a) << 24 | uint32_t(b) << 16 | uint32_t(c) << 8 | uint32_t(d); }
// The device code passes IPAddress's raw value (first octet in the lowest byte); inSubnet must not care.
static uint32_t raw4(int a, int b, int c, int d) { return uint32_t(d) << 24 | uint32_t(c) << 16 | uint32_t(b) << 8 | uint32_t(a); }

static void testInSubnet() {
  for (int order = 0; order < 2; order++) {
    auto A = order ? raw4 : ip4;
    const uint32_t self = A(192, 168, 1, 50), m24 = A(255, 255, 255, 0);
    CHECK(ctl::inSubnet(A(192, 168, 1, 7), self, m24));
    CHECK(!ctl::inSubnet(A(192, 168, 2, 7), self, m24));       // other subnet
    CHECK(!ctl::inSubnet(A(8, 8, 8, 8), self, m24));           // off-LAN mDNS answer
    CHECK(!ctl::inSubnet(self, self, m24));                    // this device
    CHECK(!ctl::inSubnet(A(192, 168, 1, 0), self, m24));       // network address
    CHECK(!ctl::inSubnet(A(192, 168, 1, 255), self, m24));     // broadcast
    CHECK(!ctl::inSubnet(0, self, m24));
    CHECK(!ctl::inSubnet(A(192, 168, 1, 7), self, 0));         // no mask: trust nothing
    const uint32_t m22 = A(255, 255, 252, 0);
    CHECK(ctl::inSubnet(A(192, 168, 3, 254), self, m22));      // a /22 spans .0.x to .3.x
    CHECK(!ctl::inSubnet(A(192, 168, 3, 255), self, m22));     // its broadcast address
    CHECK(ctl::inSubnet(A(192, 168, 0, 255), self, m22));
    CHECK(!ctl::inSubnet(A(192, 168, 4, 1), self, m22));
  }
}

static void testSweepRange() {
  ctl::HostRange r = ctl::sweepRange(ip4(192, 168, 1, 50), ip4(255, 255, 255, 0));
  CHECK_EQ(r.first, ip4(192, 168, 1, 1)); CHECK_EQ(r.count, 254);
  r = ctl::sweepRange(ip4(10, 0, 5, 9), ip4(255, 255, 252, 0));            // /22: whole subnet
  CHECK_EQ(r.first, ip4(10, 0, 4, 1)); CHECK_EQ(r.count, 1022);
  r = ctl::sweepRange(ip4(10, 0, 5, 9), ip4(255, 255, 0, 0));              // /16: clamped to our /24
  CHECK_EQ(r.first, ip4(10, 0, 5, 1)); CHECK_EQ(r.count, 254);
  r = ctl::sweepRange(ip4(10, 1, 2, 3), 0);                                // no mask
  CHECK_EQ(r.first, ip4(10, 1, 2, 1)); CHECK_EQ(r.count, 254);
  r = ctl::sweepRange(ip4(10, 1, 2, 3), ip4(255, 0, 255, 0));              // not a prefix
  CHECK_EQ(r.count, 254);
  r = ctl::sweepRange(ip4(192, 168, 1, 130), ip4(255, 255, 255, 128));     // /25: only our half
  CHECK_EQ(r.first, ip4(192, 168, 1, 129)); CHECK_EQ(r.count, 126);
  r = ctl::sweepRange(ip4(192, 168, 1, 6), ip4(255, 255, 255, 252));       // /30
  CHECK_EQ(r.first, ip4(192, 168, 1, 5)); CHECK_EQ(r.count, 2);
  CHECK_EQ(ctl::sweepRange(ip4(192, 168, 1, 6), ip4(255, 255, 255, 254)).count, 0);   // /31
  CHECK_EQ(ctl::sweepRange(ip4(192, 168, 1, 6), 0xFFFFFFFFu).count, 0);              // /32
  CHECK_EQ(ctl::prefixLength(ip4(255, 255, 255, 0)), 24);
  CHECK_EQ(ctl::prefixLength(0), 0);
  CHECK_EQ(ctl::prefixLength(0xFFFFFFFFu), 32);
  CHECK_EQ(ctl::prefixLength(ip4(255, 0, 255, 0)), -1);
}

static void testBusUse() {
  CHECK_EQ(ctl::busUse(-1, -1, 18, 19), ctl::BUS_OWN);
  CHECK_EQ(ctl::busUse(18, 19, 18, 19), ctl::BUS_SHARED);
  CHECK_EQ(ctl::busUse(21, 22, 18, 19), ctl::BUS_CONFLICT);
  CHECK_EQ(ctl::busUse(18, -1, 18, 19), ctl::BUS_CONFLICT);
}

// --- device list ------------------------------------------------------------------------------------------
struct TDev {
  uint32_t ip = 0; bool local = false; char name[33] = {}; bool on = false; bool stateKnown = false;
  bool listed = true; bool dupName = false; int tag = 0;   // tag: stands in for state that must survive a rebuild
};

static ctl::DevCand cand(uint32_t ip, const char* name, bool hasOn = false, bool on = false) {
  ctl::DevCand c; c.ip = ip; snprintf(c.name, sizeof(c.name), "%s", name); c.hasOn = hasOn; c.on = on; return c;
}

static std::vector<TDev> localOnly() {
  std::vector<TDev> d(1);
  d[0].local = true; strcpy(d[0].name, "self"); d[0].stateKnown = true;
  return d;
}

static void testMergeSortsAndFilters() {
  const uint32_t self = ip4(192, 168, 1, 50), mask = ip4(255, 255, 255, 0);
  std::vector<TDev> devs = localOnly();
  std::vector<ctl::DevCand> c = {cand(ip4(192, 168, 1, 9), "zeta", true, true), cand(ip4(192, 168, 1, 8), "Alpha"),
                                 cand(ip4(192, 168, 1, 9), "zeta again"),   // same address from another source
                                 cand(self, "me"), cand(ip4(10, 0, 0, 1), "elsewhere")};
  ctl::mergeDevices(devs, c, 0, self, mask);
  CHECK_EQ(devs.size(), 3);
  CHECK(devs[0].local);
  CHECK(!strcmp(devs[1].name, "Alpha"));
  CHECK(!strcmp(devs[2].name, "zeta"));
  CHECK(devs[2].on);                   // node list hint until the first poll
  CHECK(!devs[1].dupName && !devs[2].dupName);
}

static void testMergeKeepsState() {
  const uint32_t self = ip4(192, 168, 1, 50), mask = ip4(255, 255, 255, 0);
  std::vector<TDev> devs = localOnly();
  ctl::mergeDevices(devs, {cand(ip4(192, 168, 1, 9), "a")}, 0, self, mask);
  devs[1].tag = 42; devs[1].stateKnown = true; devs[1].on = false;
  ctl::mergeDevices(devs, {cand(ip4(192, 168, 1, 9), "a", true, true), cand(ip4(192, 168, 1, 3), "b")}, 0, self, mask);
  CHECK_EQ(devs.size(), 3);
  CHECK_EQ(devs[1].tag, 42);
  CHECK(!devs[1].on);                  // known state is not overwritten by the node list hint
}

// Regression: the selected remote briefly missing from every source fell the selection back to THIS device.
static void testMergeKeepsSelected() {
  const uint32_t self = ip4(192, 168, 1, 50), mask = ip4(255, 255, 255, 0), sel = ip4(192, 168, 1, 9);
  std::vector<TDev> devs = localOnly();
  ctl::mergeDevices(devs, {cand(sel, "sel"), cand(ip4(192, 168, 1, 3), "other")}, sel, self, mask);
  devs[ctl::findDevice(devs, sel)].tag = 7;
  ctl::mergeDevices(devs, {cand(ip4(192, 168, 1, 3), "other")}, sel, self, mask);   // gone from discovery
  int i = ctl::findDevice(devs, sel);
  CHECK(i > 0);
  CHECK(!devs[i].listed);
  CHECK_EQ(devs[i].tag, 7);
  CHECK(!strcmp(devs[i].name, "sel"));
  ctl::mergeDevices(devs, {cand(ip4(192, 168, 1, 3), "other")}, 0, self, mask);     // not selected any more
  CHECK_EQ(ctl::findDevice(devs, sel), -1);
  ctl::mergeDevices(devs, {cand(sel, "sel")}, sel, self, mask);                    // back again
  i = ctl::findDevice(devs, sel);
  CHECK(i > 0 && devs[i].listed);
  CHECK_EQ(ctl::findDevice(devs, 0), 0);
}

static void testMergeMarksDuplicateNames() {
  const uint32_t self = ip4(192, 168, 1, 50), mask = ip4(255, 255, 255, 0);
  std::vector<TDev> devs = localOnly();
  ctl::mergeDevices(devs, {cand(ip4(192, 168, 1, 20), "WLED"), cand(ip4(192, 168, 1, 10), "wled"),
                           cand(ip4(192, 168, 1, 5), "Desk"), cand(ip4(192, 168, 1, 30), "WLED")}, 0, self, mask);
  CHECK_EQ(devs.size(), 5);
  CHECK(!strcmp(devs[1].name, "Desk") && !devs[1].dupName);
  for (int i = 2; i <= 4; i++) CHECK(devs[i].dupName);
  CHECK(devs[2].ip < devs[3].ip && devs[3].ip < devs[4].ip);   // same name: by address
}

static void testHealth() {
  uint8_t f = 0;
  f = ctl::nextFails(f, false); CHECK(ctl::reachableAfter(f));     // one lost reply: still shown
  f = ctl::nextFails(f, false); CHECK(!ctl::reachableAfter(f));    // two in a row: unreachable
  f = ctl::nextFails(f, true);  CHECK(ctl::reachableAfter(f)); CHECK_EQ(f, 0);
  f = 254; f = ctl::nextFails(f, false); f = ctl::nextFails(f, false); CHECK_EQ(f, 255);   // saturates
}

static void testPresetRetry() {
  CHECK_EQ(ctl::presetRetryMs(0), 10000);
  CHECK_EQ(ctl::presetRetryMs(1), 10000);
  CHECK_EQ(ctl::presetRetryMs(2), 20000);
  CHECK_EQ(ctl::presetRetryMs(3), 40000);
  CHECK_EQ(ctl::presetRetryMs(20), 300000);
  CHECK_EQ(ctl::presetRetryMs(255), 300000);
}

// --- worker queue -----------------------------------------------------------------------------------------
struct TReq { uint32_t ip = 0; uint8_t power = 0; int16_t bri = -1, ps = -1; bool poll = false, presets = false; uint32_t briDueAt = 0; };

static void testPickRequest() {
  TReq q[4];
  CHECK_EQ(ctl::pickRequest(q, 4, 1000), -1);
  q[0].ip = 1; q[0].poll = true;
  q[1].ip = 2; q[1].presets = true;
  CHECK_EQ(ctl::pickRequest(q, 4, 1000), 0);                       // background work in slot order
  q[2].ip = 3; q[2].power = 1;
  CHECK_EQ(ctl::pickRequest(q, 4, 1000), 2);                       // a command jumps the queue
  q[2] = TReq();
  q[3].ip = 4; q[3].bri = 50; q[3].briDueAt = 2000;
  CHECK_EQ(ctl::pickRequest(q, 4, 1000), 0);                       // brightness not due yet
  CHECK_EQ(ctl::pickRequest(q, 4, 2000), 3);                       // due: before the polls
  q[0] = TReq(); q[1] = TReq();
  CHECK_EQ(ctl::pickRequest(q, 4, 1000), -1);
  q[3].ps = 3;
  CHECK_EQ(ctl::pickRequest(q, 4, 1000), 3);
}

// --- HTTP -------------------------------------------------------------------------------------------------
static ctl::HttpHead parseHead(const char* s, size_t* consumed = nullptr) {
  ctl::HttpHead h;
  size_t i = 0;
  while (s[i] && h.feed(s[i])) i++;
  if (s[i]) i++;                       // the byte that finished the head
  if (consumed) *consumed = i;
  return h;
}

static void testHttpHead() {
  size_t used = 0;
  const char* ok = "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\ncontent-LENGTH:  57 \r\n\r\n{\"on\":true}";
  ctl::HttpHead h = parseHead(ok, &used);
  CHECK_EQ(h.state, ctl::HttpHead::DONE);
  CHECK_EQ(h.status, 200);
  CHECK_EQ(h.contentLength, 57);
  CHECK(!h.chunked);
  CHECK(!strcmp(ok + used, "{\"on\":true}"));                   // the body starts right after the head
  h = parseHead("HTTP/1.0 404 Not Found\n\n");                  // bare LF line ends
  CHECK_EQ(h.state, ctl::HttpHead::DONE); CHECK_EQ(h.status, 404); CHECK_EQ(h.contentLength, -1);
  h = parseHead("HTTP/1.1 200 OK\r\nTransfer-Encoding: gzip, Chunked\r\n\r\n");
  CHECK(h.chunked);
  CHECK_EQ(parseHead("SSH-2.0-OpenSSH\r\n\r\n").state, ctl::HttpHead::BAD);
  CHECK_EQ(parseHead("HTTP/1.1 2000 OK\r\n\r\n").state, ctl::HttpHead::BAD);
  CHECK_EQ(parseHead("HTTP/1.1 200 OK\r\nContent-Length: -5\r\n\r\n").state, ctl::HttpHead::BAD);
  CHECK_EQ(parseHead("HTTP/1.1 200 OK\r\nContent-Length: 12x\r\n\r\n").state, ctl::HttpHead::BAD);
  CHECK_EQ(parseHead("HTTP/1.1 200 OK\r\nContent-Length: 99999999999\r\n\r\n").state, ctl::HttpHead::BAD);
  // a host that never ends its head is cut off at kMaxHead bytes, not read forever
  ctl::HttpHead endless;
  const char* line = "HTTP/1.1 200 OK\r\n";
  for (const char* p = line; *p; p++) endless.feed(*p);
  size_t fed = 0;
  while (endless.feed('X') && fed < 100000) fed++;
  CHECK_EQ(endless.state, ctl::HttpHead::BAD);
  CHECK(fed < ctl::HttpHead::kMaxHead);
  // an over-long header line is truncated, not overflowed (ASan would catch a write past `line`)
  std::string longHdr = "HTTP/1.1 200 OK\r\nX-Long: " + std::string(1000, 'a') + "\r\nContent-Length: 3\r\n\r\n";
  h = parseHead(longHdr.c_str());
  CHECK_EQ(h.state, ctl::HttpHead::DONE); CHECK_EQ(h.contentLength, 3);
}

static void testBodyLimit() {
  CHECK_EQ(ctl::bodyLimit(-1, 8192), 8192);          // no length: read to close, capped
  CHECK_EQ(ctl::bodyLimit(100, 8192), 100);
  CHECK_EQ(ctl::bodyLimit(8192, 8192), 8192);
  CHECK_EQ(ctl::bodyLimit(8193, 8192), -1);          // too big: rejected unread
  CHECK_EQ(ctl::bodyLimit(0, 8192), 0);
}

// --- touch reset ------------------------------------------------------------------------------------------
static void testResetPulse() {
  ctl::ResetPulse r;
  CHECK(!r.busy());
  CHECK_EQ(r.step(0), ctl::ResetPulse::NONE);
  r.start(1000);
  CHECK(r.busy());
  CHECK_EQ(r.step(1100), ctl::ResetPulse::NONE);
  CHECK_EQ(r.step(1200), ctl::ResetPulse::DRIVE_HIGH);   // 200 ms low
  CHECK_EQ(r.step(1201), ctl::ResetPulse::NONE);
  CHECK(r.busy());
  CHECK_EQ(r.step(1499), ctl::ResetPulse::NONE);
  CHECK_EQ(r.step(1500), ctl::ResetPulse::FINISHED);     // 300 ms to boot
  CHECK(!r.busy());
  r.start(0xFFFFFF00u);                                  // across the millis() wrap
  CHECK_EQ(r.step(0xFFFFFF00u + 200), ctl::ResetPulse::DRIVE_HIGH);
  CHECK_EQ(r.step(0xFFFFFF00u + 500), ctl::ResetPulse::FINISHED);
}

// --- JD9853 init table ------------------------------------------------------------------------------------
struct Cmd { uint8_t cmd, n, delay5; const uint8_t* data; };

static std::vector<Cmd> walkInit() {
  std::vector<Cmd> out;
  const uint8_t* p = kJd9853Init;
  const uint8_t* end = kJd9853Init + sizeof(kJd9853Init);
  while (!(p[0] == 0xFF && p[1] == 0xFF)) {
    CHECK(p + 3 <= end);
    if (p + 3 > end) break;
    Cmd c{p[0], p[1], p[2 + p[1]], p + 2};
    CHECK(p + 3 + c.n <= end);
    if (p + 3 + c.n > end) break;
    out.push_back(c);
    p += 3 + c.n;
  }
  CHECK(p + 2 == end);       // the terminator is the last thing in the table
  return out;
}

static void testInitTable() {
  std::vector<Cmd> cmds = walkInit();
  CHECK_EQ(long(cmds.size()), 33);                   // Waveshare's lcd_reg_init has 33 commands
  CHECK_EQ(cmds.front().cmd, 0x11);                  // sleep out first, then a 120 ms settle
  CHECK_EQ(cmds.front().delay5 * 5, 120);
  CHECK_EQ(cmds.back().cmd, 0x29);                   // display on last
  CHECK_EQ(cmds[1].cmd, 0xDF);                       // vendor unlock: DF 98 53
  CHECK_EQ(cmds[1].n, 2); CHECK_EQ(cmds[1].data[0], 0x98); CHECK_EQ(cmds[1].data[1], 0x53);
  bool invon = false;
  for (size_t i = 0; i + 1 < cmds.size(); i++) if (cmds[i].cmd == 0x21) invon = true;
  CHECK(invon);                                      // this panel needs inversion on
  for (auto& c : cmds) CHECK(c.n <= 32);             // the gamma table (0xC8) is the longest at 32 bytes
  // The window the demo programs: columns 34..205 (172 wide, offset 34 in the controller's 240-wide RAM), rows 0..319.
  bool sawCol = false, sawRow = false;
  for (auto& c : cmds) {
    if (c.cmd == 0x2A) { sawCol = true; CHECK_EQ(c.n, 4); CHECK_EQ(c.data[0] * 256 + c.data[1], 34); CHECK_EQ(c.data[2] * 256 + c.data[3], 205); }
    if (c.cmd == 0x2B) { sawRow = true; CHECK_EQ(c.n, 4); CHECK_EQ(c.data[0] * 256 + c.data[1], 0);  CHECK_EQ(c.data[2] * 256 + c.data[3], kH - 1); }
  }
  CHECK(sawCol && sawRow);
}

// --- preset ordering ------------------------------------------------------------------------------------
struct Preset { uint8_t id; bool scene; };

static void testOrderPresetsEmpty() {
  std::vector<Preset> list;
  size_t count = ctl::orderPresets(list);
  CHECK_EQ(count, 0);
  CHECK_EQ(long(list.size()), 0);
}

static void testOrderPresetsNoPlaylists() {
  std::vector<Preset> list = {{1, false}, {3, false}, {2, false}};
  size_t count = ctl::orderPresets(list);
  CHECK_EQ(count, 0);
  CHECK_EQ(list.size(), 3);
  CHECK_EQ(list[0].id, 1); CHECK(!list[0].scene);
  CHECK_EQ(list[1].id, 2); CHECK(!list[1].scene);
  CHECK_EQ(list[2].id, 3); CHECK(!list[2].scene);
}

static void testOrderPresetsOnlyPlaylists() {
  std::vector<Preset> list = {{3, true}, {1, true}, {2, true}};
  size_t count = ctl::orderPresets(list);
  CHECK_EQ(count, 3);
  CHECK_EQ(list.size(), 3);
  CHECK_EQ(list[0].id, 1); CHECK(list[0].scene);
  CHECK_EQ(list[1].id, 2); CHECK(list[1].scene);
  CHECK_EQ(list[2].id, 3); CHECK(list[2].scene);
}

static void testOrderPresetsMixed() {
  // Mixed ids: 1c, 2p, 3c, 4p, 10p. Expected: 2p, 4p, 10p, 1c, 3c. Return 3.
  std::vector<Preset> list = {{1, false}, {2, true}, {3, false}, {4, true}, {10, true}};
  size_t count = ctl::orderPresets(list);
  CHECK_EQ(count, 3);
  CHECK_EQ(list.size(), 5);
  CHECK_EQ(list[0].id, 2); CHECK(list[0].scene);
  CHECK_EQ(list[1].id, 4); CHECK(list[1].scene);
  CHECK_EQ(list[2].id, 10); CHECK(list[2].scene);
  CHECK_EQ(list[3].id, 1); CHECK(!list[3].scene);
  CHECK_EQ(list[4].id, 3); CHECK(!list[4].scene);
}

static void testIsScene() {
  CHECK(ctl::isScene(true, ""));       // playlist
  CHECK(ctl::isScene(false, "BB"));    // plain preset with a quick-load label
  CHECK(ctl::isScene(true, "HH"));
  CHECK(!ctl::isScene(false, ""));     // helper preset
  CHECK(!ctl::isScene(false, nullptr));
}

int main() {
  testIsScene();
  testPercent();
  testFillIsProportional();
  testFillRadius();
  testSliderMapping();
  testTouchDecodeRealReports();
  testTouchRelease();
  testTouchClamps();
  testScroll();
  testElapsed();
  testInitTable();
  testOrderPresetsEmpty();
  testOrderPresetsNoPlaylists();
  testOrderPresetsOnlyPlaylists();
  testOrderPresetsMixed();
  testHasher();
  testInSubnet();
  testSweepRange();
  testBusUse();
  testMergeSortsAndFilters();
  testMergeKeepsState();
  testMergeKeepsSelected();
  testMergeMarksDuplicateNames();
  testHealth();
  testPresetRetry();
  testPickRequest();
  testHttpHead();
  testBodyLimit();
  testResetPulse();
  if (failures) { std::printf("%d check(s) failed\n", failures); return 1; }
  std::printf("all logic tests passed\n");
  return 0;
}
