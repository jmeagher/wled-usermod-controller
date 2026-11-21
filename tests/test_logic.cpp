// Desktop tests for ctl_logic.h and the JD9853 init table. Build and run with tests/run.sh.
#include <cstdio>
#include <cstring>
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
  testInitTable();
  testOrderPresetsEmpty();
  testOrderPresetsNoPlaylists();
  testOrderPresetsOnlyPlaylists();
  testOrderPresetsMixed();
  if (failures) { std::printf("%d check(s) failed\n", failures); return 1; }
  std::printf("all logic tests passed\n");
  return 0;
}
