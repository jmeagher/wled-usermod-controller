// Pure logic of the controller usermod: no WLED, Arduino or hardware dependencies, so tests/test_logic.cpp can
// compile and run it on a desktop. controller.cpp uses these functions; keep behaviour changes here, with a test.
#pragma once
#include <algorithm>
#include <stdint.h>

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

}  // namespace ctl
