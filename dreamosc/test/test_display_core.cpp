// Unit tests for the OLED screen logic (display_core.h, Fizzy #141): which
// bands a framebuffer change dirties, and the POS1..POS8 screen model (redraw
// only when the SHOWN value changes, one row per band). Knob flutter is the
// knob's job (Deadband, test_controls_core.cpp).
//
// No CATCH_CONFIG_MAIN here -- test_stretch_core.cpp defines it.

#include "catch_amalgamated.hpp"

#include <algorithm>
#include <string>
#include <vector>

#include "display_core.h"

namespace {

using Buf = std::vector<uint8_t>;
Buf blank() { return Buf(OLED_W * OLED_BANDS, 0); }

// A canvas over a real SSD130x-layout framebuffer: clearBand zeroes the band,
// text writes each character's code into the first byte of its cell (enough
// to see WHICH bands a draw changes), and the calls are logged for content.
struct FakeCanvas {
  Buf fb = blank();
  std::vector<std::string> log;
  void clearBand(int band) {
    log.push_back("clear " + std::to_string(band));
    std::fill(fb.begin() + band * OLED_W, fb.begin() + (band + 1) * OLED_W, 0);
  }
  void text(int x, int y, const char* s) {
    log.push_back("text " + std::to_string(x) + "," + std::to_string(y) + " [" + s + "]");
    const int band = y / OLED_BAND_H;
    for (int i = 0; s[i]; i++)
      fb[band * OLED_W + x + i * OLED_CHAR_W] = (uint8_t)s[i];
  }
};

}  // namespace

TEST_CASE("changedBands: unknown bands are always dirty", "[display]") {
  Buf fb = blank(), shadow = blank();
  REQUIRE(changedBands(fb.data(), shadow.data(), 0x00) == 0xFF);
  REQUIRE(changedBands(fb.data(), shadow.data(), 0xFF) == 0x00);
  REQUIRE(changedBands(fb.data(), shadow.data(), 0xF0) == 0x0F);
}

TEST_CASE("changedBands: a pixel dirties exactly its own band", "[display]") {
  Buf shadow = blank();
  for (int b = 0; b < OLED_BANDS; b++) {
    for (int x : {0, 63, OLED_W - 1}) {
      Buf fb = blank();
      fb[b * OLED_W + x] = 0x01;
      REQUIRE(changedBands(fb.data(), shadow.data(), 0xFF) == (uint8_t)(1u << b));
    }
  }
}

TEST_CASE("firstBand: lowest set bit, -1 when empty", "[display]") {
  REQUIRE(firstBand(0x00) == -1);
  REQUIRE(firstBand(0x01) == 0);
  REQUIRE(firstBand(0x80) == 7);
  REQUIRE(firstBand(0x28) == 3);
}

TEST_CASE("PositionScreen: first feed primes every row and shows it", "[display]") {
  PositionScreen s;
  REQUIRE(s.pending());   // the labels are owed before any value
  for (int i = 0; i < SS_STEPS; i++) REQUIRE(s.setPosition(i, 0.1f * (float)i));
  for (int i = 0; i < SS_STEPS; i++) {
    REQUIRE(s.primed(i));
    REQUIRE(s.shown(i) == (int)lroundf(100.0f * (float)i));
  }
}

TEST_CASE("PositionScreen: a row redraws only when its rounded number changes", "[display]") {
  PositionScreen s;
  FakeCanvas c;
  for (int i = 0; i < SS_STEPS; i++) s.setPosition(i, 0.5f);
  s.draw(c);
  REQUIRE_FALSE(s.pending());
  // Moves inside the same displayed unit (500) are no redraw ...
  REQUIRE_FALSE(s.setPosition(2, 0.5004f));
  REQUIRE_FALSE(s.setPosition(2, 0.4996f));
  REQUIRE_FALSE(s.pending());
  // ... crossing to 501 is.
  REQUIRE(s.setPosition(2, 0.5006f));
  REQUIRE(s.shown(2) == 501);
  REQUIRE(s.dirtyRows() == (uint8_t)(1u << 2));
}

TEST_CASE("PositionScreen: a turn dirties only its own row", "[display]") {
  PositionScreen s;
  FakeCanvas c;
  for (int i = 0; i < SS_STEPS; i++) s.setPosition(i, 0.25f);
  s.draw(c);
  REQUIRE(s.setPosition(5, 0.75f));
  REQUIRE(s.dirtyRows() == (uint8_t)(1u << 5));
  REQUIRE(s.shown(5) == 750);
  // Out-of-range steps are ignored.
  REQUIRE_FALSE(s.setPosition(-1, 0.9f));
  REQUIRE_FALSE(s.setPosition(SS_STEPS, 0.9f));
  REQUIRE(s.dirtyRows() == (uint8_t)(1u << 5));
}

TEST_CASE("PositionScreen: draw renders POS1..POS8 and the 0..1000 value", "[display]") {
  PositionScreen s;
  FakeCanvas c;
  s.setPosition(0, 0.0f);
  s.setPosition(1, 1.0f);
  s.setPosition(7, 0.523f);
  s.draw(c);
  const int vx = PositionScreen::VALUE_X;
  auto has = [&](const std::string& line) {
    return std::find(c.log.begin(), c.log.end(), line) != c.log.end();
  };
  REQUIRE(has("text 0,0 [POS1]"));
  REQUIRE(has("text " + std::to_string(vx) + ",0 [   0]"));
  REQUIRE(has("text 0,8 [POS2]"));
  REQUIRE(has("text " + std::to_string(vx) + ",8 [1000]"));
  REQUIRE(has("text 0,56 [POS8]"));
  REQUIRE(has("text " + std::to_string(vx) + ",56 [ 523]"));
  // A step never fed shows its label and a blank value.
  REQUIRE(has("text 0,16 [POS3]"));
  REQUIRE(has("text " + std::to_string(vx) + ",16 [    ]"));
  // The value column ends exactly at the right edge.
  REQUIRE(vx + PositionScreen::VALUE_CHARS * OLED_CHAR_W == OLED_W);
}

TEST_CASE("PositionScreen: a redraw changes only that row's band on the panel", "[display]") {
  PositionScreen s;
  FakeCanvas c;
  for (int i = 0; i < SS_STEPS; i++) s.setPosition(i, 0.4f);
  s.draw(c);
  Buf shadow = c.fb;   // what the panel now holds
  REQUIRE(changedBands(c.fb.data(), shadow.data(), 0xFF) == 0);
  s.setPosition(3, 0.4111f);   // 411: one more digit cell lit than 400
  s.draw(c);
  REQUIRE(changedBands(c.fb.data(), shadow.data(), 0xFF) == (uint8_t)(1u << 3));
}
