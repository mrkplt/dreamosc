// display_core.h -- the OLED screen's logic, platform-free and host-tested
// (Fizzy #141). No Daisy dependency: the device glue (screen_oled.h) owns the
// I2C transport and libDaisy's OledDisplay; everything here decides WHAT is
// shown and WHICH part of the panel has to be re-sent.
//
// The panel (SSD1309, 128x64) stores its pixels in 8 horizontal BANDS of
// 128 x 8 pixels (the datasheet calls them "pages"; "band" here, so they are
// not mistaken for UI pages). One display byte is one 8-pixel column of one
// band, and a band is the smallest unit the panel can be re-addressed by, so
// it is the unit of every transfer: a change confined to one band costs one
// band on the bus (129 bytes, ~3 ms at 400 kHz), not the whole frame.
//
// Two economies keep the bus quiet:
//   1. A row redraws only when the number it SHOWS changes. Knob flutter is
//      removed at the knob (Deadband, controls_core.h), not here: the value
//      fed in is already still when the knob is.
//   2. The driver diffs the framebuffer against what the panel already holds
//      and sends only bands that differ (changedBands), so a caller never has
//      to track what is dirty on the panel.
#pragma once

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "stretch_core.h"   // SS_STEPS

constexpr int OLED_W      = 128;
constexpr int OLED_H      = 64;
constexpr int OLED_BAND_H = 8;
constexpr int OLED_BANDS  = OLED_H / OLED_BAND_H;
static_assert(OLED_BANDS <= 8, "band masks are uint8_t");

// The glyph cell the layout assumes: libDaisy's Font_6x8, whose 8-pixel
// height is exactly one band, so a text row drawn at y = band * 8 touches
// that band and no other.
constexpr int OLED_CHAR_W = 6;
constexpr int OLED_CHAR_H = 8;
static_assert(OLED_CHAR_H == OLED_BAND_H, "a text row must fill exactly one band");

// Bit b set when band b of `fb` differs from `shadow` (what the panel holds),
// or when bit b of `validMask` is clear (the panel's content for that band is
// unknown: never sent, or a send failed). Both buffers are OLED_W * OLED_BANDS
// bytes in the SSD130x layout (band-major, one byte per 8-pixel column).
inline uint8_t changedBands(const uint8_t* fb, const uint8_t* shadow, uint8_t validMask) {
  uint8_t mask = 0;
  for (int b = 0; b < OLED_BANDS; b++) {
    const bool valid = (validMask >> b) & 1u;
    if (!valid || memcmp(fb + b * OLED_W, shadow + b * OLED_W, OLED_W) != 0)
      mask |= (uint8_t)(1u << b);
  }
  return mask;
}

// Lowest set band in a mask, or -1 when the mask is empty.
inline int firstBand(uint8_t mask) {
  for (int b = 0; b < OLED_BANDS; b++)
    if ((mask >> b) & 1u) return b;
  return -1;
}

// The step-position screen: eight rows, POS1..POS8, one per band, each with
// that step's position (Sequencer::position[], the value the instrument
// plays -- after pickup, ownership and the speed-adaptive grid -- not the
// raw pot read) as 0..1000, the same scale as the PROFILE POS line.
//
// Usage (main loop): setPosition() for every step each pass -- it is a
// compare, and marks a row dirty only when its rounded number changes -- then
// draw() renders the dirty rows into the framebuffer through a Canvas:
//   void clearBand(int band);                       // blank one band
//   void text(int x, int y, const char* s);         // Font_6x8 at (x, y)
class PositionScreen {
 public:
  static constexpr int   ROWS  = SS_STEPS;
  static constexpr float SCALE = 1000.0f;   // position 0..1 -> 0..1000
  // Value column: right-aligned, 4 digits ("1000"), against the right edge.
  static constexpr int   VALUE_CHARS = 4;
  static constexpr int   VALUE_X     = OLED_W - VALUE_CHARS * OLED_CHAR_W;
  static_assert(ROWS <= OLED_BANDS, "one row per band");

  // Feed step `step`'s current position (0..1). Returns true when the shown
  // number changed (the row is now dirty).
  bool setPosition(int step, float pos) {
    if (step < 0 || step >= ROWS) return false;
    const uint8_t bit = (uint8_t)(1u << step);
    const int next = (int)lroundf(pos * SCALE);
    if ((primed_ & bit) && next == shown_[step]) return false;
    primed_ |= bit;
    shown_[step] = next;
    dirty_ |= bit;
    return true;
  }

  int  shown(int step) const { return shown_[step]; }
  bool primed(int step) const { return (primed_ >> step) & 1u; }
  bool pending() const { return dirty_ != 0; }
  uint8_t dirtyRows() const { return dirty_; }

  // Row text, exposed for tests. Label "POS<n>"; value right-aligned to
  // VALUE_CHARS, or blank until the step has been fed once.
  static void label(char* buf, size_t n, int step) { snprintf(buf, n, "POS%d", step + 1); }
  void value(char* buf, size_t n, int step) const {
    if (primed(step)) snprintf(buf, n, "%*d", VALUE_CHARS, shown_[step]);
    else              snprintf(buf, n, "%*s", VALUE_CHARS, "");
  }

  // Render every dirty row into the canvas's framebuffer and clear the dirty
  // set. Row r is band r: drawing it touches no other band.
  template <class Canvas>
  void draw(Canvas& c) {
    for (int r = 0; r < ROWS; r++) {
      if (!((dirty_ >> r) & 1u)) continue;
      char lab[8], val[8];
      label(lab, sizeof lab, r);
      value(val, sizeof val, r);
      c.clearBand(r);
      c.text(0, r * OLED_BAND_H, lab);
      c.text(VALUE_X, r * OLED_BAND_H, val);
    }
    dirty_ = 0;
  }

 private:
  int     shown_[ROWS] = {};
  uint8_t primed_ = 0;                          // rows fed at least once
  uint8_t dirty_  = (uint8_t)((1u << ROWS) - 1); // every label drawn on the first pass
};
