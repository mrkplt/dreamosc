// screen_oled.h -- the SSD1309 OLED on I2C1, through libDaisy's OledDisplay
// (Fizzy #141). DEVICE EDGE: includes daisy headers; the logic it runs
// (which bands changed, what each row shows) is display_core.h, host-tested.
//
// libDaisy's stack is kept whole -- OledDisplay, SSD130xDriver's init
// sequence, framebuffer and font drawing -- and two pieces are swapped in
// through SSD130xDriver's Transport template parameter and a subclass:
//
//   OledI2cTransport  replaces SSD130xI2CTransport, which wraps EVERY pixel
//     byte in its own I2C transaction (START, address, 0x40, one byte, STOP:
//     ~1050 transactions a frame) and discards every result, so one bad byte
//     left the bus wedged and each remaining byte blocking for its 1 s
//     timeout -- the main loop, which is the frame producer, stopped and the
//     sound froze (~3 s into the first OLED build). Here a band is ONE
//     transaction (0x40 then 128 bytes), every result is checked, the timeout
//     is a few ms, and the first failure latches the display off: a broken
//     display goes stale, it can never stall the producer.
//
//   BandOledDriver  replaces SSD130xDriver::Update(), which re-sends all 8
//     bands, with one that sends the FIRST band that differs from what the
//     panel holds (changedBands over a shadow copy) and returns. One call
//     blocks for at most one band (~3 ms at 400 kHz); UpdateFinished() is
//     false while bands are owed, so the main loop sends one band per pass,
//     between renders. libDaisy's own contract for asynchronous drivers.
//
// Bus: I2C1 on the Pod header's SCL/SDA (D11/D12 = PB8/PB9), 400 kHz (the
// module's rating; libDaisy's 1 MHz default is out of its spec), addr 0x3C.
#pragma once

#include "daisy_pod.h"
#include "dev/oled_ssd130x.h"
#include "hid/disp/oled_display.h"
#include "util/oled_fonts.h"

#include "display_core.h"

// Counters a transport writes and the PROFILE OLED line prints. Caller-owned
// (OledDisplay keeps its driver private, so the counts cannot be read back
// through it).
struct OledBusStats {
  uint32_t sends    = 0;   // I2C transactions attempted
  uint32_t errors   = 0;   // transactions (or the init) that failed
  uint32_t bands    = 0;   // bands sent whole
  bool     ok       = false;
};

class OledI2cTransport {
 public:
  // A band transaction is 129 bytes, ~3 ms on the wire at 400 kHz; 10 ms
  // bounds a stuck bus to one short stall before the latch shuts it off.
  static constexpr uint32_t TIMEOUT_MS = 10;

  struct Config {
    daisy::I2CHandle::Config i2c_config;
    uint8_t                  i2c_address = 0x3C;
    OledBusStats*            stats       = nullptr;   // required
    Config() {
      i2c_config.periph         = daisy::I2CHandle::Config::Peripheral::I2C_1;
      i2c_config.speed          = daisy::I2CHandle::Config::Speed::I2C_400KHZ;
      i2c_config.mode           = daisy::I2CHandle::Config::Mode::I2C_MASTER;
      i2c_config.pin_config.scl = daisy::Pin(daisy::PORTB, 8);
      i2c_config.pin_config.sda = daisy::Pin(daisy::PORTB, 9);
    }
  };

  void Init(const Config& c) {
    addr_  = c.i2c_address;
    stats_ = c.stats;
    stats_->ok = (i2c_.Init(c.i2c_config) == daisy::I2CHandle::Result::OK);
    if (!stats_->ok) stats_->errors++;
  }

  bool SendCommand(uint8_t cmd) {
    uint8_t buf[2] = {0x00, cmd};   // control byte: Co=0, D/C#=0 (command)
    return send(buf, sizeof buf);
  }

  // One transaction: the data control byte, then up to one band of bytes.
  bool SendData(const uint8_t* data, size_t n) {
    if (n > (size_t)OLED_W) return false;
    uint8_t buf[1 + OLED_W];
    buf[0] = 0x40;                  // control byte: Co=0, D/C#=1 (data stream)
    memcpy(buf + 1, data, n);
    return send(buf, (uint16_t)(n + 1));
  }

  bool ok() const { return stats_ && stats_->ok; }
  void countBand() { stats_->bands++; }

 private:
  bool send(uint8_t* buf, uint16_t n) {
    if (!ok()) return false;
    stats_->sends++;
    if (i2c_.TransmitBlocking(addr_, buf, n, TIMEOUT_MS) != daisy::I2CHandle::Result::OK) {
      stats_->errors++;
      stats_->ok = false;   // latched: no recovery yet (see the class comment)
      return false;
    }
    return true;
  }

  daisy::I2CHandle i2c_;
  uint8_t          addr_  = 0x3C;
  OledBusStats*    stats_ = nullptr;
};

class BandOledDriver : public daisy::SSD130xDriver<OLED_W, OLED_H, OledI2cTransport> {
 public:
  // Send the first band that differs from the panel, then return. The base
  // class's Init leaves the panel's RAM unknown, so every band starts invalid
  // and the first passes send the whole screen, one band per call.
  void Update() {
    if (!transport_.ok()) return;
    const int b = firstBand(changedBands(buffer_, shadow_, valid_));
    if (b < 0) return;
    // Band addressing (page addressing mode): band, column 0 (low nibble,
    // then high nibble -- 0x10 for 128-wide panels, as libDaisy sends).
    valid_ &= (uint8_t)~(1u << b);   // unknown until the send completes
    if (transport_.SendCommand((uint8_t)(0xB0 + b)) && transport_.SendCommand(0x00)
        && transport_.SendCommand(0x10)
        && transport_.SendData(&buffer_[OLED_W * b], OLED_W)) {
      memcpy(&shadow_[OLED_W * b], &buffer_[OLED_W * b], OLED_W);
      valid_ |= (uint8_t)(1u << b);
      transport_.countBand();
    }
  }

  // True when the panel matches the framebuffer, or the bus is latched off
  // (nothing more will be sent, so there is nothing to wait for).
  bool UpdateFinished() {
    return !transport_.ok() || changedBands(buffer_, shadow_, valid_) == 0;
  }

 private:
  uint8_t shadow_[OLED_W * OLED_BANDS] = {};
  uint8_t valid_ = 0;   // bands whose panel content is known (== shadow_)
};

using ScreenOled = daisy::OledDisplay<BandOledDriver>;

// The Canvas display_core.h's screens draw through: one band blanked, text in
// Font_6x8 (exactly one band tall).
struct OledCanvas {
  ScreenOled& d;
  void clearBand(int band) {
    const int y0 = band * OLED_BAND_H;
    d.DrawRect(0, y0, OLED_W - 1, y0 + OLED_BAND_H - 1, false, true);
  }
  void text(int x, int y, const char* s) {
    d.SetCursor((uint16_t)x, (uint16_t)y);
    d.WriteString(s, Font_6x8, true);
  }
};
