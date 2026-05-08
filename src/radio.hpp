#pragma once

/*
  Mirror everything you print over LoRa (RadioHead RH_RF95) while still showing
  it on USB Serial.

  Arduino cannot hook the built-in `Serial` symbol — anything that already calls
  `Serial.print` from other files will not be mirrored. Use `radio::log()` (or a
  sketch macro like `#define LOG radio::log()`) for output you want on air + USB,
  and call `radio::poll()` often from loop() so packets actually transmit.

  Typical setup order: Serial.begin → rf95.init / params → radio::begin(Serial, rf95).
*/

#include <Arduino.h>
#include <RH_RF95.h>

namespace radio {

#ifndef RADIO_LOG_BUF_CAP
#define RADIO_LOG_BUF_CAP 512
#endif
#ifndef RADIO_LOG_MAX_PACKET
#define RADIO_LOG_MAX_PACKET 200
#endif
#ifndef RADIO_LOG_TX_WAIT_MS
#define RADIO_LOG_TX_WAIT_MS 4000
#endif

class TeePrint final : public Print {
 public:
  void set_primary(Print* p) { primary_ = p; }
  void set_driver(RH_RF95* rf) { rf_ = rf; }
  void set_max_packet(size_t n) {
    if (n < 8) {
      n = 8;
    }
    if (n > RH_RF95_MAX_MESSAGE_LEN) {
      n = RH_RF95_MAX_MESSAGE_LEN;
    }
    max_packet_ = n;
  }
  void set_mirror_radio(bool on) { mirror_radio_ = on; }

  size_t write(uint8_t c) override {
    if (primary_) {
      primary_->write(c);
    }
    if (mirror_radio_ && rf_) {
      if (!push(c)) {
        ++drops_;
      }
    }
    return 1;
  }

  size_t write(const uint8_t* buf, size_t len) override {
    if (primary_) {
      primary_->write(buf, len);
    }
    if (mirror_radio_ && rf_) {
      for (size_t i = 0; i < len; ++i) {
        if (!push(buf[i])) {
          ++drops_;
          break;
        }
      }
    }
    return len;
  }

  /** Try to push buffered text out over LoRa (non-blocking aside from TX wait). */
  void poll();

  uint32_t tx_drops() const { return drops_; }
  void reset_tx_drops() { drops_ = 0; }

 private:
  bool push(uint8_t c);
  size_t find_newline_up_to(size_t max_len) const;

  Print* primary_{nullptr};
  RH_RF95* rf_{nullptr};
  size_t max_packet_{RADIO_LOG_MAX_PACKET};
  bool mirror_radio_{true};

  uint8_t buf_[RADIO_LOG_BUF_CAP]{};
  size_t head_{0};
  size_t tail_{0};
  size_t used_{0};
  uint32_t drops_{0};
};

inline TeePrint& log() {
  static TeePrint instance;
  return instance;
}

inline bool TeePrint::push(uint8_t c) {
  if (used_ >= RADIO_LOG_BUF_CAP) {
    return false;
  }
  buf_[tail_] = c;
  tail_ = (tail_ + 1) % RADIO_LOG_BUF_CAP;
  ++used_;
  return true;
}

inline size_t TeePrint::find_newline_up_to(size_t max_len) const {
  if (used_ == 0 || max_len == 0) {
    return 0;
  }
  const size_t scan = max_len < used_ ? max_len : used_;
  for (size_t i = 0; i < scan; ++i) {
    const size_t idx = (head_ + i) % RADIO_LOG_BUF_CAP;
    if (buf_[idx] == '\n') {
      return i + 1;
    }
  }
  return 0;
}

inline void TeePrint::poll() {
  if (!rf_ || used_ == 0) {
    return;
  }

  while (used_ > 0) {
    size_t chunk = 0;
    const size_t nl = find_newline_up_to(max_packet_);
    if (nl != 0) {
      chunk = nl;
    } else if (used_ >= max_packet_) {
      chunk = max_packet_;
    } else {
      break;
    }
    if (chunk > RH_RF95_MAX_MESSAGE_LEN) {
      chunk = RH_RF95_MAX_MESSAGE_LEN;
    }

    static uint8_t pkt[RH_RF95_MAX_MESSAGE_LEN];
    for (size_t i = 0; i < chunk; ++i) {
      pkt[i] = buf_[head_];
      head_ = (head_ + 1) % RADIO_LOG_BUF_CAP;
    }
    used_ -= chunk;

    if (!rf_->send(pkt, static_cast<uint8_t>(chunk))) {
      break;
    }
    if (!rf_->waitPacketSent(RADIO_LOG_TX_WAIT_MS)) {
      break;
    }
  }
}

inline void begin(Print& primary, RH_RF95& driver,
                  size_t max_packet_bytes = RADIO_LOG_MAX_PACKET) {
  log().set_primary(&primary);
  log().set_driver(&driver);
  log().set_max_packet(max_packet_bytes);
  log().set_mirror_radio(true);
}

inline void poll() { log().poll(); }

}  // namespace radio
