#pragma once

/*
  TX-only SX1276/RFM95 (no RadioHead). PA_BOOST only (RFM95W / Adafruit wing).
  Same air format as RH_RF95: Bw125Cr45Sf128, preamble 8, CRC, sync 0x12,
  RH header 0xFF,0xFF,0x00,0x00 + payload.
*/

#include <Arduino.h>
#include <SPI.h>

#ifndef RPL_LORA_ACK
#define RPL_LORA_ACK 0
#endif

namespace rpl_rf95 {

#ifndef RPL_RF95_SPI_WRITE_MASK
#define RPL_RF95_SPI_WRITE_MASK 0x80
#endif

#ifndef RPL_RF95_MAX_USER_LEN
#define RPL_RF95_MAX_USER_LEN 64
#endif

// 1 MHz: more reliable than 4 MHz on some Feather + RFM95 wings (shared SPI).
#ifndef RPL_RF95_SPI_HZ
#define RPL_RF95_SPI_HZ 1000000u
#endif

// Feather 32u4 Adalogger onboard SD uses CS 4 — keep it high on shared SPI.
#ifndef RPL_SDCARD_CS_PIN
#define RPL_SDCARD_CS_PIN 4
#endif

class TxMin {
 public:
  explicit TxMin(uint8_t cs_pin) : cs_(cs_pin) {}

  /** REG_OP_MODE: sleep + LoRa = hi bit + mode 0; reserved bits may read non‑zero. */
  static bool op_mode_is_sleep_lora(uint8_t r) { return (r & 0x87u) == 0x80u; }

  bool begin() {
    pinMode(RPL_SDCARD_CS_PIN, OUTPUT);
    digitalWrite(RPL_SDCARD_CS_PIN, HIGH);
    pinMode(cs_, OUTPUT);
    digitalWrite(cs_, HIGH);
    SPI.begin();
    delay(100);
    wr(0x01, uint8_t(0x80 | 0x00));
    delay(10);
    uint8_t om = rd(0x01);
    if (!op_mode_is_sleep_lora(om)) {
      delay(10);
      wr(0x01, uint8_t(0x80 | 0x00));
      delay(10);
      om = rd(0x01);
      if (!op_mode_is_sleep_lora(om)) {
        return false;
      }
    }
    wr(0x0e, 0);
    wr(0x0f, 0);
    wr(0x01, uint8_t(0x80 | 0x01));
    wr(0x1d, 0x72);
    wr(0x1e, 0x74);
    wr(0x26, 0x04);
    wr(0x20, 0);
    wr(0x21, 8);
    return true;
  }

  void setFrequencyMhz(float mhz) {
    const uint32_t frf =
        static_cast<uint32_t>((mhz * 1.0e6f * 524288.0f) / 32.0e6f);
    wr(0x06, static_cast<uint8_t>((frf >> 16) & 0xff));
    wr(0x07, static_cast<uint8_t>((frf >> 8) & 0xff));
    wr(0x08, static_cast<uint8_t>(frf & 0xff));
  }

  void setSyncWord(uint8_t sw) { wr(0x39, sw); }

  /** PA_BOOST path only; same limits as RH_RF95 (max +20 dBm, +3 via PA_DAC). */
  void setTxPowerDbm(int8_t dbm) {
    if (dbm > 20) {
      dbm = 20;
    }
    if (dbm < 2) {
      dbm = 2;
    }
    if (dbm > 17) {
      wr(0x4d, 0x07);
      dbm -= 3;
    } else {
      wr(0x4d, 0x04);
    }
    wr(0x09, uint8_t(0x80 | static_cast<uint8_t>(dbm - 2)));
  }

  bool send(const uint8_t* data, uint8_t len) {
    if (data == nullptr || len == 0 || len > RPL_RF95_MAX_USER_LEN) {
      return false;
    }
    // Do not call waitPacketSent(0) here: TxDone is 0 before the first packet, so timeout
    // 0 would spin forever. Only wait if still in LoRa TX (e.g. last send missing wait).
    if ((rd(0x01) & 0x07u) == 0x03u) {
      if (!waitPacketSent(30000)) {
        return false;
      }
    } else {
      wr(0x12, 0xff);
    }
    wr(0x01, uint8_t(0x80 | 0x01));

    static const uint8_t rh[4] = {0xff, 0xff, 0x00, 0x00};
    const uint8_t total = static_cast<uint8_t>(len + 4);
    wr(0x0d, 0);
    fifo_wr(rh, 4);
    fifo_wr(data, len);
    wr(0x22, total);
    wr(0x12, 0xff);
    wr(0x40, 0x40);
    wr(0x01, uint8_t(0x80 | 0x03));
    return true;
  }

  bool waitPacketSent(uint16_t timeout_ms) {
    const uint32_t start = millis();
    for (;;) {
      if (rd(0x12) & 0x08) {
        wr(0x12, 0xff);
        wr(0x01, uint8_t(0x80 | 0x01));
        return true;
      }
      if (timeout_ms != 0 && (millis() - start) >= timeout_ms) {
        return false;
      }
    }
  }

  /** After TX: RH-framed ACK (AK + LE t_ms). Compiled only if RPL_LORA_ACK. */
#if RPL_LORA_ACK
  bool receiveAck(uint32_t expect_t_ms, uint16_t timeout_ms) {
    wr(0x01, uint8_t(0x80 | 0x01));
    delayMicroseconds(100);
    wr(0x12, 0xff);
    wr(0x01, uint8_t(0x80 | 0x05));
    const uint32_t start = millis();
    for (;;) {
      const uint8_t irq = rd(0x12);
      if (irq & 0x40u) {
        const bool crc_bad = (irq & 0x20u) != 0;
        const uint8_t n = rd(0x13);
        const uint8_t addr = rd(0x10);
        wr(0x0d, addr);
        uint8_t raw[12];
        if (n < 10 || n > sizeof(raw)) {
          wr(0x12, 0xff);
          wr(0x01, uint8_t(0x80 | 0x01));
          return false;
        }
        fifo_rd(raw, n);
        wr(0x12, 0xff);
        wr(0x01, uint8_t(0x80 | 0x01));
        if (crc_bad || raw[0] != 0xff || raw[1] != 0xff || raw[2] != 0x00 ||
            raw[3] != 0x00 || raw[4] != 'A' || raw[5] != 'K') {
          return false;
        }
        const uint32_t got_ms =
            static_cast<uint32_t>(raw[6]) | (static_cast<uint32_t>(raw[7]) << 8) |
            (static_cast<uint32_t>(raw[8]) << 16) | (static_cast<uint32_t>(raw[9]) << 24);
        return got_ms == expect_t_ms;
      }
      if (millis() - start >= timeout_ms) {
        wr(0x01, uint8_t(0x80 | 0x01));
        wr(0x12, 0xff);
        return false;
      }
    }
  }
#endif

 private:
  uint8_t cs_;

  void sel(bool on) { digitalWrite(cs_, on ? LOW : HIGH); }

  uint8_t rd(uint8_t r) {
    uint8_t v;
    SPI.beginTransaction(SPISettings(RPL_RF95_SPI_HZ, MSBFIRST, SPI_MODE0));
    sel(true);
    SPI.transfer(static_cast<uint8_t>(r & ~RPL_RF95_SPI_WRITE_MASK));
    v = SPI.transfer(0);
    sel(false);
    SPI.endTransaction();
    delayMicroseconds(2);
    return v;
  }

  void wr(uint8_t r, uint8_t v) {
    SPI.beginTransaction(SPISettings(RPL_RF95_SPI_HZ, MSBFIRST, SPI_MODE0));
    sel(true);
    SPI.transfer(static_cast<uint8_t>(r | RPL_RF95_SPI_WRITE_MASK));
    SPI.transfer(v);
    sel(false);
    SPI.endTransaction();
    delayMicroseconds(2);
  }

  void fifo_wr(const uint8_t* p, uint8_t n) {
    SPI.beginTransaction(SPISettings(RPL_RF95_SPI_HZ, MSBFIRST, SPI_MODE0));
    sel(true);
    SPI.transfer(0x80);
    while (n--) {
      SPI.transfer(*p++);
    }
    sel(false);
    SPI.endTransaction();
    delayMicroseconds(2);
  }

  void fifo_rd(uint8_t* p, uint8_t n) {
    SPI.beginTransaction(SPISettings(RPL_RF95_SPI_HZ, MSBFIRST, SPI_MODE0));
    sel(true);
    SPI.transfer(0x00);
    while (n--) {
      *p++ = SPI.transfer(0x00);
    }
    sel(false);
    SPI.endTransaction();
    delayMicroseconds(2);
  }
};

}  // namespace rpl_rf95
