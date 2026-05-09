#pragma once

/*
  Minimal BMP3xx/BMP390 on Wire: no Adafruit/Bosch library — only forced readings,
  integer compensation (BMP3 C driver, non–double-precision path).

  Register map follows Bosch BMP3 public driver (same as Adafruit BMP3XX).
*/

#include <Arduino.h>
#include <Wire.h>

#include "flight-log-types.hpp"

namespace bmp390 {

#ifndef RPL_BMP390_I2C_ADDR
#define RPL_BMP390_I2C_ADDR 0x77
#endif

constexpr float kSealevelPressureHpa = 1013.25f;

// --- BMP3 register map (subset) ---
constexpr uint8_t REG_CHIP_ID = 0x00;
constexpr uint8_t REG_SENS_STATUS = 0x03;
constexpr uint8_t REG_DATA = 0x04;
constexpr uint8_t REG_PWR_CTRL = 0x1b;
constexpr uint8_t REG_OSR = 0x1c;
constexpr uint8_t REG_ODR = 0x1d;
constexpr uint8_t REG_CONFIG = 0x1f;
constexpr uint8_t REG_CALIB_DATA = 0x31;
constexpr uint8_t REG_CMD = 0x7e;

constexpr uint8_t CHIP_ID_BMP3 = 0x50;
constexpr uint8_t CHIP_ID_BMP390 = 0x60;
constexpr uint8_t SOFT_RESET = 0xb6;

constexpr uint8_t MODE_SLEEP = 0;
constexpr uint8_t MODE_FORCED = 1;

constexpr uint8_t OSR_TEMP_8X = 3;
constexpr uint8_t OSR_PRESS_4X = 2;
constexpr uint8_t ODR_50_HZ = 2;
constexpr uint8_t IIR_COEFF_3 = 2;

constexpr uint8_t DRDY_P = 0x20;
constexpr uint8_t DRDY_T = 0x40;

constexpr uint8_t OP_MODE_MSK = 0x30;
constexpr uint8_t OP_MODE_POS = 4;

namespace detail {

struct CalibRegs {
  uint16_t par_t1{};
  uint16_t par_t2{};
  int8_t par_t3{};
  int16_t par_p1{};
  int16_t par_p2{};
  int8_t par_p3{};
  int8_t par_p4{};
  uint16_t par_p5{};
  uint16_t par_p6{};
  int8_t par_p7{};
  int8_t par_p8{};
  int16_t par_p9{};
  int8_t par_p10{};
  int8_t par_p11{};
  int64_t t_lin{};
};

inline uint8_t i2c_addr() { return RPL_BMP390_I2C_ADDR; }

inline bool write_reg(uint8_t reg, uint8_t v) {
  Wire.beginTransmission(i2c_addr());
  Wire.write(reg);
  Wire.write(v);
  return Wire.endTransmission() == 0;
}

inline bool read_regs(uint8_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(i2c_addr());
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  const uint8_t got = Wire.requestFrom(i2c_addr(), len);
  if (got != len) {
    return false;
  }
  for (uint8_t i = 0; i < len; i++) {
    buf[i] = Wire.read();
  }
  return true;
}

inline void parse_calib(const uint8_t* reg_data, CalibRegs* c) {
  c->par_t1 = (uint16_t)(((uint16_t)reg_data[1] << 8) | reg_data[0]);
  c->par_t2 = (uint16_t)(((uint16_t)reg_data[3] << 8) | reg_data[2]);
  c->par_t3 = (int8_t)reg_data[4];
  c->par_p1 = (int16_t)(((uint16_t)reg_data[6] << 8) | reg_data[5]);
  c->par_p2 = (int16_t)(((uint16_t)reg_data[8] << 8) | reg_data[7]);
  c->par_p3 = (int8_t)reg_data[9];
  c->par_p4 = (int8_t)reg_data[10];
  c->par_p5 = (uint16_t)(((uint16_t)reg_data[12] << 8) | reg_data[11]);
  c->par_p6 = (uint16_t)(((uint16_t)reg_data[14] << 8) | reg_data[13]);
  c->par_p7 = (int8_t)reg_data[15];
  c->par_p8 = (int8_t)reg_data[16];
  c->par_p9 = (int16_t)(((uint16_t)reg_data[18] << 8) | reg_data[17]);
  c->par_p10 = (int8_t)reg_data[19];
  c->par_p11 = (int8_t)reg_data[20];
}

inline void parse_sensor_data(const uint8_t* reg_data, uint32_t* press,
                              uint32_t* temp) {
  uint32_t p_x = (uint32_t)reg_data[0];
  uint32_t p_l = (uint32_t)reg_data[1] << 8;
  uint32_t p_m = (uint32_t)reg_data[2] << 16;
  *press = p_m | p_l | p_x;

  uint32_t t_x = (uint32_t)reg_data[3];
  uint32_t t_l = (uint32_t)reg_data[4] << 8;
  uint32_t t_m = (uint32_t)reg_data[5] << 16;
  *temp = t_m | t_l | t_x;
}

// Integer compensate — verbatim logic from Bosch bmp3.c (no double-precision).
inline int64_t compensate_temperature(uint32_t uncomp_temp, CalibRegs* c) {
  int64_t pd1 =
      ((int64_t)uncomp_temp - ((int64_t)256 * c->par_t1));
  int64_t pd2 = (int64_t)c->par_t2 * pd1;
  int64_t pd3 = (pd1 * pd1);
  int64_t pd4 = (int64_t)pd3 * c->par_t3;
  int64_t pd5 = ((int64_t)(pd2 * 262144) + pd4);
  int64_t pd6 = pd5 / 4294967296LL;
  c->t_lin = pd6;
  return (int64_t)((pd6 * 25) / 16384);
}

inline uint64_t compensate_pressure(uint32_t uncomp_press,
                                    const CalibRegs* c) {
  const CalibRegs& r = *c;
  int64_t pd1 = r.t_lin * r.t_lin;
  int64_t pd2 = pd1 / 64;
  int64_t pd3 = (pd2 * r.t_lin) / 256;
  int64_t pd4 = (r.par_p8 * pd3) / 32;
  int64_t pd5 = (r.par_p7 * pd1) * 16;
  int64_t pd6 = (r.par_p6 * r.t_lin) * 4194304;
  int64_t offset = (r.par_p5 * 140737488355328LL) + pd4 + pd5 + pd6;
  pd2 = (r.par_p4 * pd3) / 32;
  pd4 = (r.par_p3 * pd1) * 4;
  pd5 = (r.par_p2 - 16384) * r.t_lin * 2097152;
  int64_t sensitivity =
      ((r.par_p1 - 16384) * 70368744177664LL) + pd2 + pd4 + pd5;
  pd1 = (sensitivity / 16777216) * (int64_t)uncomp_press;
  pd2 = (int64_t)r.par_p10 * r.t_lin;
  pd3 = pd2 + (65536 * (int64_t)r.par_p9);
  pd4 = (pd3 * (int64_t)uncomp_press) / 8192;
  pd5 = ((int64_t)uncomp_press * (pd4 / 10)) / 512;
  pd5 = pd5 * 10;
  pd6 = (int64_t)((uint64_t)uncomp_press * (uint64_t)uncomp_press);
  pd2 = ((int64_t)r.par_p11 * pd6) / 65536;
  pd3 = (pd2 * (int64_t)uncomp_press) / 128;
  pd4 = (offset / 4) + pd1 + pd5 + pd3;
  return (uint64_t)(((uint64_t)pd4 * 25) / (uint64_t)1099511627776ULL);
}

inline CalibRegs& calib() {
  static CalibRegs c{};
  return c;
}

inline bool apply_static_config() {
  const uint8_t osr =
      static_cast<uint8_t>((OSR_TEMP_8X << 3) & 0x38) | (OSR_PRESS_4X & 0x07);
  if (!write_reg(REG_OSR, osr)) {
    return false;
  }
  if (!write_reg(REG_ODR, ODR_50_HZ & 0x1f)) {
    return false;
  }
  uint8_t cfg = 0;
  if (!read_regs(REG_CONFIG, &cfg, 1)) {
    return false;
  }
  cfg = static_cast<uint8_t>((cfg & static_cast<uint8_t>(~0x0e)) |
                             static_cast<uint8_t>((IIR_COEFF_3 << 1) & 0x0e));
  return write_reg(REG_CONFIG, cfg);
}

inline bool trigger_forced() {
  uint8_t pwr = 0;
  if (!read_regs(REG_PWR_CTRL, &pwr, 1)) {
    return false;
  }
  pwr = static_cast<uint8_t>(pwr | 0x03);
  pwr = static_cast<uint8_t>((pwr & static_cast<uint8_t>(~OP_MODE_MSK)) |
                             static_cast<uint8_t>(MODE_FORCED << OP_MODE_POS));
  return write_reg(REG_PWR_CTRL, pwr);
}

}  // namespace detail

inline void setup() {
  Wire.begin();

  detail::write_reg(REG_CMD, SOFT_RESET);
  delay(10);

  uint8_t id = 0;
  if (!detail::read_regs(REG_CHIP_ID, &id, 1) ||
      (id != CHIP_ID_BMP3 && id != CHIP_ID_BMP390)) {
    Serial.println(
        F("bmp390_min: no BMP — check I2C, wiring, address (0x76 vs 0x77)."));
    while (true) {
    }
  }

  uint8_t cal[21]{};
  if (!detail::read_regs(REG_CALIB_DATA, cal, 21)) {
    Serial.println(F("bmp390_min: calib read failed"));
    while (true) {
    }
  }
  detail::parse_calib(cal, &detail::calib());

  if (!detail::apply_static_config()) {
    Serial.println(F("bmp390_min: config failed"));
    while (true) {
    }
  }

  uint8_t pwr = 0x03;
  pwr = static_cast<uint8_t>((pwr & static_cast<uint8_t>(~OP_MODE_MSK)) |
                             static_cast<uint8_t>(MODE_SLEEP << OP_MODE_POS));
  if (!detail::write_reg(REG_PWR_CTRL, pwr)) {
    Serial.println(F("bmp390_min: pwr failed"));
    while (true) {
    }
  }
}

inline bool sample_for_flight(flight::BmpSample& out) {
  if (!detail::trigger_forced()) {
    out.ok = false;
    return false;
  }

  uint16_t spins = 0;
  uint8_t st = 0;
  while (spins < 500) {
    if (!detail::read_regs(REG_SENS_STATUS, &st, 1)) {
      out.ok = false;
      return false;
    }
    if ((st & (DRDY_P | DRDY_T)) == (DRDY_P | DRDY_T)) {
      break;
    }
    delayMicroseconds(200);
    spins++;
  }
  if ((st & (DRDY_P | DRDY_T)) != (DRDY_P | DRDY_T)) {
    out.ok = false;
    return false;
  }

  uint8_t raw[6]{};
  if (!detail::read_regs(REG_DATA, raw, 6)) {
    out.ok = false;
    return false;
  }

  uint32_t up = 0, ut = 0;
  detail::parse_sensor_data(raw, &up, &ut);

  const int64_t t_centi = detail::compensate_temperature(ut, &detail::calib());
  const uint64_t p_centi_pa = detail::compensate_pressure(up, &detail::calib());

  out.ok = true;
  out.temp_c = static_cast<float>(t_centi) / 100.0f;
  out.pressure_hpa = static_cast<float>(p_centi_pa) / 10000.0f;
  /*
    ~8.2 m/hPa near MSL (linearized); for ISA altitude use pressure on the
    ground station. Keeps powf off the 32u4 build.
  */
  out.altitude_m = (kSealevelPressureHpa - out.pressure_hpa) * 8.2f;
  return true;
}

inline void loop() {
  flight::BmpSample s{};
  if (!sample_for_flight(s) || !s.ok) {
    Serial.println(F("bmp sample failed"));
    delay(500);
    return;
  }
  Serial.print(F("T_C: "));
  Serial.print(s.temp_c, 2);
  Serial.print(F("  P_hPa: "));
  Serial.print(s.pressure_hpa, 2);
  Serial.print(F("  alt_m (~): "));
  Serial.print(s.altitude_m, 1);
  Serial.println();
  delay(500);
}

}  // namespace bmp390
