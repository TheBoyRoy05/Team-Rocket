#pragma once

/*
  Packed binary telemetry for LoRa (v1). Little-endian on wire (AVR native).

  Scales (decode on ground):
  - euler_*_cd: centi-degrees (value / 100.0f = deg)
  - acc_* / lin_* _cmps2: centi-m/s² (value / 100.0f = m/s²)
  - gyr_*_cdps: centi-deg/s (deg/s * 100; BNO055 reports deg/s)
  - mag_*_dt: deci-microtesla (value / 10.0f ≈ µT)
  - temp_cc: centi-°C
  - press_dhpa: deci-hPa (value / 10.0f = hPa)
  - alt_dm: deci-metres (value / 10.0f = m)
*/

#include <Arduino.h>
#include <stdint.h>
#include <string.h>

#include "flight-log-types.hpp"

struct FlightPacketV1 {
  uint8_t magic[2];
  uint8_t ver;
  uint32_t t_ms;
  uint8_t cal_sys;
  uint8_t cal_gyro;
  uint8_t cal_accel;
  uint8_t cal_mag;
  int16_t euler_h_cd;
  int16_t euler_r_cd;
  int16_t euler_p_cd;
  int16_t acc_gx_cmps2;
  int16_t acc_gy_cmps2;
  int16_t acc_gz_cmps2;
  int16_t lin_ax_cmps2;
  int16_t lin_ay_cmps2;
  int16_t lin_az_cmps2;
  int16_t gyr_x_cdps;
  int16_t gyr_y_cdps;
  int16_t gyr_z_cdps;
  int16_t mag_x_dt;
  int16_t mag_y_dt;
  int16_t mag_z_dt;
  uint8_t bmp_ok;
  int16_t temp_cc;
  uint16_t press_dhpa;
  int16_t alt_dm;
} __attribute__((packed));

namespace flight_packet {

constexpr uint8_t kMagic0 = 'T';
constexpr uint8_t kMagic1 = 'R';
constexpr uint8_t kVersion = 1;
constexpr size_t kEncodedSize = sizeof(FlightPacketV1);

namespace detail {

inline int16_t scale_clamp_f(float x, float mul) {
  const float v = x * mul;
  if (v >= 32767.0f) {
    return 32767;
  }
  if (v <= -32768.0f) {
    return -32768;
  }
  return static_cast<int16_t>(v);
}

inline uint16_t scale_clamp_u16(float x, float mul) {
  const float v = x * mul;
  if (v >= 65535.0f) {
    return 65535;
  }
  if (v <= 0.0f) {
    return 0;
  }
  return static_cast<uint16_t>(v);
}

}  // namespace detail

/** Returns wire length (kEncodedSize) or 0 if `cap` too small. */
inline size_t encode(uint8_t* dst, size_t cap, uint32_t t_ms,
                     const flight::ImuSample& imu, const flight::BmpSample& bmp) {
  if (cap < kEncodedSize || dst == nullptr) {
    return 0;
  }
  FlightPacketV1 p{};
  p.magic[0] = kMagic0;
  p.magic[1] = kMagic1;
  p.ver = kVersion;
  p.t_ms = t_ms;
  p.cal_sys = imu.cal_sys;
  p.cal_gyro = imu.cal_gyro;
  p.cal_accel = imu.cal_accel;
  p.cal_mag = imu.cal_mag;
  p.euler_h_cd = detail::scale_clamp_f(imu.euler_h, 100.0f);
  p.euler_r_cd = detail::scale_clamp_f(imu.euler_r, 100.0f);
  p.euler_p_cd = detail::scale_clamp_f(imu.euler_p, 100.0f);
  p.acc_gx_cmps2 = detail::scale_clamp_f(imu.acc_gx, 100.0f);
  p.acc_gy_cmps2 = detail::scale_clamp_f(imu.acc_gy, 100.0f);
  p.acc_gz_cmps2 = detail::scale_clamp_f(imu.acc_gz, 100.0f);
  p.lin_ax_cmps2 = detail::scale_clamp_f(imu.lin_ax, 100.0f);
  p.lin_ay_cmps2 = detail::scale_clamp_f(imu.lin_ay, 100.0f);
  p.lin_az_cmps2 = detail::scale_clamp_f(imu.lin_az, 100.0f);
  p.gyr_x_cdps = detail::scale_clamp_f(imu.gyr_x, 100.0f);
  p.gyr_y_cdps = detail::scale_clamp_f(imu.gyr_y, 100.0f);
  p.gyr_z_cdps = detail::scale_clamp_f(imu.gyr_z, 100.0f);
  p.mag_x_dt = detail::scale_clamp_f(imu.mag_x, 10.0f);
  p.mag_y_dt = detail::scale_clamp_f(imu.mag_y, 10.0f);
  p.mag_z_dt = detail::scale_clamp_f(imu.mag_z, 10.0f);
  if (bmp.ok) {
    p.bmp_ok = 1;
    p.temp_cc = detail::scale_clamp_f(bmp.temp_c, 100.0f);
    p.press_dhpa = detail::scale_clamp_u16(bmp.pressure_hpa, 10.0f);
    p.alt_dm = detail::scale_clamp_f(bmp.altitude_m, 10.0f);
  } else {
    p.bmp_ok = 0;
    p.temp_cc = 0;
    p.press_dhpa = 0;
    p.alt_dm = 0;
  }
  memcpy(dst, &p, kEncodedSize);
  return kEncodedSize;
}

/** If buffer is a v1 packet, print one CSV line. Optional `rssi_dbm` appends `,rssi`. */
inline bool try_print_decoded_csv(Stream& out, const uint8_t* buf, uint8_t len,
                                  const int16_t* rssi_dbm) {
  if (len < kEncodedSize || buf == nullptr) {
    return false;
  }
  FlightPacketV1 p;
  memcpy(&p, buf, kEncodedSize);
  if (p.magic[0] != kMagic0 || p.magic[1] != kMagic1 || p.ver != kVersion) {
    return false;
  }

  out.print(F("TR,"));
  out.print(p.t_ms);
  out.print(',');
  out.print(p.cal_sys);
  out.print(',');
  out.print(p.cal_gyro);
  out.print(',');
  out.print(p.cal_accel);
  out.print(',');
  out.print(p.cal_mag);
  out.print(',');
  out.print(p.euler_h_cd / 100.0f, 2);
  out.print(',');
  out.print(p.euler_r_cd / 100.0f, 2);
  out.print(',');
  out.print(p.euler_p_cd / 100.0f, 2);
  out.print(',');
  out.print(p.acc_gx_cmps2 / 100.0f, 3);
  out.print(',');
  out.print(p.acc_gy_cmps2 / 100.0f, 3);
  out.print(',');
  out.print(p.acc_gz_cmps2 / 100.0f, 3);
  out.print(',');
  out.print(p.lin_ax_cmps2 / 100.0f, 3);
  out.print(',');
  out.print(p.lin_ay_cmps2 / 100.0f, 3);
  out.print(',');
  out.print(p.lin_az_cmps2 / 100.0f, 3);
  out.print(',');
  out.print(p.gyr_x_cdps / 100.0f, 3);
  out.print(',');
  out.print(p.gyr_y_cdps / 100.0f, 3);
  out.print(',');
  out.print(p.gyr_z_cdps / 100.0f, 3);
  out.print(',');
  out.print(p.mag_x_dt / 10.0f, 3);
  out.print(',');
  out.print(p.mag_y_dt / 10.0f, 3);
  out.print(',');
  out.print(p.mag_z_dt / 10.0f, 3);
  out.print(',');
  out.print(p.bmp_ok);
  out.print(',');
  out.print(p.temp_cc / 100.0f, 2);
  out.print(',');
  out.print(p.press_dhpa / 10.0f, 2);
  out.print(',');
  out.print(p.alt_dm / 10.0f, 1);
  if (rssi_dbm != nullptr) {
    out.print(',');
    out.println(*rssi_dbm);
  } else {
    out.println();
  }
  return true;
}

inline bool try_print_decoded_csv(Stream& out, const uint8_t* buf, uint8_t len) {
  return try_print_decoded_csv(out, buf, len, nullptr);
}

/** CSV column header matching `try_print_decoded_csv` (+ optional RSSI). */
inline void print_tr_csv_header(Stream& out, bool with_rssi) {
  out.print(F("TR,t_ms,cal_sys,cal_gyro,cal_accel,cal_mag,"
              "euler_h,euler_r,euler_p,"
              "acc_gx,acc_gy,acc_gz,lin_ax,lin_ay,lin_az,"
              "gyr_x,gyr_y,gyr_z,mag_x,mag_y,mag_z,"
              "bmp_ok,temp_C,press_hPa,alt_m"));
  if (with_rssi) {
    out.print(F(",rssi"));
  }
  out.println();
}

/** 6-byte LoRa ACK payload (RadioHead adds RH header on TX). Echoes `t_ms` from FlightPacketV1. */
inline size_t encode_ack_payload(uint8_t* dst, size_t cap, uint32_t t_ms) {
  if (cap < 6 || dst == nullptr) {
    return 0;
  }
  dst[0] = 'A';
  dst[1] = 'K';
  dst[2] = static_cast<uint8_t>(t_ms);
  dst[3] = static_cast<uint8_t>(t_ms >> 8);
  dst[4] = static_cast<uint8_t>(t_ms >> 16);
  dst[5] = static_cast<uint8_t>(t_ms >> 24);
  return 6;
}

inline bool decode_ack_payload(const uint8_t* buf, uint8_t len, uint32_t* out_t_ms) {
  if (len < 6 || buf == nullptr || out_t_ms == nullptr) {
    return false;
  }
  if (buf[0] != 'A' || buf[1] != 'K') {
    return false;
  }
  *out_t_ms =
      static_cast<uint32_t>(buf[2]) | (static_cast<uint32_t>(buf[3]) << 8) |
      (static_cast<uint32_t>(buf[4]) << 16) | (static_cast<uint32_t>(buf[5]) << 24);
  return true;
}

/** User payload of a received RH-framed packet (after 4-byte header): TR v1 `t_ms` at offset 3. */
inline uint32_t read_t_ms_from_tr_payload(const uint8_t* buf, uint8_t len) {
  if (len < 7 || buf == nullptr) {
    return 0;
  }
  uint32_t t_ms;
  memcpy(&t_ms, buf + 3, 4);
  return t_ms;
}

}  // namespace flight_packet
