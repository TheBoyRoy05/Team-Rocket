#pragma once

#include <stdint.h>

namespace flight {

// Filled by imu::sample_for_flight / bmp390::sample_for_flight, then formatted by sd::append_flight_row.
struct ImuSample {
  uint8_t cal_sys{};
  uint8_t cal_gyro{};
  uint8_t cal_accel{};
  uint8_t cal_mag{};
  float euler_h{};
  float euler_r{};
  float euler_p{};
  float lin_ax{};
  float lin_ay{};
  float lin_az{};
  float gyr_x{};
  float gyr_y{};
  float gyr_z{};
  float mag_x{};
  float mag_y{};
  float mag_z{};
};

struct BmpSample {
  bool ok{};
  float temp_c{};
  float pressure_hpa{};
  float altitude_m{};
};

}  // namespace flight
