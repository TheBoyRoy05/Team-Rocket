#pragma once

/*
  BNO055 9-axis test (header-only)

  Fusion mode (NDOF) combines accelerometer, gyroscope, and magnetometer.
  getEvent(&event) alone only fills Euler/orientation; raw/fused vectors come
  from getVector().
*/

#include <Arduino.h>
#include <Adafruit_BNO055.h>
#include <string.h>

#include "flight-log-types.hpp"

namespace imu {

inline Adafruit_BNO055& sensor() {
  static Adafruit_BNO055 instance(55);
  return instance;
}

namespace detail {
inline bool& initialized() {
  static bool v = false;
  return v;
}
}  // namespace detail

inline bool ready() { return detail::initialized(); }

#ifndef IMU_INIT_RETRY_MS
#define IMU_INIT_RETRY_MS 1000
#endif
#ifndef IMU_INIT_LOG_INTERVAL_MS
#define IMU_INIT_LOG_INTERVAL_MS 5000
#endif

/** Non-blocking: call from setup once and from loop until `ready()`. */
inline bool try_init() {
  if (detail::initialized()) {
    return true;
  }
  static unsigned long last_attempt_ms = 0;
  static unsigned long last_log_ms = 0;
  const unsigned long now = millis();
  if (last_attempt_ms != 0U && (now - last_attempt_ms) < IMU_INIT_RETRY_MS) {
    return false;
  }
  last_attempt_ms = now;

  if (!sensor().begin()) {
    if (last_log_ms == 0U || (now - last_log_ms) >= IMU_INIT_LOG_INTERVAL_MS) {
      Serial.println(F("imu: BNO055 not found — will retry"));
      last_log_ms = now;
    }
    return false;
  }

  delay(1000);
  sensor().setExtCrystalUse(true);
  detail::initialized() = true;
  Serial.println(F("imu: BNO055 ok"));
  return true;
}

// Fill flight::ImuSample for SD logging. If !ready(), fills zeros.
inline void sample_for_flight(flight::ImuSample& out) {
  if (!ready()) {
    memset(&out, 0, sizeof(out));
    return;
  }
  sensor().getCalibration(&out.cal_sys, &out.cal_gyro, &out.cal_accel,
                          &out.cal_mag);

  const auto euler = sensor().getVector(Adafruit_BNO055::VECTOR_EULER);
  const auto acc_linear =
      sensor().getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  const auto gyr = sensor().getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  const auto mag = sensor().getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

  out.euler_h = static_cast<float>(euler.x());
  out.euler_r = static_cast<float>(euler.y());
  out.euler_p = static_cast<float>(euler.z());
  out.lin_ax = static_cast<float>(acc_linear.x());
  out.lin_ay = static_cast<float>(acc_linear.y());
  out.lin_az = static_cast<float>(acc_linear.z());
  out.gyr_x = static_cast<float>(gyr.x());
  out.gyr_y = static_cast<float>(gyr.y());
  out.gyr_z = static_cast<float>(gyr.z());
  out.mag_x = static_cast<float>(mag.x());
  out.mag_y = static_cast<float>(mag.y());
  out.mag_z = static_cast<float>(mag.z());
}

inline void loop() {
  if (!ready()) {
    return;
  }
  uint8_t c_sys = 0, c_gyro = 0, c_accel = 0, c_mag = 0;
  sensor().getCalibration(&c_sys, &c_gyro, &c_accel, &c_mag);

  const auto euler = sensor().getVector(Adafruit_BNO055::VECTOR_EULER);
  // Gravity removed by fusion; ~0 when still (small offsets if uncalibrated).
  const auto acc_linear =
      sensor().getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  const auto gyr = sensor().getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  const auto mag = sensor().getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

  Serial.print(F("cal s,g,a,m: "));
  Serial.print(c_sys);
  Serial.print(F(","));
  Serial.print(c_gyro);
  Serial.print(F(","));
  Serial.print(c_accel);
  Serial.print(F(","));
  Serial.println(c_mag);

  Serial.print(F("euler deg H,R,P: "));
  Serial.print(euler.x(), 2);
  Serial.print(F("\t"));
  Serial.print(euler.y(), 2);
  Serial.print(F("\t"));
  Serial.println(euler.z(), 2);

  Serial.print(F("linear m/s^2: "));
  Serial.print(acc_linear.x(), 3);
  Serial.print(F("\t"));
  Serial.print(acc_linear.y(), 3);
  Serial.print(F("\t"));
  Serial.println(acc_linear.z(), 3);

  Serial.print(F("gyro deg/s: "));
  Serial.print(gyr.x(), 3);
  Serial.print(F("\t"));
  Serial.print(gyr.y(), 3);
  Serial.print(F("\t"));
  Serial.println(gyr.z(), 3);

  Serial.print(F("mag uT: "));
  Serial.print(mag.x(), 3);
  Serial.print(F("\t"));
  Serial.print(mag.y(), 3);
  Serial.print(F("\t"));
  Serial.println(mag.z(), 3);

  Serial.println();
  delay(100);
}

}  // namespace imu
