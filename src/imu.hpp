#pragma once

/*
  BNO055 9-axis test (header-only)

  Fusion mode (NDOF) combines accelerometer, gyroscope, and magnetometer.
  getEvent(&event) alone only fills Euler/orientation; raw/fused vectors come
  from getVector().
*/

#include <Arduino.h>
#include <Adafruit_BNO055.h>

#include "flight-log-types.hpp"

namespace imu {

inline Adafruit_BNO055& sensor() {
  static Adafruit_BNO055 instance(55);
  return instance;
}

inline void setup() {
  // Call Serial.begin from sd::flight_log_setup (or sketch) before setup() if you
  // want the messages below on USB.

  // Default mode is OPERATION_MODE_NDOF (9-DoF sensor fusion).
  if (!sensor().begin()) {
    Serial.println(
        F("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!"));
    while (true) {
    }
  }

  delay(1000);
  sensor().setExtCrystalUse(true);
}

// Fill flight::ImuSample for SD logging (call sd::flight_log_setup / Serial first for debug).
inline void sample_for_flight(flight::ImuSample& out) {
  sensor().getCalibration(&out.cal_sys, &out.cal_gyro, &out.cal_accel,
                          &out.cal_mag);

  const auto euler = sensor().getVector(Adafruit_BNO055::VECTOR_EULER);
  const auto acc_grav_off =
      sensor().getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  const auto acc_linear =
      sensor().getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  const auto gyr = sensor().getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  const auto mag = sensor().getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

  out.euler_h = static_cast<float>(euler.x());
  out.euler_r = static_cast<float>(euler.y());
  out.euler_p = static_cast<float>(euler.z());
  out.acc_gx = static_cast<float>(acc_grav_off.x());
  out.acc_gy = static_cast<float>(acc_grav_off.y());
  out.acc_gz = static_cast<float>(acc_grav_off.z());
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
  uint8_t c_sys = 0, c_gyro = 0, c_accel = 0, c_mag = 0;
  sensor().getCalibration(&c_sys, &c_gyro, &c_accel, &c_mag);

  const auto euler = sensor().getVector(Adafruit_BNO055::VECTOR_EULER);
  // Includes ~1 g from gravity; |a| ~ 9.8 m/s^2 when still.
  const auto acc_grav_off =
      sensor().getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
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

  Serial.print(F("acc+grav m/s^2: "));
  Serial.print(acc_grav_off.x(), 3);
  Serial.print(F("\t"));
  Serial.print(acc_grav_off.y(), 3);
  Serial.print(F("\t"));
  Serial.println(acc_grav_off.z(), 3);

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
