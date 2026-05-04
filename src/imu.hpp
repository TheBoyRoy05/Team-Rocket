#pragma once

/*
  BNO055 9-axis test (header-only)

  Fusion mode (NDOF) combines accelerometer, gyroscope, and magnetometer.
  getEvent(&event) alone only fills Euler/orientation; raw/fused vectors come
  from getVector().
*/

#include <Arduino.h>
#include <Adafruit_BNO055.h>

namespace imu {

inline Adafruit_BNO055& sensor() {
  static Adafruit_BNO055 instance(55);
  return instance;
}

inline void setup() {
  Serial.begin(9600);
  Serial.println(F("BNO055 9-axis + orientation"));
  Serial.println();

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

inline void loop() {
  uint8_t c_sys = 0, c_gyro = 0, c_accel = 0, c_mag = 0;
  sensor().getCalibration(&c_sys, &c_gyro, &c_accel, &c_mag);

  const imu::Vector<3> euler =
      sensor().getVector(Adafruit_BNO055::VECTOR_EULER);
  // Includes ~1 g from gravity; |a| ~ 9.8 m/s^2 when still.
  const imu::Vector<3> acc_grav_off =
      sensor().getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  // Gravity removed by fusion; ~0 when still (small offsets if uncalibrated).
  const imu::Vector<3> acc_linear =
      sensor().getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  const imu::Vector<3> gyr =
      sensor().getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  const imu::Vector<3> mag =
      sensor().getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

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
