#pragma once

/*
  BMP390 barometric pressure / temperature (header-only)

  - Default (RPL_BMP_MIN): Wire-only + Bosch integer math — smallest flash.
  - M0 / full: RPL_BMP_MIN unset — Adafruit BMP3XX + Bosch library.

  I2C: default 0x77; for 0x76 use build flag -D RPL_BMP390_I2C_ADDR=0x76 (min)
  or change kI2cAddr below (Adafruit path).
*/

#include <Arduino.h>

#if defined(RPL_BMP_MIN) && RPL_BMP_MIN
#include "bmp390_min.hpp"
#else
#include <Wire.h>
#include <Adafruit_BMP3XX.h>

#include "flight-log-types.hpp"

namespace bmp390 {

constexpr uint8_t kDefaultI2cAddr = BMP3XX_DEFAULT_ADDRESS;
constexpr uint8_t kI2cAddr = kDefaultI2cAddr;
constexpr float kSealevelPressureHpa = 1013.25f;

inline Adafruit_BMP3XX& sensor() {
  static Adafruit_BMP3XX instance;
  return instance;
}

inline void setup() {
  Wire.begin();

  if (!sensor().begin_I2C(kI2cAddr, &Wire)) {
    Serial.println(
        F("No BMP3XX — check I2C wiring, pull-ups, and address (0x76 vs 0x77)."));
    while (true) {
    }
  }

  sensor().setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  sensor().setPressureOversampling(BMP3_OVERSAMPLING_4X);
  sensor().setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  sensor().setOutputDataRate(BMP3_ODR_50_HZ);
}

inline bool sample_for_flight(flight::BmpSample& out) {
  if (!sensor().performReading()) {
    out.ok = false;
    out.temp_c = 0;
    out.pressure_hpa = 0;
    out.altitude_m = 0;
    return false;
  }
  out.ok = true;
  out.temp_c = sensor().temperature;
  out.pressure_hpa = sensor().pressure / 100.0f;
  out.altitude_m = sensor().readAltitude(kSealevelPressureHpa);
  return true;
}

inline void loop() {
  if (!sensor().performReading()) {
    Serial.println(F("performReading failed"));
    delay(500);
    return;
  }

  Serial.print(F("T_C: "));
  Serial.print(sensor().temperature, 2);
  Serial.print(F("  P_hPa: "));
  Serial.print(sensor().pressure / 100.0, 2);
  Serial.print(F("  alt_m (~): "));
  Serial.print(sensor().readAltitude(kSealevelPressureHpa), 1);
  Serial.println();

  delay(500);
}

}  // namespace bmp390
#endif
