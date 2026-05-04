#pragma once

/*
  BMP390 barometric pressure / temperature test (header-only)

  I2C wiring: SDA/SCL to the Feather M0 I2C bus, 3V3 + GND.
  Default address 0x77; use 0x76 if the module ties SDO to GND.
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP3XX.h>

namespace bmp390 {

constexpr uint8_t kDefaultI2cAddr = BMP3XX_DEFAULT_ADDRESS;
// If your breakout uses 0x76 instead, set kI2cAddr to 0x76.
constexpr uint8_t kI2cAddr = kDefaultI2cAddr;

constexpr float kSealevelPressureHpa = 1013.25f;

inline Adafruit_BMP3XX& sensor() {
  static Adafruit_BMP3XX instance;
  return instance;
}

inline void setup() {
  Serial.begin(9600);
  Serial.println(F("BMP390 / BMP3XX test"));
  Serial.println();

  Wire.begin();

  if (!sensor().begin_I2C(kI2cAddr, &Wire)) {
    Serial.println(
        F("No BMP3XX — check I2C wiring, pull-ups, and address (0x76 vs 0x77)."));
    while (true) {
    }
  }

  Serial.print(F("chip_id: 0x"));
  Serial.println(sensor().chipID(), HEX);

  sensor().setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  sensor().setPressureOversampling(BMP3_OVERSAMPLING_4X);
  sensor().setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  sensor().setOutputDataRate(BMP3_ODR_50_HZ);
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
