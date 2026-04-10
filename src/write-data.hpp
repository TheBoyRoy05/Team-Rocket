#pragma once

#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <stdio.h>

namespace write_data {

// Onboard microSD on Feather M0 Adalogger (see Adafruit pinouts). If you use the Adalogger
// FeatherWing on a plain Feather M0, try CS pin 10 instead.
constexpr int kSdCsPin = 4;

inline File& log_file() {
  static File f;
  return f;
}

inline void setup() {
  Serial.begin(115200);

  SPI.begin();
  if (!SD.begin(kSdCsPin)) {
    Serial.println(F("write_data: SD begin failed (card or CS pin?)"));
    return;
  }
  Serial.println(F("write_data: SD ready"));

  // Truncate once per boot so each session starts a fresh CSV.
  log_file() = SD.open("dummy.csv", FILE_WRITE);
  if (!log_file()) {
    Serial.println(F("write_data: could not open dummy.csv"));
    return;
  }
  log_file().println(F("millis,counter,temp_c,hum_pct"));
  log_file().flush();
}

inline void loop() {
  static uint32_t counter = 0;

  if (!log_file()) {
    delay(1000);
    return;
  }

  const unsigned long t = millis();
  const float temp_c = 20.0f + static_cast<float>(counter % 17) * 0.1f;
  const float hum_pct = 45.0f + static_cast<float>(counter % 23) * 0.2f;

  char line[56];
  const int n =
      snprintf(line, sizeof(line), "%lu,%lu,%.2f,%.2f\n", t,
               static_cast<unsigned long>(counter), static_cast<double>(temp_c),
               static_cast<double>(hum_pct));
  if (n <= 0 || n >= static_cast<int>(sizeof(line))) {
    Serial.println(F("write_data: snprintf failed or line too long"));
  } else {
    const size_t len = static_cast<size_t>(n);
    log_file().write(reinterpret_cast<const uint8_t*>(line), len);
    log_file().flush();
    Serial.print(F("logged: "));
    Serial.write(reinterpret_cast<const uint8_t*>(line), len);
  }

  ++counter;
  delay(1000);
}

}  // namespace write_data
