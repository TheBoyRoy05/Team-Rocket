#pragma once

#include <Arduino.h>
#include <SD.h>
#include <SPI.h>

namespace read_data {

// Match write_data.hpp (Feather Adalogger / wing SD CS).
constexpr int kSdCsPin = 4;
constexpr char kLogFilename[] = "flight.csv";

// Call from your sketch setup(), e.g. after Serial is ready. Prints the full
// file to Serial (chunked for small RAM on AVR).
inline void setup() {
  Serial.begin(115200);
  const unsigned long usb_wait_start = millis();
  while (!Serial && (millis() - usb_wait_start) < 5000) {
    delay(10);
  }

  SPI.begin();
  if (!SD.begin(kSdCsPin)) {
    Serial.println(F("read_data: SD begin failed (card or CS pin?)"));
    return;
  }

  File f = SD.open(kLogFilename, FILE_READ);
  if (!f) {
    Serial.print(F("read_data: could not open "));
    Serial.println(kLogFilename);
    return;
  }

  Serial.print(F("read_data: --- "));
  Serial.print(kLogFilename);
  Serial.println(F(" ---"));
  uint8_t buf[64];
  while (f.available()) {
    const int n = f.read(buf, sizeof(buf));
    if (n > 0) {
      Serial.write(buf, static_cast<size_t>(n));
    }
  }
  f.close();
  Serial.println();
  Serial.println(F("read_data: --- end ---"));
}

}  // namespace read_data
