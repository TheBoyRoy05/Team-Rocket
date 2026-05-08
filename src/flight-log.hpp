#pragma once

#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "flight-log-types.hpp"

namespace flight_log {

// Match write_data.hpp / Feather Adalogger CS.
constexpr int kSdCsPin = 4;
constexpr char kFlightFilename[] = "flight.csv";
// One CSV row is ~250 bytes; keep ≥ that plus slack. Flushed each loop by default.
constexpr size_t kSdBufferCap = 320;

inline File& flight_file() {
  static File f;
  return f;
}

inline uint8_t* sd_buffer() {
  static uint8_t buf[kSdBufferCap];
  return buf;
}

inline size_t& sd_buffer_len() {
  static size_t len = 0;
  return len;
}

inline void flush_to_card() {
  File& f = flight_file();
  size_t& len = sd_buffer_len();
  if (f && len > 0) {
    f.write(sd_buffer(), len);
    f.flush();
    len = 0;
  }
}

// Append one line (must include trailing '\n'). Flushes to SD if the RAM buffer is full.
inline bool buffer_push_line(const char* line, size_t line_len) {
  if (line_len == 0 || line_len > kSdBufferCap) {
    return false;
  }
  size_t& len = sd_buffer_len();
  if (len + line_len > kSdBufferCap) {
    flush_to_card();
  }
  if (len + line_len > kSdBufferCap) {
    return false;
  }
  memcpy(sd_buffer() + len, line, line_len);
  len += line_len;
  return true;
}

inline void add_float_field(char* line, int* pos, size_t cap, float v,
                            int prec, char sep) {
  char num[16];
  dtostrf(static_cast<double>(v), 0, prec, num);
  const int n =
      snprintf(line + *pos, cap - static_cast<size_t>(*pos), "%s%c", num, sep);
  if (n > 0) {
    *pos += n;
  }
}

// Serial + SD init; opens flight.csv (truncated) and writes CSV header.
inline bool setup() {
  Serial.begin(115200);
  const unsigned long usb_wait_start = millis();
  while (!Serial && (millis() - usb_wait_start) < 5000) {
    delay(10);
  }

  SPI.begin();
  if (!SD.begin(kSdCsPin)) {
    Serial.println(F("flight_log: SD begin failed (card or CS pin?)"));
    return false;
  }
  Serial.println(F("flight_log: SD ok, flight.csv"));

  sd_buffer_len() = 0;
  flight_file() = SD.open(kFlightFilename, FILE_WRITE);
  if (!flight_file()) {
    Serial.println(F("flight_log: open flight.csv failed"));
    return false;
  }
  flight_file().println(
      F("millis,c_sys,c_gyro,c_accel,c_mag,"
        "euler_h,euler_r,euler_p,"
        "acc_gx,acc_gy,acc_gz,lin_ax,lin_ay,lin_az,"
        "gyr_x,gyr_y,gyr_z,mag_x,mag_y,mag_z,"
        "bmp_ok,bmp_t_c,bmp_p_hpa,bmp_alt_m"));
  flight_file().flush();
  return true;
}

// Format one combined row and push to the SD staging buffer.
inline bool append_row(unsigned long t, const flight::ImuSample& imu,
                       const flight::BmpSample& bmp) {
  static char line[272];
  int pos = snprintf(
      line, sizeof(line), "%lu,%u,%u,%u,%u,", t, static_cast<unsigned>(imu.cal_sys),
      static_cast<unsigned>(imu.cal_gyro), static_cast<unsigned>(imu.cal_accel),
      static_cast<unsigned>(imu.cal_mag));
  if (pos < 0 || static_cast<size_t>(pos) >= sizeof(line)) {
    return false;
  }

  add_float_field(line, &pos, sizeof(line), imu.euler_h, 2, ',');
  add_float_field(line, &pos, sizeof(line), imu.euler_r, 2, ',');
  add_float_field(line, &pos, sizeof(line), imu.euler_p, 2, ',');
  add_float_field(line, &pos, sizeof(line), imu.acc_gx, 3, ',');
  add_float_field(line, &pos, sizeof(line), imu.acc_gy, 3, ',');
  add_float_field(line, &pos, sizeof(line), imu.acc_gz, 3, ',');
  add_float_field(line, &pos, sizeof(line), imu.lin_ax, 3, ',');
  add_float_field(line, &pos, sizeof(line), imu.lin_ay, 3, ',');
  add_float_field(line, &pos, sizeof(line), imu.lin_az, 3, ',');
  add_float_field(line, &pos, sizeof(line), imu.gyr_x, 3, ',');
  add_float_field(line, &pos, sizeof(line), imu.gyr_y, 3, ',');
  add_float_field(line, &pos, sizeof(line), imu.gyr_z, 3, ',');
  add_float_field(line, &pos, sizeof(line), imu.mag_x, 3, ',');
  add_float_field(line, &pos, sizeof(line), imu.mag_y, 3, ',');
  add_float_field(line, &pos, sizeof(line), imu.mag_z, 3, ',');

  const int n = snprintf(line + pos, sizeof(line) - static_cast<size_t>(pos), "%u,",
                         bmp.ok ? 1u : 0u);
  if (n > 0) {
    pos += n;
  }
  if (bmp.ok) {
    add_float_field(line, &pos, sizeof(line), bmp.temp_c, 2, ',');
    add_float_field(line, &pos, sizeof(line), bmp.pressure_hpa, 2, ',');
    add_float_field(line, &pos, sizeof(line), bmp.altitude_m, 1, '\n');
  } else {
    snprintf(line + pos, sizeof(line) - static_cast<size_t>(pos), ",,\n");
  }

  return buffer_push_line(line, strlen(line));
}

// Call periodically (e.g. end of loop) so buffered bytes actually reach the card.
inline void flush() { flush_to_card(); }

}  // namespace flight_log
