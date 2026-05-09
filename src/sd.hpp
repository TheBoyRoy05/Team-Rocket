#pragma once

/*
  Feather M0 Adalogger (and similar): SPI SD card helpers — init, diagnostics,
  read to Serial, write with explicit paths/payloads, flight CSV buffering.

  Default CS: 4 (onboard microSD). Adalogger FeatherWing on a plain Feather
  often uses CS 10 — pass a different pin to begin() / print_card_info().
*/

#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "flight-log-types.hpp"

#if defined(RPL_SD_BINARY) && RPL_SD_BINARY
#include "flight_packet.hpp"
#endif

namespace sd {

constexpr int kCsPin = 4;
constexpr char kFlightCsv[] = "flight.csv";

#ifndef RPL_SD_FLIGHT_BUF_CAP
#define RPL_SD_FLIGHT_BUF_CAP 320
#endif
constexpr size_t kFlightBufferCap = RPL_SD_FLIGHT_BUF_CAP;

namespace detail {

inline File& flight_file() {
  static File f;
  return f;
}

inline uint8_t* flight_sd_buffer() {
  static uint8_t buf[kFlightBufferCap];
  return buf;
}

inline size_t& flight_sd_buffer_len() {
  static size_t len = 0;
  return len;
}

inline File& session_file() {
  static File f;
  return f;
}

inline void setup_32u4() {
  Serial.begin(115200);
  const unsigned long usb_wait_start = millis();
  while (!Serial && (millis() - usb_wait_start) < 5000) {
    delay(10);
  }
}

inline void flight_flush_to_card() {
  File& f = flight_file();
  size_t& len = flight_sd_buffer_len();
  if (f && len > 0) {
    f.write(flight_sd_buffer(), len);
    f.flush();
    len = 0;
  }
}

inline bool flight_buffer_push_line(const char* line, size_t line_len) {
  if (line_len == 0 || line_len > kFlightBufferCap) {
    return false;
  }
  size_t& len = flight_sd_buffer_len();
  if (len + line_len > kFlightBufferCap) {
    flight_flush_to_card();
  }
  if (len + line_len > kFlightBufferCap) {
    return false;
  }
  memcpy(flight_sd_buffer() + len, line, line_len);
  len += line_len;
  return true;
}

inline void add_float_field(char* line, int* pos, size_t cap, float v,
                            int prec, char sep) {
  char num[24];
#if defined(__AVR__)
  dtostrf(static_cast<double>(v), 0, prec, num);
  num[sizeof(num) - 1] = '\0';
#else
  snprintf(num, sizeof(num), "%.*f", prec, static_cast<double>(v));
#endif
  const int n =
      snprintf(line + *pos, cap - static_cast<size_t>(*pos), "%s%c", num, sep);
  if (n > 0) {
    *pos += n;
  }
}

}  // namespace detail

/** USB Serial @ 115200 with short wait for host (Feather 32u4 CDC). */
inline void setup_32u4() { detail::setup_32u4(); }

inline bool begin(int cs_pin = kCsPin) {
  SPI.begin();
  return SD.begin(cs_pin);
}

inline bool remove_file(const char* path) {
  if (!SD.exists(path)) {
    return true;
  }
  return SD.remove(path);
}

#if !defined(RPL_MINIMAL_SD) || !RPL_MINIMAL_SD

// Low-level card + FAT summary (calls SPI.begin). Does not use SD.begin().
// Serial must already be configured if you want output.
inline bool print_card_info(int cs_pin = kCsPin) {
  SPI.begin();
  Sd2Card card;
  SdVolume volume;
  SdFile root;

  Serial.print(F("\nInitializing SD card..."));

  if (!card.init(SPI_HALF_SPEED, cs_pin)) {
    Serial.println(F("initialization failed. Things to check:"));
    Serial.println(F("* is a card inserted?"));
    Serial.println(F("* is your wiring correct?"));
    Serial.println(
        F("* did you change the chipSelect pin to match your shield or module?"));
    return false;
  }
  Serial.println(F("Wiring is correct and a card is present."));

  Serial.println();
  Serial.print(F("Card type:         "));
  switch (card.type()) {
    case SD_CARD_TYPE_SD1:
      Serial.println(F("SD1"));
      break;
    case SD_CARD_TYPE_SD2:
      Serial.println(F("SD2"));
      break;
    case SD_CARD_TYPE_SDHC:
      Serial.println(F("SDHC"));
      break;
    default:
      Serial.println(F("Unknown"));
  }

  if (!volume.init(card)) {
    Serial.println(
        F("Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card"));
    return false;
  }

  Serial.print(F("Clusters:          "));
  Serial.println(volume.clusterCount());
  Serial.print(F("Blocks x Cluster:  "));
  Serial.println(volume.blocksPerCluster());

  Serial.print(F("Total Blocks:      "));
  Serial.println(volume.blocksPerCluster() * volume.clusterCount());
  Serial.println();

  uint32_t volumesize;
  Serial.print(F("Volume type is:    FAT"));
  Serial.println(volume.fatType(), DEC);

  volumesize = volume.blocksPerCluster();
  volumesize *= volume.clusterCount();
  volumesize /= 2;
  Serial.print(F("Volume size (Kb):  "));
  Serial.println(volumesize);
  Serial.print(F("Volume size (Mb):  "));
  volumesize /= 1024;
  Serial.println(volumesize);
  Serial.print(F("Volume size (Gb):  "));
  Serial.println(static_cast<float>(volumesize) / 1024.0f);

  Serial.println(F("\nFiles found on the card (name, date and size in bytes): "));
  root.openRoot(volume);
  root.ls(LS_R | LS_DATE | LS_SIZE);
  return true;
}

// Requires begin() first. Dumps an entire file to Serial in small chunks.
inline void dump_file_to_serial(const char* path) {
  File f = SD.open(path, FILE_READ);
  if (!f) {
    Serial.print(F("sd: could not open "));
    Serial.println(path);
    return;
  }

  Serial.print(F("sd: --- "));
  Serial.print(path);
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
  Serial.println(F("sd: --- end ---"));
}

inline bool append_bytes(const char* path, const uint8_t* data, size_t len) {
  if (len == 0) {
    return true;
  }
  File f = SD.open(path, FILE_WRITE);
  if (!f) {
    return false;
  }
  const size_t w = f.write(data, len);
  f.close();
  return w == len;
}

inline bool overwrite_file(const char* path, const uint8_t* data, size_t len) {
  SD.remove(path);
  File f = SD.open(path, FILE_WRITE);
  if (!f) {
    return false;
  }
  const size_t w = len == 0 ? 0 : f.write(data, len);
  f.close();
  return w == len;
}

// Appends one line (caller supplies '\n' if desired). If truncate_first,
// the file is replaced before writing.
inline bool println_file(const char* path, const char* line, bool truncate_first) {
  if (truncate_first) {
    SD.remove(path);
  }
  File f = SD.open(path, FILE_WRITE);
  if (!f) {
    return false;
  }
  f.println(line);
  f.close();
  return true;
}

// Open one file for repeated writes (see write-data demo pattern).
inline bool open_write_session(const char* path, bool truncate_existing) {
  if (truncate_existing) {
    SD.remove(path);
  }
  detail::session_file() = SD.open(path, FILE_WRITE);
  return static_cast<bool>(detail::session_file());
}

inline File& session_file() { return detail::session_file(); }

inline bool session_write(const uint8_t* data, size_t len) {
  File& f = detail::session_file();
  if (!f || len == 0) {
    return false;
  }
  return f.write(data, len) == len;
}

inline void session_flush() {
  File& f = detail::session_file();
  if (f) {
    f.flush();
  }
}

#endif  // !RPL_MINIMAL_SD

// Serial + SD; opens flight.csv and writes CSV header (omit when RPL_SD_BINARY).
#if !(defined(RPL_SD_BINARY) && RPL_SD_BINARY)
inline bool flight_log_setup(int cs_pin = kCsPin) {
  Serial.begin(115200);
  const unsigned long usb_wait_start = millis();
  while (!Serial && (millis() - usb_wait_start) < 5000) {
    delay(10);
  }

  if (!begin(cs_pin)) {
    Serial.println(F("sd: flight log — SD begin failed (card or CS pin?)"));
    return false;
  }
  Serial.println(F("sd: flight log — ok, flight.csv"));

  detail::flight_sd_buffer_len() = 0;
  SD.remove(kFlightCsv);
  detail::flight_file() = SD.open(kFlightCsv, FILE_WRITE);
  if (!detail::flight_file()) {
    Serial.println(F("sd: open flight.csv failed"));
    return false;
  }
  detail::flight_file().println(
#if defined(RPL_COMPACT_CSV) && RPL_COMPACT_CSV
      F("millis,euler_h,euler_r,euler_p,lin_az,gyr_z,"
        "bmp_ok,bmp_t_c,bmp_p_hpa,bmp_alt_m")
#else
      F("millis,c_sys,c_gyro,c_accel,c_mag,"
        "euler_h,euler_r,euler_p,"
        "acc_gx,acc_gy,acc_gz,lin_ax,lin_ay,lin_az,"
        "gyr_x,gyr_y,gyr_z,mag_x,mag_y,mag_z,"
        "bmp_ok,bmp_t_c,bmp_p_hpa,bmp_alt_m")
#endif
  );
  detail::flight_file().flush();
  return true;
}
#endif

#if defined(RPL_SD_BINARY) && RPL_SD_BINARY
constexpr char kFlightBin[] = "flight.bin";

inline bool flight_log_setup_bin(int cs_pin = kCsPin) {
  Serial.begin(115200);
  const unsigned long usb_wait_start = millis();
  while (!Serial && (millis() - usb_wait_start) < 5000) {
    delay(10);
  }

  if (!begin(cs_pin)) {
    Serial.println(F("sd: flight.bin — SD begin failed"));
    return false;
  }
  Serial.println(F("sd: flight.bin ok"));

  detail::flight_sd_buffer_len() = 0;
  SD.remove(kFlightBin);
  detail::flight_file() = SD.open(kFlightBin, FILE_WRITE);
  if (!detail::flight_file()) {
    Serial.println(F("sd: open flight.bin failed"));
    return false;
  }
  const uint8_t hdr[4] = {
      'T', 'R', 'B',
#if defined(RPL_ENABLE_LORA) && RPL_ENABLE_LORA
      2
#else
      1
#endif
  };
  detail::flight_file().write(hdr, sizeof(hdr));
  detail::flight_file().flush();
  return true;
}

#if defined(RPL_ENABLE_LORA) && RPL_ENABLE_LORA
inline bool append_flight_bin(const uint8_t* pkt48, uint8_t lora_tx_done,
                              uint8_t lora_ack_ok) {
  File& f = detail::flight_file();
  if (!f || pkt48 == nullptr) {
    return false;
  }
  if (f.write(pkt48, flight_packet::kEncodedSize) != flight_packet::kEncodedSize) {
    return false;
  }
  const uint8_t meta[2] = {lora_tx_done, lora_ack_ok};
  if (f.write(meta, sizeof(meta)) != sizeof(meta)) {
    return false;
  }
  f.flush();
  return true;
}
#else
inline bool append_flight_bin(const uint8_t* data, size_t len) {
  File& f = detail::flight_file();
  if (!f || data == nullptr || len == 0) {
    return false;
  }
  if (f.write(data, len) != len) {
    return false;
  }
  f.flush();
  return true;
}
#endif
#endif

#if !(defined(RPL_SD_BINARY) && RPL_SD_BINARY)
inline bool append_flight_row(unsigned long t, const flight::ImuSample& imu,
                              const flight::BmpSample& bmp) {
#if defined(RPL_COMPACT_CSV) && RPL_COMPACT_CSV
  static char line[160];
  int pos = snprintf(line, sizeof(line), "%lu,", t);
  if (pos < 0 || static_cast<size_t>(pos) >= sizeof(line)) {
    return false;
  }
  detail::add_float_field(line, &pos, sizeof(line), imu.euler_h, 2, ',');
  detail::add_float_field(line, &pos, sizeof(line), imu.euler_r, 2, ',');
  detail::add_float_field(line, &pos, sizeof(line), imu.euler_p, 2, ',');
  detail::add_float_field(line, &pos, sizeof(line), imu.lin_az, 3, ',');
  detail::add_float_field(line, &pos, sizeof(line), imu.gyr_z, 3, ',');
  const int n = snprintf(line + pos, sizeof(line) - static_cast<size_t>(pos), "%u,",
                         bmp.ok ? 1u : 0u);
  if (n > 0) {
    pos += n;
  }
  if (bmp.ok) {
    detail::add_float_field(line, &pos, sizeof(line), bmp.temp_c, 2, ',');
    detail::add_float_field(line, &pos, sizeof(line), bmp.pressure_hpa, 2, ',');
    detail::add_float_field(line, &pos, sizeof(line), bmp.altitude_m, 1, '\n');
  } else {
    snprintf(line + pos, sizeof(line) - static_cast<size_t>(pos), ",,\n");
  }
  return detail::flight_buffer_push_line(line, strlen(line));
#else
  static char line[272];
  int pos = snprintf(
      line, sizeof(line), "%lu,%u,%u,%u,%u,", t, static_cast<unsigned>(imu.cal_sys),
      static_cast<unsigned>(imu.cal_gyro), static_cast<unsigned>(imu.cal_accel),
      static_cast<unsigned>(imu.cal_mag));
  if (pos < 0 || static_cast<size_t>(pos) >= sizeof(line)) {
    return false;
  }

  detail::add_float_field(line, &pos, sizeof(line), imu.euler_h, 2, ',');
  detail::add_float_field(line, &pos, sizeof(line), imu.euler_r, 2, ',');
  detail::add_float_field(line, &pos, sizeof(line), imu.euler_p, 2, ',');
  detail::add_float_field(line, &pos, sizeof(line), imu.acc_gx, 3, ',');
  detail::add_float_field(line, &pos, sizeof(line), imu.acc_gy, 3, ',');
  detail::add_float_field(line, &pos, sizeof(line), imu.acc_gz, 3, ',');
  detail::add_float_field(line, &pos, sizeof(line), imu.lin_ax, 3, ',');
  detail::add_float_field(line, &pos, sizeof(line), imu.lin_ay, 3, ',');
  detail::add_float_field(line, &pos, sizeof(line), imu.lin_az, 3, ',');
  detail::add_float_field(line, &pos, sizeof(line), imu.gyr_x, 3, ',');
  detail::add_float_field(line, &pos, sizeof(line), imu.gyr_y, 3, ',');
  detail::add_float_field(line, &pos, sizeof(line), imu.gyr_z, 3, ',');
  detail::add_float_field(line, &pos, sizeof(line), imu.mag_x, 3, ',');
  detail::add_float_field(line, &pos, sizeof(line), imu.mag_y, 3, ',');
  detail::add_float_field(line, &pos, sizeof(line), imu.mag_z, 3, ',');

  const int n = snprintf(line + pos, sizeof(line) - static_cast<size_t>(pos), "%u,",
                         bmp.ok ? 1u : 0u);
  if (n > 0) {
    pos += n;
  }
  if (bmp.ok) {
    detail::add_float_field(line, &pos, sizeof(line), bmp.temp_c, 2, ',');
    detail::add_float_field(line, &pos, sizeof(line), bmp.pressure_hpa, 2, ',');
    detail::add_float_field(line, &pos, sizeof(line), bmp.altitude_m, 1, '\n');
  } else {
    snprintf(line + pos, sizeof(line) - static_cast<size_t>(pos), ",,\n");
  }

  return detail::flight_buffer_push_line(line, strlen(line));
#endif
}
#endif  // !(RPL_SD_BINARY)

inline void flight_log_flush() { detail::flight_flush_to_card(); }

inline bool clear_flight_log() { return remove_file(kFlightCsv); }

}  // namespace sd
