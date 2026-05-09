// Feather 32u4 Adalogger flight stack: BNO055 + SD (+ optional BMP390 +
// LoRa). Use PlatformIO env `main-lora` for radio firmware (BMP omitted for
// flash). USB / LoRa telemetry lines match the CSV row format (see sd.hpp).

#include <Arduino.h>

#include "imu.hpp"
#include "sd.hpp"

#if !RPL_NO_BMP
#include "bmp390.hpp"
#endif

#if RPL_ENABLE_LORA
#include <SPI.h>
#include <RH_RF95.h>
#include "radio.hpp"
#endif

#ifndef RPL_TELEMETRY_MIRROR
#define RPL_TELEMETRY_MIRROR 1
#endif

#if RPL_ENABLE_LORA
// Adafruit Feather 32u4 + RFM9x wiring (same as RadioHead examples).
constexpr uint8_t kRfm95Cs = 8;
constexpr uint8_t kRfm95Int = 7;
constexpr uint8_t kRfm95Rst = 4;
constexpr float kRf95Mhz = 915.0f;

RH_RF95 g_rf95(kRfm95Cs, kRfm95Int);
#endif

#if RPL_TELEMETRY_MIRROR
inline Print* telemetry_mirror() {
#if RPL_ENABLE_LORA
  return &radio::log();
#else
  return &Serial;
#endif
}
#else
inline Print* telemetry_mirror() { return nullptr; }
#endif

#if !RPL_NO_BMP
inline void sample_baro(flight::BmpSample& out) { bmp390::sample_for_flight(out); }
#else
inline void sample_baro(flight::BmpSample&) {}
#endif

void setup() {
#if RPL_ENABLE_LORA
  pinMode(kRfm95Rst, OUTPUT);
  digitalWrite(kRfm95Rst, HIGH);
#endif

  sd::flight_log_setup();

  imu::try_init();
#if !RPL_NO_BMP
  bmp390::setup();
#endif

#if RPL_ENABLE_LORA
  digitalWrite(kRfm95Rst, LOW);
  delay(10);
  digitalWrite(kRfm95Rst, HIGH);
  delay(10);

  if (!g_rf95.init()) {
    Serial.println(F("LoRa init failed"));
    while (true) {
      delay(1000);
    }
  }
  if (!g_rf95.setFrequency(kRf95Mhz)) {
    Serial.println(F("LoRa setFrequency failed"));
    while (true) {
      delay(1000);
    }
  }
  g_rf95.setModemConfig(RH_RF95::Bw125Cr45Sf128);
  g_rf95.setPreambleLength(8);
  g_rf95.spiWrite(RH_RF95_REG_39_SYNC_WORD, 0x12);
  g_rf95.setTxPower(23, false);
  radio::begin(Serial, g_rf95, RADIO_LOG_MAX_PACKET);
#endif
}

void loop() {
  static flight::ImuSample imu{};
  static flight::BmpSample bmp{};

  imu::try_init();

  const unsigned long t = millis();
  if (imu::ready()) {
    imu::sample_for_flight(imu);
    sample_baro(bmp);

    sd::append_flight_row(t, imu, bmp, telemetry_mirror());
  }
  sd::flight_log_flush();

#if RPL_ENABLE_LORA
  radio::poll();
#endif

  delay(100);
}
