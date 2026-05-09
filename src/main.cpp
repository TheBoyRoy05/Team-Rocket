#include "imu.hpp"
#include "sd.hpp"
#if !RPL_NO_BMP
#include "bmp390.hpp"
#endif
#if ((defined(RPL_ENABLE_LORA) && RPL_ENABLE_LORA) || \
     (defined(RPL_SD_BINARY) && RPL_SD_BINARY))
#include "flight_packet.hpp"
#endif
#if defined(RPL_ENABLE_LORA) && RPL_ENABLE_LORA
#ifndef RPL_LORA_ACK
#define RPL_LORA_ACK 0
#endif
#ifndef LORA_ACK_TIMEOUT_MS
#define LORA_ACK_TIMEOUT_MS 400
#endif
#include <SPI.h>
#if defined(RPL_RF95_MINIMAL) && RPL_RF95_MINIMAL
#include "rf95_tx_min.hpp"
#else
#include <RH_RF95.h>
#endif
#endif

#ifndef LORA_CS
#define LORA_CS 5
#endif
#ifndef LORA_RST
#define LORA_RST 6
#endif
#ifndef LORA_IRQ
#define LORA_IRQ 0
#endif
#ifndef RF95_FREQ_MHZ
#define RF95_FREQ_MHZ 915.0f
#endif
#ifndef LORA_TX_WAIT_SENT_MS
#define LORA_TX_WAIT_SENT_MS 15000
#endif

#ifndef RPL_LOG_EVENTS
#define RPL_LOG_EVENTS 1
#endif

#if RPL_LOG_EVENTS
/** USB: one line when any BNO055 cal 0–3 changes: leading c then sys,gyro,accel,mag as digits. */
static void log_cal_if_changed(const flight::ImuSample& imu) {
  static uint8_t ps = 0xFF;
  static uint8_t pg = 0xFF;
  static uint8_t pa = 0xFF;
  static uint8_t pm = 0xFF;
  if (imu.cal_sys == ps && imu.cal_gyro == pg && imu.cal_accel == pa &&
      imu.cal_mag == pm) {
    return;
  }
  ps = imu.cal_sys;
  pg = imu.cal_gyro;
  pa = imu.cal_accel;
  pm = imu.cal_mag;
  Serial.write('c');
  Serial.write(static_cast<char>('0' + (ps > 3 ? 3 : ps)));
  Serial.write(static_cast<char>('0' + (pg > 3 ? 3 : pg)));
  Serial.write(static_cast<char>('0' + (pa > 3 ? 3 : pa)));
  Serial.write(static_cast<char>('0' + (pm > 3 ? 3 : pm)));
  Serial.write('\n');
}
#endif

#ifndef RPL_LOG_ACK_USB
#define RPL_LOG_ACK_USB 0
#endif

#if defined(RPL_ENABLE_LORA) && RPL_ENABLE_LORA
#if defined(RPL_RF95_MINIMAL) && RPL_RF95_MINIMAL
rpl_rf95::TxMin g_rf95(LORA_CS);
#else
RH_RF95 g_rf95(LORA_CS, LORA_IRQ);
#endif
#endif

#if defined(RPL_ENABLE_LORA) && RPL_ENABLE_LORA
#if !(defined(RPL_RF95_MINIMAL) && RPL_RF95_MINIMAL)
static bool rh_receive_ack(RH_RF95& rf, uint32_t expect_t_ms, uint16_t timeout_ms) {
  const uint32_t t0 = millis();
  while (millis() - t0 < timeout_ms) {
    if (rf.available()) {
      uint8_t buf[12];
      uint8_t len = sizeof(buf);
      if (rf.recv(buf, &len)) {
        uint32_t got_ms = 0;
        if (flight_packet::decode_ack_payload(buf, len, &got_ms) &&
            got_ms == expect_t_ms) {
          return true;
        }
      }
    }
    delay(1);
  }
  return false;
}
#endif
#endif

static void debug_print_sample(uint32_t t, const flight::ImuSample& imu,
                               const flight::BmpSample& bmp) {
#ifdef RPL_DEBUG_SERIAL
  Serial.print(F("dbg "));
  Serial.print(t);
  Serial.print(F(" euler "));
  Serial.print(imu.euler_h, 2);
  Serial.print(F(","));
  Serial.print(imu.euler_r, 2);
  Serial.print(F(","));
  Serial.print(imu.euler_p, 2);
  Serial.print(F(" bmp_ok="));
  Serial.print(bmp.ok);
  if (bmp.ok) {
    Serial.print(F(" T="));
    Serial.print(bmp.temp_c, 2);
    Serial.print(F(" P="));
    Serial.print(bmp.pressure_hpa, 2);
    Serial.print(F(" alt="));
    Serial.print(bmp.altitude_m, 1);
  }
  Serial.println();
#else
  (void)t;
  (void)imu;
  (void)bmp;
#endif
}

void setup() {
  Serial.begin(115200);
  {
    const unsigned long t0 = millis();
    while (!Serial && (millis() - t0) < 5000) {
      delay(10);
    }
  }

  imu::setup();
#if !RPL_NO_BMP
  bmp390::setup();
#endif

  if (!sd::
#if defined(RPL_SD_BINARY) && RPL_SD_BINARY
           flight_log_setup_bin()) {
#else
           flight_log_setup()) {
#endif
    Serial.println(F("main: sd flight_log_setup failed"));
  }

#if defined(RPL_ENABLE_LORA) && RPL_ENABLE_LORA
#if !(defined(RPL_RF95_MINIMAL) && RPL_RF95_MINIMAL) && defined(__AVR_ATmega32U4__)
  if (digitalPinToInterrupt(LORA_IRQ) == NOT_AN_INTERRUPT) {
    Serial.println(F("CONFIG: LORA_IRQ must be D0,D1,D2,D3, or D7 on 32u4."));
    while (true) {
      delay(1000);
    }
  }
#endif
  pinMode(LORA_RST, OUTPUT);
  digitalWrite(LORA_RST, HIGH);
  delay(10);
  digitalWrite(LORA_RST, LOW);
  delay(10);
  digitalWrite(LORA_RST, HIGH);
  delay(10);

#if defined(RPL_RF95_MINIMAL) && RPL_RF95_MINIMAL
  if (!g_rf95.begin()) {
    Serial.println(F("LoRa radio init failed"));
    while (true) {
      delay(1000);
    }
  }
  g_rf95.setFrequencyMhz(RF95_FREQ_MHZ);
  g_rf95.setSyncWord(0x12);
  g_rf95.setTxPowerDbm(23);
#else
  if (!g_rf95.init()) {
    Serial.println(F("LoRa radio init failed"));
    while (true) {
      delay(1000);
    }
  }
  if (!g_rf95.setFrequency(RF95_FREQ_MHZ)) {
    Serial.println(F("LoRa setFrequency failed"));
    while (true) {
      delay(1000);
    }
  }
  g_rf95.setModemConfig(RH_RF95::Bw125Cr45Sf128);
  g_rf95.setPreambleLength(8);
  g_rf95.spiWrite(RH_RF95_REG_39_SYNC_WORD, 0x12);
  g_rf95.setTxPower(23, false);
#endif
#endif
}

void loop() {
  flight::ImuSample imu{};
  flight::BmpSample bmp{};

  imu::sample_for_flight(imu);
#if RPL_LOG_EVENTS
  log_cal_if_changed(imu);
#endif
#if !RPL_NO_BMP
  bmp390::sample_for_flight(bmp);
#else
  bmp.ok = false;
#endif

  const uint32_t t = millis();

#if defined(RPL_SD_BINARY) && RPL_SD_BINARY
  uint8_t pkt[flight_packet::kEncodedSize];
  const size_t n = flight_packet::encode(pkt, sizeof(pkt), t, imu, bmp);
  uint8_t lora_tx_ok = 0;
  uint8_t lora_ack_ok = 0;
#if defined(RPL_ENABLE_LORA) && RPL_ENABLE_LORA
  if (n > 0U && g_rf95.send(pkt, static_cast<uint8_t>(n))) {
#if defined(RPL_RF95_MINIMAL) && RPL_RF95_MINIMAL
    lora_tx_ok =
        g_rf95.waitPacketSent(static_cast<uint16_t>(LORA_TX_WAIT_SENT_MS)) ? 1 : 0;
#if RPL_LORA_ACK
    if (lora_tx_ok) {
      lora_ack_ok = g_rf95.receiveAck(t, static_cast<uint16_t>(LORA_ACK_TIMEOUT_MS)) ? 1
                                                                                        : 0;
    }
#endif
#else
    lora_tx_ok = g_rf95.waitPacketSent(LORA_TX_WAIT_SENT_MS) ? 1 : 0;
#if RPL_LORA_ACK
    if (lora_tx_ok) {
      lora_ack_ok =
          rh_receive_ack(g_rf95, t, static_cast<uint16_t>(LORA_ACK_TIMEOUT_MS)) ? 1 : 0;
    }
#endif
#endif
#if (RPL_LOG_EVENTS || RPL_LOG_ACK_USB) && RPL_LORA_ACK
    if (lora_ack_ok) {
      Serial.println(F("ack"));
    }
#endif
  }
#endif
  if (n > 0U) {
#if defined(RPL_ENABLE_LORA) && RPL_ENABLE_LORA
    sd::append_flight_bin(pkt, lora_tx_ok, lora_ack_ok);
#else
    sd::append_flight_bin(pkt, n);
#endif
  }
#else
  sd::append_flight_row(t, imu, bmp);
#endif
  sd::flight_log_flush();

#if defined(RPL_ENABLE_LORA) && RPL_ENABLE_LORA && !(defined(RPL_SD_BINARY) && RPL_SD_BINARY)
  uint8_t pkt[flight_packet::kEncodedSize];
  const size_t n = flight_packet::encode(pkt, sizeof(pkt), t, imu, bmp);
  if (n > 0U && g_rf95.send(pkt, static_cast<uint8_t>(n))) {
#if defined(RPL_RF95_MINIMAL) && RPL_RF95_MINIMAL
    g_rf95.waitPacketSent(static_cast<uint16_t>(LORA_TX_WAIT_SENT_MS));
#else
    g_rf95.waitPacketSent(LORA_TX_WAIT_SENT_MS);
#endif
  }
#endif

  debug_print_sample(t, imu, bmp);

  delay(100);
}
