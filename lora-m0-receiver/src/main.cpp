// Feather9x_RX — RadioHead RH_RF95 receiver for Feather M0 RFM9x.
// Decodes full TR v1 telemetry (see src/flight_packet.hpp): prints CSV + RSSI on Serial,
// optionally appends the same rows to lora_rx.csv when SD is enabled (see platformio).
//
// LoRa params match TX: SF7, BW 125 kHz, CR 4/5, preamble 8, sync 0x12, 915 MHz.

#include <SPI.h>
#include <RH_RF95.h>

#include "flight_packet.hpp"

#ifndef RPL_RX_SD_CS
#define RPL_RX_SD_CS (-1)
#endif

#if RPL_RX_SD_CS >= 0
#include <SD.h>
#endif

#if defined(__AVR_ATmega32U4__)
#define RFM95_CS 8
#define RFM95_INT 7
#define RFM95_RST 4

#elif defined(ADAFRUIT_FEATHER_M0) || defined(ADAFRUIT_FEATHER_M0_EXPRESS) || defined(ARDUINO_SAMD_FEATHER_M0)
#define RFM95_CS 8
#define RFM95_INT 3
#define RFM95_RST 4

#elif defined(ARDUINO_ADAFRUIT_FEATHER_RP2040_RFM)
#define RFM95_CS 16
#define RFM95_INT 21
#define RFM95_RST 17

#elif defined(__AVR_ATmega328P__)
#define RFM95_CS 4
#define RFM95_INT 3
#define RFM95_RST 2

#elif defined(ESP8266)
#define RFM95_CS 2
#define RFM95_INT 15
#define RFM95_RST 16

#elif defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2) || defined(ARDUINO_NRF52840_FEATHER) || defined(ARDUINO_NRF52840_FEATHER_SENSE)
#define RFM95_CS 10
#define RFM95_INT 9
#define RFM95_RST 11

#elif defined(ESP32)
#define RFM95_CS 33
#define RFM95_INT 27
#define RFM95_RST 13

#elif defined(ARDUINO_NRF52832_FEATHER)
#define RFM95_CS 11
#define RFM95_INT 31
#define RFM95_RST 7

#else
#error "Board not recognized — set RFM95_CS, RFM95_INT, RFM95_RST for your wiring."
#endif

#define RF95_FREQ 915.0

RH_RF95 rf95(RFM95_CS, RFM95_INT);

#if RPL_RX_SD_CS >= 0
static File rx_log;
static bool rx_sd_ok = false;
#endif

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(115200);
#if defined(USBCON)
  unsigned long t = millis();
  while (!Serial && millis() - t < 4000) {}
#endif
  delay(100);

  Serial.println(F("LoRa RX: TR telemetry -> USB CSV (+ RSSI col); SD if built with RPL_RX_SD_CS"));

#if RPL_RX_SD_CS >= 0
  if (SD.begin(RPL_RX_SD_CS)) {
    rx_log = SD.open("lora_rx.csv", FILE_WRITE);
    if (rx_log) {
      const uint32_t sz = rx_log.size();
      (void)rx_log.seek(sz);
      if (sz == 0U) {
        flight_packet::print_tr_csv_header(rx_log, true);
        rx_log.flush();
      }
      rx_sd_ok = true;
      Serial.println(F("SD: append lora_rx.csv"));
    }
  }
  if (!rx_sd_ok) {
    Serial.println(F("SD: disabled (fail or no card)"));
  }
#endif

  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println(F("LoRa radio init failed"));
    while (1) {
      delay(1000);
    }
  }
  Serial.println(F("LoRa radio init OK"));

  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println(F("setFrequency failed"));
    while (1) {
      delay(1000);
    }
  }
  Serial.print(F("Freq MHz "));
  Serial.println(RF95_FREQ);

  rf95.setModemConfig(RH_RF95::Bw125Cr45Sf128);
  rf95.setPreambleLength(8);
  rf95.spiWrite(RH_RF95_REG_39_SYNC_WORD, 0x12);
  rf95.setTxPower(23, false);

  flight_packet::print_tr_csv_header(Serial, true);
}

void loop() {
  if (!rf95.available()) {
    return;
  }

  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  if (!rf95.recv(buf, &len)) {
    Serial.println(F("Receive failed"));
    return;
  }

  digitalWrite(LED_BUILTIN, HIGH);
  const int16_t rssi = static_cast<int16_t>(rf95.lastRssi());

  if (len >= flight_packet::kEncodedSize && buf[0] == flight_packet::kMagic0 &&
      buf[1] == flight_packet::kMagic1 && buf[2] == flight_packet::kVersion) {
    const uint32_t t_tx = flight_packet::read_t_ms_from_tr_payload(buf, len);
    uint8_t apl[6];
    if (flight_packet::encode_ack_payload(apl, sizeof(apl), t_tx) == 6) {
      (void)rf95.send(apl, sizeof(apl));
      (void)rf95.waitPacketSent(3000);
    }
  }

  const bool decoded =
      flight_packet::try_print_decoded_csv(Serial, buf, len, &rssi);

#if RPL_RX_SD_CS >= 0
  if (rx_sd_ok && decoded) {
    (void)flight_packet::try_print_decoded_csv(rx_log, buf, len, &rssi);
    rx_log.flush();
  }
#endif

  if (!decoded) {
    RH_RF95::printBuffer("Received: ", buf, len);
    if (len < sizeof(buf)) {
      buf[len] = 0;
    }
    Serial.print(F("Got: "));
    Serial.println(reinterpret_cast<char*>(buf));
    Serial.print(F("RSSI "));
    Serial.println(rssi, DEC);
  }

  digitalWrite(LED_BUILTIN, LOW);
}
