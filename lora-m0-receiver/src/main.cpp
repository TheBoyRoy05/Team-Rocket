// Feather9x_RX — RadioHead RH_RF95 receiver for Feather M0 RFM9x.
// Based on Adafruit’s guide:
// https://learn.adafruit.com/adafruit-feather-m0-radio-with-lora-radio-module/using-the-rfm-9x-radio
//
// LoRa params match repo `src/lora_tx_main.cpp` (Feather 32u4 RadioHead TX): SF7, BW 125 kHz, CR 4/5,
// preamble 8, sync word 0x12, 915 MHz.

#include <SPI.h>
#include <RH_RF95.h>

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

  Serial.println(F("Feather LoRa RX (RadioHead)"));

  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println(F("LoRa radio init failed"));
    Serial.println(F("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info"));
    while (1) { delay(1000); }
  }
  Serial.println(F("LoRa radio init OK!"));

  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println(F("setFrequency failed"));
    while (1) { delay(1000); }
  }
  Serial.print(F("Set Freq to: "));
  Serial.println(RF95_FREQ);

  rf95.setModemConfig(RH_RF95::Bw125Cr45Sf128);
  rf95.setPreambleLength(8);
  rf95.spiWrite(RH_RF95_REG_39_SYNC_WORD, 0x12);

  rf95.setTxPower(23, false);
}

void loop() {
  if (rf95.available()) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len)) {
      digitalWrite(LED_BUILTIN, HIGH);
      RH_RF95::printBuffer("Received: ", buf, len);
      if (len < sizeof(buf)) {
        buf[len] = 0;
      }
      Serial.print(F("Got: "));
      Serial.println(reinterpret_cast<char*>(buf));
      Serial.print(F("RSSI: "));
      Serial.println(rf95.lastRssi(), DEC);

      uint8_t data[] = "And hello back to you";
      rf95.send(data, sizeof(data));
      rf95.waitPacketSent();
      Serial.println(F("Sent a reply"));
      digitalWrite(LED_BUILTIN, LOW);
    } else {
      Serial.println(F("Receive failed"));
    }
  }
}
