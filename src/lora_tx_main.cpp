/**
 * Feather 32u4 + RFM95 — LoRa TX bench test (RadioHead RH_RF95).
 * Flash with env feather32u4_lora_tx (excludes src/main.cpp).
 *
 * Pin defaults match Adafruit Feather9x_TX / Feather9x_RX:
 *   Feather 32u4 w/ radio: CS=8, RST=4, DIO0/INT=7
 *   Feather 32u4 w/ Radio Wing: CS=10, RST=11, INT=2 (SDA) — use build_flags / env feather32u4_lora_tx_wing
 *
 * Feather 32u4 interrupt pins: RadioHead needs DIO0 on a pin that supports attachInterrupt. The
 * Arduino core only allows **D0, D1, D2, D3, and D7** on this chip — **not** D5, D6, D8, D9, D10,
 * etc. Many Feather boards **do not break out D7** on the headers (you may see 5, 6, 9, 10…); in
 * that case use **D2 (SDA)** or **D3 (SCL)** for DIO0 (same as the LoRa Radio Wing), or **D0/D1**
 * if free (**D0** is Serial1 RX — avoid `Serial1` on that pin). Set `LORA_IRQ` to match.
 * Wire RFM DIO0 to that pad — software cannot fix a DIO0→D9/D10 PCB for RadioHead.
 * See: https://learn.adafruit.com/adafruit-feather-32u4-radio-with-rfm69-hcw-radio-module/
 *
 * Radio params match lora-m0-receiver (M0 RX): 915 MHz, Bw125Cr45Sf128, preamble 8, sync 0x12.
 *
 * USB: setup waits briefly for Serial so early lines are less likely to drop. If the host never
 * opens the COM port (or DTR is odd), an infinite wait would look like a dead monitor — see
 * LORA_SERIAL_WAIT_MS. For no wait at all: -D LORA_TX_HEADLESS
 */
#include <Arduino.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <string.h>

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
/** Max ms to wait for TxDone IRQ inside waitPacketSent (0 = library default, can hang forever). */
#ifndef LORA_TX_WAIT_SENT_MS
#define LORA_TX_WAIT_SENT_MS 15000
#endif
/** Max ms to wait for USB Serial in setup (0 = wait forever). Use 20000–30000 on fussy Windows CDC. */
#ifndef LORA_SERIAL_WAIT_MS
#define LORA_SERIAL_WAIT_MS 30000
#endif

RH_RF95 rf95(LORA_CS, LORA_IRQ);

// RH_RF95_MAX_MESSAGE_LEN is ~251 B — too large for stack on ATmega32u4; keep reply buffer static/small.
static uint8_t s_replyBuf[80];

void setup() {
  pinMode(LORA_RST, OUTPUT);
  digitalWrite(LORA_RST, HIGH);

  Serial.begin(115200);
#if defined(USBCON) && !defined(LORA_TX_HEADLESS)
  if (LORA_SERIAL_WAIT_MS > 0) {
    const unsigned long deadline = millis() + static_cast<unsigned long>(LORA_SERIAL_WAIT_MS);
    while (!Serial && static_cast<long>(deadline - millis()) > 0) {
      delay(10);
    }
  } else {
    while (!Serial) {
      delay(10);
    }
  }
#endif

  delay(100);
  Serial.println(F("Feather LoRa TX (RadioHead)"));
  Serial.flush();

#if defined(__AVR_ATmega32U4__)
  if (digitalPinToInterrupt(LORA_IRQ) == NOT_AN_INTERRUPT) {
    Serial.println(F("CONFIG ERROR: LORA_IRQ (DIO0) cannot use this pin on Feather 32u4."));
    Serial.println(F("attachInterrupt is only valid on digital pins 0, 1, 2, 3, and 7."));
    Serial.print(F("You set LORA_IRQ="));
    Serial.print(LORA_IRQ);
    Serial.println(F(" — use D0, D1, D2, D3, or D7 for DIO0; D5/6/8/9/10 cannot attachInterrupt on 32u4."));
    while (true) {
      delay(1000);
    }
  }
#endif

  digitalWrite(LORA_RST, LOW);
  delay(10);
  digitalWrite(LORA_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println(F("LoRa radio init failed"));
    Serial.println(F("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info"));
    Serial.print(F("Pins: CS="));
    Serial.print(LORA_CS);
    Serial.print(F(" RST="));
    Serial.print(LORA_RST);
    Serial.print(F(" DIO0="));
    Serial.println(LORA_IRQ);
    Serial.println(F("Wing / no D7 header: env ..._irq0 (DIO0→D0), ..._irq2 (→D2), ..._irq3 (→D3)"));
    while (true) { delay(1000); }
  }
  Serial.println(F("LoRa radio init OK!"));

  if (!rf95.setFrequency(RF95_FREQ_MHZ)) {
    Serial.println(F("setFrequency failed"));
    while (true) { delay(1000); }
  }
  Serial.print(F("Set Freq to: "));
  Serial.println(RF95_FREQ_MHZ);

  rf95.setModemConfig(RH_RF95::Bw125Cr45Sf128);
  rf95.setPreambleLength(8);
  rf95.spiWrite(RH_RF95_REG_39_SYNC_WORD, 0x12);
  rf95.setTxPower(23, false);

  Serial.println(F("32u4 RadioHead TX ready — sending once per second."));
}

void loop() {
  static uint32_t seq;

  delay(1000);

  char radiopacket[24];
  snprintf(radiopacket, sizeof(radiopacket), "ping %lu", static_cast<unsigned long>(seq++));
  const uint8_t len = static_cast<uint8_t>(strlen(radiopacket));

  Serial.print(F("Sending "));
  Serial.println(radiopacket);
  Serial.flush();

  if (!rf95.send(reinterpret_cast<const uint8_t*>(radiopacket), len)) {
    Serial.println(F("send() failed (e.g. CAD busy); skipping this cycle."));
    return;
  }
  if (!rf95.waitPacketSent(LORA_TX_WAIT_SENT_MS)) {
    Serial.println(F("waitPacketSent timeout — no TxDone from radio (check DIO0 → IRQ pin, wiring)."));
    return;
  }

  uint8_t replyLen = sizeof(s_replyBuf);
  Serial.println(F("Waiting for reply..."));
  if (rf95.waitAvailableTimeout(1000)) {
    if (rf95.recv(s_replyBuf, &replyLen)) {
      if (replyLen < sizeof(s_replyBuf)) {
        s_replyBuf[replyLen] = 0;
      }
      Serial.print(F("Got reply: "));
      Serial.println(reinterpret_cast<char*>(s_replyBuf));
      Serial.print(F("RSSI: "));
      Serial.println(rf95.lastRssi(), DEC);
    } else {
      Serial.println(F("Receive failed"));
    }
  } else {
    Serial.println(F("No reply, is there a listener around?"));
  }
}
