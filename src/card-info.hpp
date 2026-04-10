#pragma once

/*
  SD card test (header-only)

  Based on the Arduino SD "CardInfo" example: utility APIs to print SD/FAT
  details and list files. Useful when you are not sure the card is working.

  CS pin: 4 matches Feather M0 Adalogger onboard SD; use 10 for many
  Adalogger FeatherWing stacks on a plain Feather.
*/

#include <Arduino.h>
#include <SD.h>
#include <SPI.h>

namespace card_info {

constexpr int kChipSelect = 4;

inline void setup() {
  Sd2Card card;
  SdVolume volume;
  SdFile root;

  Serial.begin(9600);
  while (!Serial) {
  }

  Serial.print(F("\nInitializing SD card..."));

  if (!card.init(SPI_HALF_SPEED, kChipSelect)) {
    Serial.println(F("initialization failed. Things to check:"));
    Serial.println(F("* is a card inserted?"));
    Serial.println(F("* is your wiring correct?"));
    Serial.println(
        F("* did you change the chipSelect pin to match your shield or module?"));
    while (true) {
    }
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
    while (true) {
    }
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
}

inline void loop() {}

}  // namespace card_info
