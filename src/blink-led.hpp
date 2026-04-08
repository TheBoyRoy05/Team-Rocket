#pragma once

#include <Arduino.h>

namespace blink_led {

inline void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
}

inline void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  Serial.println("LED on");
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
  Serial.println("LED off");
}

}
