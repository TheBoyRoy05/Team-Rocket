#include "blink-led.hpp"
#include "write-data.hpp"
#include "card-info.hpp"

void setup() {
  card_info::setup();
  write_data::setup();
}

void loop() {
  write_data::loop();
}
