#include "imu.hpp"
#include "bmp390.hpp"

void setup() {
  imu::setup();
  bmp390::setup();
}

void loop() {
  imu::loop();
  bmp390::loop();
}
