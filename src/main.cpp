#include "bmp390.hpp"
#include "flight-log.hpp"
#include "imu.hpp"
#include "read-data.hpp"

void setup() {
  // flight_log::setup();
  // imu::setup();
  // bmp390::setup();
  read_data::setup();
}

void loop() {
  // flight::ImuSample imu_sample{};
  // flight::BmpSample bmp_sample{};
  // imu::sample_for_flight(imu_sample);
  // bmp390::sample_for_flight(bmp_sample);
  // flight_log::append_row(millis(), imu_sample, bmp_sample);
  // flight_log::flush();
  // delay(100);
}
