#include "bmp390.hpp"
#include "imu.hpp"
#include "sd.hpp"

void setup() {
  sd::setup_32u4();

  // sd::flight_log_setup();
  // imu::setup();
  // bmp390::setup();
  
  sd::begin();
  sd::dump_file_to_serial(sd::kFlightCsv);
}

void loop() {
  // flight::ImuSample imu_sample{};
  // flight::BmpSample bmp_sample{};
  // imu::sample_for_flight(imu_sample);
  // bmp390::sample_for_flight(bmp_sample);
  // sd::append_flight_row(millis(), imu_sample, bmp_sample);
  // sd::flight_log_flush();
  // delay(100);
}
