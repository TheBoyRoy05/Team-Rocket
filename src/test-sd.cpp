#include "sd.hpp"

void setup() {
  sd::setup_32u4();
  sd::begin();
  sd::dump_file_to_serial(sd::kFlightCsv);
}

void loop() {}
