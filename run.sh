#!/bin/bash
# Usage: source ./run.sh   (from repo root) to define piorun in the current shell.
# Or copy the function into ~/.bashrc for every new terminal.

piorun() {
  pio run -e "$1" -t upload -t monitor
}

# Decode flight.bin → data/flight.csv. Default: SD volume (Windows often mounts as D:).
# Override: flight_csv /e/flight.bin  or  export FLIGHT_BIN_DEFAULT=/e/flight.bin
flight-csv() {
  local bin="${1:-${FLIGHT_BIN_DEFAULT:-D:/FLIGHT.BIN}}"
  python scripts/flight_bin_to_csv.py "$bin" > data/flight.csv
}