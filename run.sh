#!/bin/bash
# Usage: source ./run.sh   (from repo root) to define piorun in the current shell.
# Or copy the function into ~/.bashrc for every new terminal.

piorun() {
  pio run -e "$1" -t upload -t monitor
}
