#!/bin/bash
# ponytail: this script, test_compile_hub.sh and the root Arduino.h/SPI.h stubs
# exist only for manual g++ syntax checks — delete all four once confirmed
# nobody runs them by hand (pio run is the real check)
set -e
g++ -fsyntax-only -I. -I./lib/E28_SX1280 -I./lib/TallyProtocol tally_slave/src/main.cpp lib/E28_SX1280/E28_SX1280.cpp lib/TallyProtocol/TallyProtocol.cpp lib/TallyProtocol/TallyLink.cpp
