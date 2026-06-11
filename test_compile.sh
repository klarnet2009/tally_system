#!/bin/bash
set -e
g++ -fsyntax-only -I. -I./lib/E28_SX1280 -I./lib/TallyProtocol tally_slave/src/main.cpp lib/E28_SX1280/E28_SX1280.cpp lib/TallyProtocol/TallyProtocol.cpp
