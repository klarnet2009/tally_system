#!/bin/bash
set -e
g++ -fsyntax-only -I. -I./lib/E28_SX1280 -I./lib/TallyProtocol lib/E28_SX1280/E28_SX1280.cpp lib/TallyProtocol/TallyProtocol.cpp lib/TallyProtocol/TallyLink.cpp
