#!/bin/bash
g++ -fsyntax-only -I. -I./tally_hub/lib/E28_SX1280 -I./tally_hub/lib/TallyProtocol -I./tally_hub/src main_original.cpp 2>&1 | head -n 20
