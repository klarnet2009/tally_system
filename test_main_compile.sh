#!/bin/bash
# NOTE: informational only — main_original.cpp includes NimBLEDevice.h, for which
# no stub header exists, so this check fails at the first include and the `| head`
# pipe masks the exit code. Kept for reference; do not gate CI on it.
g++ -fsyntax-only -I. -I./lib/E28_SX1280 -I./lib/TallyProtocol -I./tally_hub/src main_original.cpp 2>&1 | head -n 20
