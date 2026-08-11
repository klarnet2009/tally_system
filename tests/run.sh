#!/bin/bash
# Host-side logic tests. No hardware, no PlatformIO — these run anywhere g++ does.
#
# They cover the units where a bug is INVISIBLE by inspection: wire bit-packing,
# and every rule that is a function of millis() (link supervision, burst
# schedule, listen-before-talk, channel-escape policy). On real hardware those
# timing rules can only be exercised by waiting in real time, which is why they
# went untested for so long — and why the audit found four of its five defects in
# the escape policy, which had no test at all until it was extracted here.
#
# These also replaced the old root Arduino.h/SPI.h syntax-check stubs: this suite
# compiles the same translation units for real, so the stubs were retired.
set -e
cd "$(dirname "$0")/.."
CXX=${CXX:-g++}
FLAGS="-std=c++17 -Wall -Wextra -Werror -Itests -Ilib/TallyProtocol -Ilib/E28_SX1280"
OUT=$(mktemp -d)
trap 'rm -rf "$OUT"' EXIT

fail=0
build_run() {
  name=$1; shift
  $CXX $FLAGS -o "$OUT/$name" tests/shim.cpp "$@"
  "$OUT/$name" || fail=1
  echo
}

build_run t_protocol tests/test_protocol.cpp lib/TallyProtocol/TallyProtocol.cpp
build_run t_link     tests/test_link.cpp     lib/TallyProtocol/TallyProtocol.cpp lib/TallyProtocol/TallyLink.cpp
build_run t_burst    tests/test_burst.cpp
build_run t_escape   tests/test_escape.cpp

if [ $fail -ne 0 ]; then
  echo "TESTS FAILED"
  exit 1
fi
echo "all host tests passed"
