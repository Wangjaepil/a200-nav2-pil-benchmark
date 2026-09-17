#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

CORE="${ROOT}/src/dynamic_scan_filter_core.cpp"
UNIT="${ROOT}/test/test_dynamic_scan_filter_core_standalone.cpp"
STRESS="${ROOT}/test/test_motion_gate_stress.cpp"
INCLUDE="-I${ROOT}/include"
WARNINGS=(-Wall -Wextra -Wpedantic -Werror)

g++ -std=c++17 "${WARNINGS[@]}" "${INCLUDE}" \
  "${CORE}" "${UNIT}" -O2 \
  -o /tmp/a200_dynamic_scan_filter_v04_test
/tmp/a200_dynamic_scan_filter_v04_test

g++ -std=c++17 "${WARNINGS[@]}" "${INCLUDE}" \
  "${CORE}" "${STRESS}" -O2 \
  -o /tmp/a200_dynamic_scan_filter_v04_stress
/tmp/a200_dynamic_scan_filter_v04_stress

# Run the deterministic unit suite again with UB/ASan.
g++ -std=c++17 "${WARNINGS[@]}" "${INCLUDE}" \
  "${CORE}" "${UNIT}" -O1 -g -fno-omit-frame-pointer \
  -fsanitize=address,undefined \
  -o /tmp/a200_dynamic_scan_filter_v04_test_san
ASAN_OPTIONS=detect_leaks=0 /tmp/a200_dynamic_scan_filter_v04_test_san

echo "DYNAMIC_SCAN_FILTER_V04_SANITIZER_OK"
