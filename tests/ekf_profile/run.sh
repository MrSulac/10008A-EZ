#!/bin/sh
set -eu
EKF_TEST_ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
EKF_TEST_RUN_DIR=$(mktemp -d "${TMPDIR:-/tmp}/ekf-profile-test.XXXXXX")
trap 'rm -rf -- "$EKF_TEST_RUN_DIR"' EXIT
set -- -std=c++17 -O2 -Wall -Wextra
if [ "${SANITIZE:-0}" = 1 ]; then
    set -- "$@" -g -fsanitize=address,undefined -fno-omit-frame-pointer
fi
"${CXX:-c++}" "$@" -I"$EKF_TEST_ROOT/tests/ekf_profile/stubs" -I"$EKF_TEST_ROOT/include" \
    "$EKF_TEST_ROOT/tests/ekf_profile/host_test.cpp" -o "$EKF_TEST_RUN_DIR/host_test"
"$EKF_TEST_RUN_DIR/host_test"
