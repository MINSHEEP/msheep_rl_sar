#!/usr/bin/env bash
# Copyright (c) 2024-2026 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cmake -S "$project_dir/src/rl_sar" -B "$project_dir/cmake_build" \
  -DUSE_CMAKE=ON -DBUILD_DIAN_ONLY=ON -DCMAKE_BUILD_TYPE=Release
cmake --build "$project_dir/cmake_build" --target rl_real_dian test_dian test_dian_imu test_dian_227 test_dian_gamepad test_dian_fsm test_dian_keyboard --parallel "${DIAN_BUILD_JOBS:-2}"
ctest --test-dir "$project_dir/cmake_build" --output-on-failure
