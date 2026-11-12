#!/usr/bin/env bash
# Copyright (c) 2024-2026 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec "$project_dir/cmake_build/bin/rl_real_dian" --mode keyboard "$@"
