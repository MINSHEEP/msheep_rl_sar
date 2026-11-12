#!/usr/bin/env bash
# Copyright (c) 2024-2026 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0
# Record one suspended-to-RL session. Extra arguments are passed to rl_real_dian.
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
binary="$project_dir/cmake_build/bin/rl_real_dian"
if [[ ! -x "$binary" ]]; then
  echo "未找到 $binary ，先运行 bash scripts/build_dian.sh" >&2
  exit 1
fi
stamp="$(date +%Y%m%d_%H%M%S)"
out="$project_dir/logs/dian_$stamp"
mkdir -p "$out"
{
  echo "time $(date -Is)"
  echo "host $(hostname 2>/dev/null || true)"
  uname -a || true
  echo "--- link ---"
  ip -br link 2>/dev/null || true
  echo "--- serial ---"
  ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true
  echo "--- model ---"
  model="$project_dir/policy/dian/student227/policy.onnx"
  if [[ -f "$model" ]]; then
    ls -l "$model" || true
    sha256sum "$model" || true
  else
    echo "missing $model"
  fi
} > "$out/environment.txt"
cp -a "$project_dir/policy/dian/hardware.yaml" "$out/hardware.yaml"
cp -a "$project_dir/policy/dian/base.yaml" "$out/base.yaml"
cp -a "$project_dir/policy/dian/student227/config.yaml" "$out/student227.yaml"
echo "录制目录 $out"
echo "悬空后按 m 使能，0 站立，1 切到 RL。结束按 x 或 Ctrl+C。"
echo "trace.csv 每个控制周期一行，退出原因在 stop.txt。"
exec "$binary" --mode keyboard --record "$out" "$@"
