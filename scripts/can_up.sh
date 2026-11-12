#!/usr/bin/env bash
# Copyright (c) 2024-2026 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail
# This robot's four USB adapters. Do not configure the onboard can4.
lock_fds=()
for i in 0 1 2 3; do
    iface="can$i"
    actual="$(basename "$(readlink -f "/sys/class/net/$iface/device")")"
    expected="1-1.4.$((i+1)):1.0"
    [[ "$actual" == "$expected" ]] || { echo "$iface USB mapping mismatch: $actual (expected $expected)" >&2; exit 1; }
    index="$(cat "/sys/class/net/$iface/ifindex")"
    exec {can_lock}>"/tmp/custom-quad-can-$index.lock"
    flock -n "$can_lock" || { echo "$iface is in use; stop the controller before configuring CAN" >&2; exit 1; }
    lock_fds+=("$can_lock")
done
for i in 0 1 2 3; do
    iface="can$i"
    detail="$(ip -details link show "$iface")"
    if [[ "$detail" == *'state UP'* && "$detail" == *'can state ERROR-ACTIVE'* && "$detail" == *'bitrate 1000000 '* ]]; then
        echo "$iface already UP, 1 Mbps"
        continue
    fi
    sudo ip link set "$iface" down
    sudo ip link set "$iface" type can bitrate 1000000
    sudo ip link set "$iface" txqueuelen 1024
    sudo ip link set "$iface" up
    echo "$iface configured: UP, 1 Mbps"
done
ip -details link show type can
