# Copyright (c) 2024-2026 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0
"""Exercise the real keyboard executable through a PTY, without CAN or IMU devices."""
import os
import pty
import select
import signal
import subprocess
import sys
import termios
import time

binary, policy_root, model = sys.argv[1:]


def session(mode):
    master, slave = pty.openpty()
    original = termios.tcgetattr(slave)
    process = subprocess.Popen([binary, '--policy-root', policy_root, '--mode', mode,
                                '--model', model, '--duration', '20'],
                               stdin=slave, stdout=slave, stderr=slave, start_new_session=True)
    return master, slave, original, process


def expect(master, text, timeout=6):
    output = b''
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if select.select([master], [], [], .1)[0]:
            output += os.read(master, 8192)
            if text.encode() in output:
                return
    raise AssertionError(f'Missing {text!r}: {output.decode(errors="replace")}')


master, slave, original, process = session('keyboard-mock')
try:
    expect(master, 'Keyboard:')
    os.write(master, b'm')
    expect(master, 'Motors enabled')
    os.write(master, b'0')
    expect(master, 'Standing ready')
    os.write(master, b'1')
    expect(master, '[DIAN FSM] RLFSMStateRLLocomotion')
    os.write(master, b'w')
    expect(master, 'vx=0.100')
    os.write(master, b' ')
    expect(master, 'vx=0.000')
    os.write(master, b'k')
    expect(master, '[DIAN FSM] RLFSMStatePassive')
    os.write(master, b'x')
    assert process.wait(timeout=3) == 0
    assert termios.tcgetattr(slave) == original, 'Terminal attributes were not restored'
finally:
    if process.poll() is None:
        process.kill()
        process.wait()
    os.close(master)
    os.close(slave)

for exit_signal in (signal.SIGINT, signal.SIGHUP):
    master, slave, original, process = session('keyboard-mock')
    try:
        expect(master, 'Keyboard:')
        os.write(master, b'm')
        expect(master, 'Motors enabled')
        process.send_signal(exit_signal)
        expect(master, '[DIAN FSM] RLFSMStateStop')
        assert process.wait(timeout=3) == 0
        assert termios.tcgetattr(slave) == original
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
        os.close(master)
        os.close(slave)

print('PASS: PTY enable/stand/RL/forward/zero/disable/exit; SIGINT/SIGHUP stop; terminal restored')
