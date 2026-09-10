#!/usr/bin/env python3
"""Authorized live Serial stress: queries + state log + optional small motor commands.

Uses existing configuration/limits, never saves or calibrates. Leaves the drive off.
"""
import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import select
import struct
import termios
import time
import tty

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--port', required=True)
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--motion', action='store_true', help='Enable bounded motor commands')
parser.add_argument('--rate', type=int, default=20, help='Parameter requests per second (1..100)')
args = parser.parse_args()
assert 1 <= args.rate <= 100
fd = os.open(args.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
tty.setraw(fd)
attrs = termios.tcgetattr(fd)
attrs[2] |= termios.CLOCAL | termios.CREAD
attrs[2] &= ~termios.HUPCL
attrs[4] = attrs[5] = termios.B115200
termios.tcsetattr(fd, termios.TCSANOW, attrs)
fcntl.ioctl(fd, termios.TIOCMBIS, struct.pack('I', termios.TIOCM_DTR | termios.TIOCM_RTS))
buffer = b''
report = {'passed': False, 'commands': [], 'states': []}


def read_lines(timeout):
    global buffer
    if select.select([fd], [], [], max(0, timeout))[0]:
        buffer += os.read(fd, 4096).replace(b'\r', b'\n')
    lines = []
    while b'\n' in buffer:
        line, buffer = buffer.split(b'\n', 1)
        if not line:
            continue
        text = line.decode('ascii')
        if text.startswith('state: '):
            values = list(map(float, text[7:].split()))
            assert len(values) == 3 and all(math.isfinite(v) for v in values), text
            report['states'].append([time.monotonic(), *values])
        else:
            lines.append(text)
    return lines


def command(text, expected):
    start = time.monotonic()
    os.write(fd, (text + '\r\n').encode())
    while time.monotonic() - start < .25:
        for reply in read_lines(.01):
            assert not reply.startswith('ERROR'), (text, reply)
            if reply.startswith(expected):
                latency = time.monotonic() - start
                report['commands'].append({'command': text, 'reply': reply, 'latency_s': latency})
                return reply
    raise AssertionError(f'Timeout: {text}')


try:
    termios.tcflush(fd, termios.TCIFLUSH)
    command('log_off', 'OK: log_off')
    command('STOP', 'OK: STOP')
    command('is_on:0', 'OK: is_on:0')
    if args.motion:
        command('is_on:1', 'OK: is_on:1')
    command('log_on', 'OK: log_on')
    modes = ['servo_cmd: 0 0', 'servo_cmd: 0 0.05', 'servo_cmd: 0 -0.05',
             'mit_cmd: 0 0.05 0 0 2', 'position_step', 'servo_cmd: 3 0', 'servo_cmd: 1 0']
    for mode in modes:
        if mode == 'position_step':
            assert report['states']
            mode = f"servo_cmd: 2 {report['states'][-1][1] + .03:.6f}"
        if args.motion:
            command(mode, 'OK: ' + mode.split(':')[0])
        phase_start = time.monotonic()
        for i in range(3 * args.rate):
            command('firmware_rev:?', 'firmware_rev:')
            deadline = phase_start + (i + 1) / args.rate
            while time.monotonic() < deadline:
                read_lines(min(.01, deadline - time.monotonic()))
        print(mode, 3 * args.rate, 'replies, states:', len(report['states']), flush=True)
        command('STOP', 'OK: STOP')
    assert len(report['states']) >= 1900, len(report['states'])
    intervals = [b[0] - a[0] for a, b in zip(report['states'], report['states'][1:])]
    assert max(intervals) < .05, max(intervals)
    report['max_state_gap_s'] = max(intervals)
    report['max_response_s'] = max(c['latency_s'] for c in report['commands'])
    report['passed'] = True
finally:
    try:
        command('STOP', 'OK: STOP')
        command('log_off', 'OK: log_off')
        command('is_on:0', 'OK: is_on:0')
        assert command('is_on:?', 'is_on:') == 'is_on:0'
    except Exception as exc:
        report['cleanup_error'] = str(exc)
        report['passed'] = False
        raise
    finally:
        os.close(fd)
        args.output.write_text(json.dumps(report, indent=2) + '\n')
print('PASS', report['max_response_s'], report['max_state_gap_s'])
