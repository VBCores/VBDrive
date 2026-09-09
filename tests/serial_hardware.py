#!/usr/bin/env python3
"""Explicit live Serial test; stages config and tests SAVE/APPLY with unchanged values.

No movement target is sent. Firmware reboot and driver enable/disable ARE performed.
Use only with an authorized, secured VBDrive. Writes a transcript and JSON report.
"""
import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import re
import select
import struct
import termios
import time
import tty

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--port', required=True)
parser.add_argument('--revision', required=True)
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
schema = re.findall(r'\{ParameterId::\w+,\s*"([^"]+)",\s*ParameterType::(\w+),\s*(true|false),\s*(true|false)',
                    (Path(__file__).resolve().parents[1] / 'App/parameters.hpp').read_text())
assert len(schema) == 33
schema = [entry for entry in schema if entry[0] not in ('bootloader', 'cmd_errors')]
assert len(schema) == 31
fd = os.open(args.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
tty.setraw(fd)
attrs = termios.tcgetattr(fd)
attrs[2] |= termios.CLOCAL | termios.CREAD
attrs[2] &= ~termios.HUPCL
attrs[4] = attrs[5] = termios.B19200
termios.tcsetattr(fd, termios.TCSANOW, attrs)
fcntl.ioctl(fd, termios.TIOCMBIS, struct.pack('I', termios.TIOCM_DTR | termios.TIOCM_RTS))
transcript = []
passed = False
in_config = False
in_test = False

def command(text, duration=.25):
    termios.tcflush(fd, termios.TCIFLUSH)
    os.write(fd, (text+'\r\n').encode())
    deadline = time.monotonic()+duration
    data = b''
    while time.monotonic() < deadline:
        if select.select([fd], [], [], max(0, deadline-time.monotonic()))[0]:
            data += os.read(fd, 4096)
    reply = data.decode(errors='replace').replace('\x00', '').strip()
    transcript.append({'command': text, 'response': reply})
    print(text, repr(reply), flush=True)
    return reply

def read(name):
    reply = command(name+':?')
    assert reply.startswith(name+':'), reply
    return reply.split(':', 1)[1].strip()

def same(a, b):
    if a == b: return True
    try: return math.isnan(float(a)) and math.isnan(float(b))
    except ValueError: return False

try:
    assert read('firmware_rev') == args.revision
    assert read('vbdrive_model') == 'M4310'
    original = {name: read(name) for name, *_ in schema}
    for name, _, mutable, _ in schema:
        if mutable == 'false':
            assert 'Read-only' in command(name+':1')
    for name in ('not_a_param', 'state.is_on', 'limit.speed', 'gear_suffix'):
        assert 'Unknown parameter' in command(name+':?')
    assert 'CONFIG mode required' in command('kp:5')
    assert 'Invalid value' in command('is_on:2')
    assert 'OK' in command('is_on:0')
    assert read('is_on') == '0'
    assert 'OK' in command('is_on:1')
    assert read('is_on') == '1'

    assert 'ENABLED' in command('CONFIG'); in_config = True
    for name, kind, _, persistent in schema:
        if persistent == 'true':
            value = '-1' if kind == 'INTEGER32' else '1'
            assert 'OK' in command(name+':'+value)
            assert float(read(name)) == float(value)
    assert 'ENABLED' in command('CONFIG')
    assert float(read('gear')) == 1
    assert 'Invalid value' in command('gear:256')
    assert 'Invalid value' in command('kp:bad')
    assert 'Invalid value' in command('ang_dir:0')
    assert 'Read-only' in command('firmware_rev:bad')
    assert read('firmware_rev') == args.revision
    assert 'DISCARDED' in command('EXIT'); in_config = False
    for name, _, _, persistent in schema:
        if persistent == 'true': assert same(read(name), original[name])
    command('CONFIG'); in_config = True
    assert 'default' in command('RESET')
    assert 'DISCARDED' in command('EXIT'); in_config = False
    assert read('gear') == original['gear'] and read('node_id') == original['node_id']

    # Exercise actual EEPROM SAVE + reboot without changing the user's value.
    command('CONFIG'); in_config = True
    assert 'OK' in command('gear:'+original['gear'])
    assert 'Invalid value' in command('gear:bad')
    assert 'Saved config' in command('SAVE'); in_config = False
    command('APPLY', 3)
    assert read('firmware_rev') == args.revision
    for name, _, _, persistent in schema:
        if persistent == 'true': assert same(read(name), original[name])

    assert 'Entering TEST' in command('TEST'); in_test = True
    assert read('firmware_rev') == args.revision
    command('log_on', .4)
    assert 'rotor:' in command('firmware_rev:?', .4)
    command('log_off')
    assert 'No effort' in command('do_free')
    assert 'Stopping TEST' in command('STOP'); in_test = False
    for name in ('bootloader', 'cmd_errors'):
        assert 'Unknown parameter' in command(name+':?')
    passed = True
finally:
    if in_config: command('EXIT')
    if in_test: command('STOP')
    os.close(fd)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps({'passed': passed, 'transcript': transcript}, indent=2)+'\n')
print('PASS: live Serial catalog, readonly, CONFIG/EXIT/RESET/SAVE/APPLY, TEST/logging/free/STOP')
