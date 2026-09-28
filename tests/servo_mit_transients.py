#!/usr/bin/env python3
"""Record a bounded VBDrive 01 position or velocity step and Cyphal State stream."""

import argparse
import fcntl
import json
import os
import re
import select
import statistics
import struct
import subprocess
import termios
import threading
import time
import tty
from pathlib import Path


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--port', default='/dev/cu.usbmodem1203')
parser.add_argument('--host', default='igor@172.16.125.130')
parser.add_argument('--mode', required=True, choices=['mit', 'servo'])
parser.add_argument('--control', choices=['position', 'velocity'], default='position')
parser.add_argument('--kp', type=float, default=20.0, help='Position gain')
parser.add_argument('--kd', type=float, default=1.0, help='Position damping or velocity gain')
parser.add_argument('--step', type=float, default=0.04, help='Position (rad) or velocity (rad/s) step')
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()

samples = []
collector = subprocess.Popen(
    ['ssh', '-o', 'BatchMode=yes', args.host,
     'stdbuf -oL candump -L vcan1.0,106EE301:1FFFFFFF'],
    stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1,
)
frame_pattern = re.compile(r'^\((\d+\.\d+)\) vcan1\.0 106EE301##[0-9A-Fa-f]([0-9A-Fa-f]+)$')


def collect():
    """Decode VBDrive 01 State frames: uint56 timestamp and three float32 fields."""
    for line in collector.stdout:
        match = frame_pattern.match(line.strip())
        if not match:
            continue
        data = bytes.fromhex(match.group(2))
        if len(data) < 20:
            continue
        angle, velocity, torque = struct.unpack_from('<fff', data, 7)
        samples.append([int.from_bytes(data[:7], 'little'), angle, velocity, torque])


thread = threading.Thread(target=collect, daemon=True)
thread.start()
fd = None
identity_verified = False
motor_may_be_on = False
base = None
failure = None
commands = []
events = {}
try:
    fd = os.open(args.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    tty.setraw(fd)
    attrs = termios.tcgetattr(fd)
    attrs[2] |= termios.CLOCAL | termios.CREAD
    attrs[2] &= ~termios.HUPCL
    attrs[4] = attrs[5] = termios.B115200
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    fcntl.ioctl(fd, termios.TIOCMBIS, struct.pack('I', termios.TIOCM_DTR | termios.TIOCM_RTS))
    termios.tcflush(fd, termios.TCIFLUSH)
    buffer = b''

    def command(text, expected=None):
        """Send one command and return the requested response line."""
        global buffer
        os.write(fd, (text + '\r\n').encode())
        prefix = expected or ((text.split(':', 1)[0] if text.startswith(('mit_cmd:', 'servo_cmd:'))
                               else text.split()[0]) + ' OK')
        deadline = time.monotonic() + 0.7
        while time.monotonic() < deadline:
            if select.select([fd], [], [], 0.02)[0]:
                buffer += os.read(fd, 4096).replace(b'\r', b'\n')
            while b'\n' in buffer:
                line, buffer = buffer.split(b'\n', 1)
                reply = line.decode('ascii', errors='replace')
                if ' ERROR:' in reply:
                    raise RuntimeError(f'{text}: {reply}')
                if reply.startswith(prefix):
                    commands.append([text, reply, len(samples)])
                    return reply
        raise TimeoutError(f'{text}: no success response')

    assert command('name:?', 'name:') == 'name:vbdrive01'
    assert command('node_id:?', 'node_id:') == 'node_id:1'
    assert command('gear:?', 'gear:') == 'gear:36'
    identity_verified = True
    command('STOP')
    state = command('is_on:?', 'is_on:')
    if state == 'is_on:1':
        motor_may_be_on = True
        command('is_on:0')
        motor_may_be_on = False
    elif state != 'is_on:0':
        raise RuntimeError(f'Unexpected motor state: {state}')
    command('log_off')
    deadline = time.monotonic() + 2.0
    while len(samples) < 100 and time.monotonic() < deadline:
        time.sleep(0.01)
    if len(samples) < 100:
        raise RuntimeError('No VBDrive 01 Cyphal State stream')
    def set_target(target):
        """Select MIT or Servo control with the requested position or velocity target."""
        if args.mode == 'mit':
            if args.control == 'position':
                command(f'mit_cmd: {target:.7f} 0 0 {args.kp:g} {args.kd:g}')
            else:
                command(f'mit_cmd: 0 {target:.7f} 0 0 {args.kd:g}')
        else:
            command(f'servo_cmd: {2 if args.control == "position" else 0} {target:.7f}')

    base = statistics.median(row[1] for row in samples[-100:]) if args.control == 'position' else 0.0
    set_target(base)
    time.sleep(0.1)
    motor_may_be_on = True
    command('is_on:1')
    time.sleep(0.35)
    events['step_index'] = len(samples)
    set_target(base + args.step)
    time.sleep(0.8)
    events['return_index'] = len(samples)
    set_target(base)
    time.sleep(0.8)
    command('STOP')
    command('is_on:0')
    motor_may_be_on = False
    events['end_index'] = len(samples)
except Exception as exc:
    failure = repr(exc)
    raise
finally:
    if fd is not None:
        try:
            if identity_verified:
                os.write(fd, b'STOP\r\n' + (b'is_on:0\r\n' if motor_may_be_on else b''))
                termios.tcdrain(fd)
                time.sleep(0.05)
        finally:
            os.close(fd)
    collector.terminate()
    try:
        collector.wait(timeout=2)
    except subprocess.TimeoutExpired:
        collector.kill()
        collector.wait()
    result = {'mode': args.mode, 'control': args.control, 'kp': args.kp, 'kd': args.kd,
              'step_rad': args.step, 'base_rad': base,
              'events': events, 'commands': commands, 'samples': samples,
              'failure': failure, 'physical_label': '01'}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result) + '\n')
    print('saved', args.output, 'samples', len(samples), 'failure', failure)
