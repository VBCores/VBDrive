#!/usr/bin/env python3
"""Send bounded 1 kHz Cyphal Servo or MIT velocity commands to node 1 on vcan1.0."""

import argparse
import json
import math
import socket
import statistics
import struct
import time


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--count', type=int, default=3000)
parser.add_argument('--amplitude', type=float, default=0.0, help='1 Hz sine amplitude in rad/s or rad')
parser.add_argument('--mode', choices=('servo', 'mit'), default='servo')
parser.add_argument('--control', choices=('velocity', 'position'), default='velocity')
args = parser.parse_args()
if not 1 <= args.count <= 10000 or not 0.0 <= args.amplitude <= 0.05:
    parser.error('count must be 1..10000 and amplitude 0..0.05')
if args.mode == 'mit' and args.control != 'velocity':
    parser.error('MIT position control is not supported by this test')

# libcanard message CAN ID: nominal priority, message flags, subject + node 1, source 126.
subject = 3408 if args.mode == 'servo' else 2108
can_id = 0x80000000 | (4 << 26) | (0x6000 | subject) << 8 | 126


def make_frame(index, value):
    tail = 0xE0 | (index & 31)
    if args.mode == 'servo':
        payload = struct.pack('<BfB', 2 if args.control == 'position' else 0, value, tail)
    else:
        # MIT: torque, position, velocity, position gain, velocity gain;
        # CAN FD pads the 20-byte DSDL payload to 24 bytes before the tail.
        payload = struct.pack('<fffff3xB', 0, 0, value, 0, 30, tail)
    return struct.pack('=IBBBB64s', can_id, len(payload), 1, 0, 0, payload)


sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FD_FRAMES, 1)
sock.bind(('vcan1.0',))
center = 0.0
if args.control == 'position':
    sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, struct.pack('=II', 0x106EE301, 0x1FFFFFFF))
    sock.settimeout(1.0)
    frame = sock.recv(72)
    center = struct.unpack_from('<f', frame, 8 + 7)[0]
sent_ns = []
period_ns = 1_000_000
start_ns = time.monotonic_ns() + 100_000_000

try:
    for index in range(args.count):
        target_ns = start_ns + index * period_ns
        remaining_ns = target_ns - time.monotonic_ns()
        if remaining_ns > 0:
            time.sleep(remaining_ns / 1e9)
        value = center + args.amplitude * math.sin(2.0 * math.pi * index / 1000.0)
        sock.send(make_frame(index, value))
        sent_ns.append(time.monotonic_ns())
finally:
    for offset in range(3):
        sock.send(make_frame(len(sent_ns) + offset, center))
        time.sleep(0.001)
    sock.close()

intervals_ms = [(right - left) / 1e6 for left, right in zip(sent_ns, sent_ns[1:])]
print(json.dumps({
    'sent': len(sent_ns),
    'amplitude': args.amplitude,
    'mode': args.mode,
    'control': args.control,
    'center_rad': center if args.control == 'position' else None,
    'elapsed_s': (sent_ns[-1] - sent_ns[0]) / 1e9 if len(sent_ns) > 1 else 0.0,
    'interval_median_ms': statistics.median(intervals_ms) if intervals_ms else None,
    'interval_p95_ms': sorted(intervals_ms)[int(0.95 * len(intervals_ms))] if intervals_ms else None,
    'interval_max_ms': max(intervals_ms) if intervals_ms else None,
}))
