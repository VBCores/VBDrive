#!/usr/bin/env python3
"""Send bounded 1 kHz Cyphal Servo or MIT commands to node 1 on vcan1.0."""

import argparse
import json
import math
import socket
import statistics
import struct
import time


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--count', type=int, default=3000)
parser.add_argument('--amplitude', type=float, default=0.0, help='1 Hz sine amplitude in rad, rad/s, Nm or V')
parser.add_argument('--mode', choices=('servo', 'mit'), default='servo')
parser.add_argument('--control', choices=('velocity', 'position', 'torque', 'voltage'), default='velocity')
parser.add_argument('--servo-type', type=int, help='Servo control_type (default: direct mode for --control)')
parser.add_argument('--realtime', action='store_true', help='Pin CPU 1, FIFO priority 20 and lock memory; requires root')
parser.add_argument('--constant', action='store_true', help='Send the same nonzero target on every tick')
args = parser.parse_args()
if not 1 <= args.count <= 60000 or not 0.0 <= args.amplitude <= 0.05:
    parser.error('count must be 1..60000 and amplitude 0..0.05')
if args.mode == 'mit' and args.control == 'voltage':
    parser.error('MIT has no voltage control')
if args.servo_type is not None and (args.mode != 'servo' or args.servo_type not in range(7)):
    parser.error('servo-type must be 0..6 in Servo mode')
if args.servo_type in (0, 1) and args.control != 'velocity':
    parser.error('velocity Servo types require --control velocity')
if args.servo_type in (3, 4, 5) and args.control != 'position':
    parser.error('position Servo types require --control position')
if args.servo_type == 2 and args.control != 'torque':
    parser.error('torque Servo type requires --control torque')
if args.servo_type == 6 and args.control != 'voltage':
    parser.error('voltage Servo type requires --control voltage')

# libcanard message CAN ID: nominal priority, message flags, subject + node 1, source 126.
subject = 3408 if args.mode == 'servo' else 2108
can_id = 0x80000000 | (4 << 26) | (0x6000 | subject) << 8 | 126


def make_frame(index, value):
    tail = 0xE0 | (index & 31)
    if args.mode == 'servo':
        direct_types = {'velocity': 0, 'torque': 2, 'position': 3, 'voltage': 6}
        control_type = args.servo_type if args.servo_type is not None else direct_types[args.control]
        payload = struct.pack('<BfBB', control_type, value, 0, tail)
    else:
        # MIT: torque, position, velocity, position gain, velocity gain;
        # CAN FD pads the 20-byte DSDL payload to 24 bytes before the tail.
        if args.control == 'position':
            payload = struct.pack('<fffff3xB', 0, value, 0, 150, 10, tail)
        elif args.control == 'torque':
            payload = struct.pack('<fffff3xB', value, 0, 0, 0, 0, tail)
        else:
            payload = struct.pack('<fffff3xB', 0, 0, value, 0, 1, tail)
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
realtime = {}
if args.realtime:
    import ctypes
    import gc
    os = __import__('os')
    affinity = sorted(os.sched_getaffinity(0))
    os.sched_setaffinity(0, {affinity[min(1, len(affinity)-1)]})
    os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(20))
    libc = ctypes.CDLL(None, use_errno=True)
    if libc.mlockall(3) != 0:
        raise OSError(ctypes.get_errno(), 'mlockall failed')
    gc.disable()
    realtime = {'cpu': list(os.sched_getaffinity(0)), 'priority': 20, 'memory_locked': True}
sent_ns = []
period_ns = 1_000_000
start_ns = time.monotonic_ns() + 100_000_000

try:
    for index in range(args.count):
        target_ns = start_ns + index * period_ns
        remaining_ns = target_ns - time.monotonic_ns()
        if remaining_ns > 0:
            time.sleep(remaining_ns / 1e9)
        value = center + (args.amplitude if args.constant else
                          args.amplitude * math.sin(2.0 * math.pi * index / 1000.0))
        sock.send(make_frame(index, value))
        sent_ns.append(time.monotonic_ns())
finally:
    for offset in range(3):
        sock.send(make_frame(len(sent_ns) + offset, center))
        time.sleep(0.001)
    sock.close()

intervals_ms = [(right - left) / 1e6 for left, right in zip(sent_ns, sent_ns[1:])]
print(json.dumps({
    'realtime': realtime,
    'sent': len(sent_ns),
    'amplitude': args.amplitude,
    'mode': args.mode,
    'control': args.control,
    'center_rad': center if args.control == 'position' else None,
    'elapsed_s': (sent_ns[-1] - sent_ns[0]) / 1e9 if len(sent_ns) > 1 else 0.0,
    'interval_median_ms': statistics.median(intervals_ms) if intervals_ms else None,
    'interval_p98_ms': sorted(intervals_ms)[int(0.98 * len(intervals_ms))] if intervals_ms else None,
    'interval_p95_ms': sorted(intervals_ms)[int(0.95 * len(intervals_ms))] if intervals_ms else None,
    'interval_max_ms': max(intervals_ms) if intervals_ms else None,
}))
