#!/usr/bin/env python3
from collections import namedtuple
from itertools import cycle
import math
import socket
import struct
import time

HOST = 'localhost'   # The remote host
PORT = 11000         # The same port as used by the server


# *   StandardMsgType 4 bytes
# *   CommType        4 bytes
# *   ReplyType       4 bytes
# *   robot_id            (industrial::shared_types::shared_int)    4  bytes
# *   sequence            (industrial::shared_types::shared_int)    4  bytes
# *   valid_fields        (industrial::shared_types::shared_int)    4  bytes
# *   time                (industrial::shared_types::shared_real)   4  bytes
# *   positions           (industrial::joint_data)                  40 bytes
# *   velocities          (industrial::joint_data)                  40 bytes
# *   accelerations       (industrial::joint_data)                  40 bytes
JointTrajPtFull = namedtuple('JointTrajPtFull', [
    'msg_type', 'comm_type', 'reply_type',
    'robot_id', 'sequence', 'valid_fields', 'time',
])


def pack_positions(positions):
    positions = [el for el in map(math.radians, positions)]
    # prefix length
    msg = struct.pack('<I', 148)
    msg += struct.pack('<iiiiiif', *JointTrajPtFull(
        msg_type=14, comm_type=2, reply_type=-1,
        robot_id=-1, sequence=0, valid_fields=2, time=-1,
    ))
    msg += struct.pack('<ffffffffff', *(positions + [0, 0, 0, 0]))
    msg += struct.pack('<ffffffffff', *([-1] * 10))
    msg += struct.pack('<ffffffffff', *([-1] * 10))
    return msg


def print_resp(resp):
    if not resp:
        return
    try:
        print(JointTrajPtFull(*struct.unpack(
            '<iiiiiiifffffffffffffffffffffffffffffff', resp
        )[1:8]))
    except Exception:
        print(len(resp), resp)
        raise


class Connection:

    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.conn = None

    def __enter__(self):
        self.conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.conn.connect((self.host, self.port))
        return self

    def __exit__(self, exc_type, exc_value, exc_tb):
        self.conn.close()

    def reconnect(self):
        self.conn.close()
        self.conn.connect((self.host, self.port))

    def communicate(self, msg, period):
        size = len(msg)
        self.conn.send(msg)
        resp = self.conn.recv(size)
        time.sleep(period)
        return resp

TRAJECTORY = [
    [0, 0, 0, 0, 0, 0],
    [-9.15, 3.85, 26.03, 13.88, 16.41, 8.15],
    [-2.15, 10.73, 29.56, 27.63, 16.41, 8.15],
    [4.61, 14.57, 34.47, 13.26, 29.03, -6.46],
    [12.66, 20.69, 21.97, -2.94, 33.16, 7.74],
    [10.11, 21.94, 25.5, -7.7, 21.65, -8.57],
    [13.01, 21.14, 30.44, 0.06, 25.45, -8.57],
    [10.2, 23.22, 45.16, 7.2, 25.45, -8.57],
    [10.2, 27.57, 48.38, 0, 79.96, 0],
    [0.33, 26.61, 65.57, 0, 90, 0],
    [-3.4, 17.73, 59.83, 0, 102.82, -4.42],
    [-25.93, 14.53, 28.44, -0.03, 113.64, -3.94],
    [-47.89, 11.02, 20.37, -0.03, 51.09, 0],
    [-38.23, 0, 7.02, -10.41, 15.16, 0],
]


def main(*args):
    rate = float(args[0]) if args else 10
    with Connection(HOST, PORT) as conn:
        for points in cycle(TRAJECTORY):
            try:
                resp = conn.communicate(pack_positions(points), 1 / rate)
            except IOError:
                # reconnects if the rate is too slow
                conn.reconnect()
                resp = conn.communicate(pack_positions(points), 1 / rate)
            print_resp(resp)

if __name__ == '__main__':
    import sys
    sys.exit(main(*sys.argv[1:]))
