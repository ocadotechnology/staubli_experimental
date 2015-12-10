#!/usr/bin/env python3
from collections import namedtuple
from itertools import cycle
import math
import socket
import struct
import sys
import threading
import time

HOST = 'localhost'   # The remote host
PORT = 11000         # The same port as used by the server
START = time.time()
TIMEOUT = None       # seconds or None
evt_listening  = threading.Event()

# JointTrajPtFull
# prefix
# *   Length          4 bytes
# header
# *   StandardMsgType 4 bytes
# *   CommType        4 bytes
# *   ReplyType       4 bytes
# body
# *   robot_id            (industrial::shared_types::shared_int)    4  bytes
# *   sequence            (industrial::shared_types::shared_int)    4  bytes
# *   valid_fields        (industrial::shared_types::shared_int)    4  bytes
# *   time                (industrial::shared_types::shared_real)   4  bytes
# *   positions           (industrial::joint_data)                  40 bytes
# *   velocities          (industrial::joint_data)                  40 bytes
# *   accelerations       (industrial::joint_data)                  40 bytes

JointTrajPtPart = namedtuple('JointTrajPtPart', [
    'msg_type', 'comm_type', 'reply_type',
    'robot_id', 'sequence', 'valid_fields', 'time',
])


def pack_positions(positions):
    positions = [el for el in map(math.radians, positions)]
    # prefix length
    msg = struct.pack('<I', 148)
    # header + body
    msg += struct.pack('<iiiiiif', *JointTrajPtPart(
        msg_type=14, comm_type=2, reply_type=-1,
        robot_id=0, sequence=0, valid_fields=0xF,
        time=time.time() - START,
    ))
    msg += struct.pack('<ffffffffff', *(positions + [0] * 4))
    # velocities
    msg += struct.pack('<ffffffffff', *([100] + [0] * 9))
    # accelerations
    msg += struct.pack('<ffffffffff', *([100, 100] + [0] * 8))
    return msg


def print_resp(resp):
    if not resp:
        print('No response.')
        return
    try:
        print(JointTrajPtPart(*struct.unpack(
            '<iiiiiiifffffffffffffffffffffffffffffff', resp
        )[1:8]))
    except Exception:
        print(len(resp), resp)
        raise


class Connection:

    def __init__(self, host, port, timeout=None):
        self.conn = None
        self._host = host
        self._port = port
        self._timeout = timeout

    def __enter__(self):
        self.conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.conn.connect((self._host, self._port))
        self.conn.settimeout(self._timeout)
        return self

    def __exit__(self, *exc_args):
        self.conn.close()

    def reconnect(self):
        self.__exit__()
        sys.stderr.write("Connection lost.\n")
        self.__enter__() 

    def communicate(self, msg):
        size = len(msg)
        self.conn.send(msg)

    def listen(self, interval):
        with self:
            evt_listening.set()
            while True:
                intervalcall(self.receive, interval)

    def receive(self):
        print("Listening: ", end='')
        try:
            resp = self.conn.recv(152)
        except IOError:
            self.reconnect()
            resp = self.conn.recv(152)
        print_resp(resp)


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


def intervalcall(func, interval):
    start = time.time()
    func()
    ttsleep = interval - (time.time() - start)
    if ttsleep > 0:
        time.sleep(ttsleep)
    else:
        sys.stderr.write(
            "{} exceeded the specified interval {:.2f}x times\n"
            .format(func.__name__, 1 - ttsleep / interval)
        )


def send_position(conn, points):
    try:
        resp = conn.communicate(pack_positions(points))
    except IOError:
        # reconnects if the rate is too slow
        conn.reconnect()
        resp = conn.communicate(pack_positions(points))


def main(*args):
    rate = float(args[0]) if args else 4
    conn_subsc = Connection(HOST, 11002)
    subscriber = threading.Thread(target=conn_subsc.listen, args=(1 / rate,))
    subscriber.start()
    evt_listening.wait()
    time.sleep(1.0)
    print("Started")
    with Connection(HOST, PORT, TIMEOUT) as conn:
        for points in cycle(TRAJECTORY):
            intervalcall(lambda: send_position(conn, points), 1 / rate)

if __name__ == '__main__':
    sys.exit(main(*sys.argv[1:]))