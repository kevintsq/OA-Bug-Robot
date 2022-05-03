import sys
from enum import Enum, auto
from threading import Condition, Thread, Event
import json
import socketserver
from struct import Struct

from serial import Serial


BLUE = (255, 0, 0)
GREEN = (0, 255, 0)
RED = (0, 0, 255)
RESOLUTION = 360 / 1147


def scan_index(azimuth):
    """0 <= azimuth < 360"""
    return int(azimuth / RESOLUTION)


def normalize_azimuth(azimuth):
    """Normalize azimuth to (-180, 180]."""
    if azimuth <= -180:
        return azimuth + 360
    elif azimuth > 180:
        return azimuth - 360
    else:
        return azimuth


class Frame(Struct):
    CHKSUM_BYTE = Struct(">B")

    def pack_with_chksum(self, *data):
        s = self.pack(*data)
        s += Frame.CHKSUM_BYTE.pack(sum(s) & 0xFF)
        return s


class Olfactory(Thread):
    def __init__(self, device, robot, *args, **kwargs):
        super().__init__(daemon=True, args=args, kwargs=kwargs)
        self.serial = Serial(device, 115200)
        self.stop_event = Event()
        self.robot = robot

    def __del__(self):
        # Thread has no __del__ method
        self.serial.close()

    def stop(self):
        self.stop_event.set()

    def run(self):
        self.stop_event.clear()
        buffer = b''
        while b'\r' not in buffer and not self.stop_event.is_set():
            buffer = self.serial.readline()
            if not buffer:
                raise Exception("Error reading data from serial port.")
        buffer = b''
        while not self.stop_event.is_set():
            while b'\r' not in buffer and not self.stop_event.is_set():
                data = self.serial.readline()
                if not data:
                    raise Exception("Error reading data from serial port.")
                buffer += data
            data_list = buffer.split(b'\r')
            # print(data_list)
            last_one = data_list[-1]
            if last_one == b'\n' or last_one == b'':
                buffer = b''
            else:
                buffer = last_one
            if len(data_list) > 1:
                try:
                    result = json.loads(data_list[-2])
                except:
                    print("Bad format.", file=sys.stderr)
                    continue
                self.robot.olfactory_info.update(result)


class ThreadingTCPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    def __init__(self, robot, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.robot = robot


class ThreadingTCPRequestHandler(socketserver.BaseRequestHandler):
    def handle(self):
        robot = self.server.robot
        buffer = b''
        while True:
            while b'\r' not in buffer:
                data = self.request.recv(1024)
                if not data:
                    print(f"\nConnection closed by {self.client_address}.")
                    robot.auditory_info.finish()
                    robot.olfactory_info.finish()
                    robot.motion_info.finish()
                    return
                buffer += data
            data_list = buffer.split(b'\r')
            last_one = data_list[-1]
            if last_one == b'\n' or last_one == b'':
                buffer = b''
            else:
                buffer = last_one
            if len(data_list) > 1:
                try:
                    result = json.loads(data_list[-2])
                except:
                    print("Bad format.", file=sys.stderr)
                    continue
                if result["type"] == "auditory":
                    robot.auditory_info.update(result)
                elif result["type"] == "olfactory":
                    robot.olfactory_info.update(result)
                elif result["type"] == "motion":
                    robot.motion_info.update(result)
                else:
                    print("Bad format.", file=sys.stderr)
                    return


class Shared:
    def __init__(self, initial=None, needs_update=False, on_update=None):
        self.instance = initial
        self.lock = Condition()
        self.needs_update = needs_update
        self.on_update = on_update
        self.updated = False
        self.finished = False

    def update(self, info):
        with self.lock:
            self.instance = info
            if self.on_update:
                self.on_update(info)
            self.updated = True
            self.lock.notify()

    def get(self):
        with self.lock:
            if self.finished:
                raise Exception("SharedInfo is finished.")
            if self.needs_update:
                self.lock.wait_for(self.is_updated)
            self.updated = False
            return self.instance

    def is_updated(self):
        return self.updated

    def finish(self):
        with self.lock:
            self.updated = True
            self.lock.notify()
            self.finished = True


class Debug(Enum):
    VISION = auto()
    OTHER = auto()


class Direction(Enum):
    FRONT = auto()
    BACK = auto()
    LEFT = auto()
    RIGHT = auto()
