from enum import Enum, auto
from threading import Condition
import json
import socketserver


BLUE = (255, 0, 0)
GREEN = (0, 255, 0)
RED = (0, 0, 255)


def normalize_azimuth(azimuth):
    """Normalize azimuth to (-180, 180]."""
    if azimuth <= -180:
        return azimuth + 360
    elif azimuth > 180:
        return azimuth - 360
    else:
        return azimuth


class ThreadingTCPRequestHandler(socketserver.BaseRequestHandler):
    def handle(self):
        robot = self.server.robot
        buffer = b''
        while True:
            while b'\r' not in buffer:
                data = self.request.recv(1024)
                if not data:
                    print(f"\nConnection closed by {self.client_address}.")
                    robot.track_info.finish()
                    robot.pot_info.finish()
                    robot.concentration_info.finish()
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
                    print("Bad format.")
                    continue
                if result["type"] == "tracks":
                    info = result["src"]
                    robot.track_info.update(info)
                    if robot.debug_type == Debug.OTHER:
                        robot.visualization.mainWindow.updateSoundSphere(info)
                elif result["type"] == "pots":
                    robot.pot_info.update(result)  # TODO
                elif result["type"] == "concentration":
                    robot.concentration_info.update(result)
                    print(result)
                    if robot.debug_type == Debug.OTHER:
                        robot.visualization.mainWindow.updateCharts(result)
                elif result["type"] == "motion":
                    robot.motion_info.update(result)
                    if robot.debug_type == Debug.OTHER:
                        robot.visualization.mainWindow.updateCompass(result)
                else:
                    print("Bad format.")
                    return


class ThreadingTCPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    def __init__(self, robot, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.robot = robot


class SharedInfo:
    def __init__(self):
        self.info: dict = None
        self.lock = Condition()
        self.updated = False
        self.finished = False

    def update(self, info):
        with self.lock:
            self.info = info
            self.updated = True
            self.lock.notify()

    def get(self):
        with self.lock:
            if self.finished:
                raise Exception("SharedInfo is finished.")
            self.lock.wait_for(self.is_updated)
            self.updated = False
            return self.info

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
