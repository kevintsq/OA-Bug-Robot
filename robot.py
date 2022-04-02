#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# from time import sleep
# import math
import os
import subprocess

# import numpy as np
# import cv2 as cv
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QIcon
from PyQt5.QtWidgets import QApplication

from state import *
from utils import *
import views


HOST = "127.0.0.1"
PORT = 8080


class Robot(Thread):
    INFO_CMD_STRUCT = Frame(">BBBBB")
    SPEED_CONTROL_STRUCT = Frame(">BBBBfff")
    SPEED_INFO_STRUCT = Frame(">fff")
    BATTERY_INFO_STRUCT = Frame(">HHHB")

    class JustStartedState(AbstractState):
        def __init__(self, robot):
            super().__init__(robot)

        def transfer_when_colliding_wall(self):
            super().transfer_when_colliding_wall()
            robot: Robot = self.get_robot()
            raise NotImplemented  # TODO
            # action = robot.get_action_from_contour(robot.contours[0])
            # if action == robot.go_front:
            #     robot.state = robot.following_wall_state
            # else:
            #     robot.go_front()

        def transfer_when_not_following_wall(self):
            robot: Robot = self.get_robot()
            azimuth = robot.get_azimuths_from_sound().pop()
            current = robot.motion_info.get()["azimuth"]["yaw"]
            robot.turn_degree(normalize_azimuth(azimuth - current))

    class FollowingWallState(AbstractState):
        def __init__(self, robot):
            super().__init__(robot)

        def transfer_when_not_following_wall(self):
            robot: Robot = self.get_robot()
            if robot.is_revisiting_places():
                self.transfer_when_revisiting_places()
            robot.go_following_wall()

        def transfer_when_revisiting_places(self):
            super().transfer_when_revisiting_places()
            robot: Robot = self.get_robot()
            robot.state = robot.just_started_state

        def transfer_to_next_state(self):
            robot: Robot = self.get_robot()
            if robot.is_colliding_another_robot():
                self.transfer_when_colliding_another_robot()
            elif robot.is_visiting_turning_point():  # Needs Turning
                self.transfer_when_not_following_wall()
            else:
                robot.go_following_wall()

    def __init__(self, enable_vision=False, *args, **kwargs):
        super().__init__(daemon=True, *args, **kwargs)

        if os.path.exists('/dev/serial'):
            serial_devices = os.listdir('/dev/serial')
        else:
            serial_devices = []

        QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
        QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
        self.app = QApplication(sys.argv)
        self.app.setWindowIcon(QIcon("plane.svg"))
        self.app.beep()
        self.mainWindow = views.MainWindow()
        self.controlPanel = self.mainWindow.controlPanel

        if len(serial_devices) == 0:
            self.controlPanel.controllerButton.setEnabled(False)
            self.controlPanel.auditoryButton.setEnabled(False)
            self.controlPanel.olfactoryButton.setEnabled(False)
            self.controlPanel.motionButton.setEnabled(False)

        self.controlPanel.controllerDropdown.addItems(serial_devices)
        self.controlPanel.auditoryDropdown.addItems(serial_devices)
        self.controlPanel.olfactoryDropdown.addItems(serial_devices)
        self.controlPanel.motionDropdown.addItems(serial_devices)

        self.controlPanel.controllerButton.clicked.connect(self.setup_controller)
        self.controlPanel.auditoryButton.clicked.connect(self.setup_auditory)
        self.controlPanel.olfactoryButton.clicked.connect(self.setup_olfactory)
        self.controlPanel.motionButton.clicked.connect(self.setup_motion)

        self.controlPanel.goButton.clicked.connect(self.on_go_button_clicked)
        self.controlPanel.stopButton.clicked.connect(self.on_stop_button_clicked)

        self.stop_event = Event()
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_battery_info)  # must use QTimes to avoid QProgressBar segfault

        # these infos must be created before the server is ready
        self.auditory_info = SharedInfo()
        self.olfactory_info = SharedInfo()
        self.motion_info = SharedInfo()

        self.server = ThreadingTCPServer(self, (HOST, PORT), ThreadingTCPRequestHandler)
        self.server.allow_reuse_address = True
        # Start a thread with the server -- that thread will then start one more thread for each request
        server_thread = Thread(target=self.server.serve_forever, daemon=True)
        server_thread.start()
        print(f"Server loop running in thread: {server_thread.native_id}...")

        self.controller = None
        self.auditory_process = None
        self.olfactory_thread = None
        self.motion_process = None
        self.video_capture = None
        if enable_vision:
            self.setup_vision()
        # self.saw_img = None
        # self.contours = None
        self.turning_point = False

        self.just_started_state = self.JustStartedState(self)
        self.following_wall_state = self.FollowingWallState(self)
        self.state = self.just_started_state

        self.in_room = False
        self.mission_complete = False

        # GPIO.setmode(GPIO.BCM)
        # GPIO.setup(self.DIN1, GPIO.OUT)

    def __enter__(self):
        return self

    def __exit__(self, *args):
        print("Cleaning up...")
        if self.controller:
            self.stop()
            self.controller.close()
        # GPIO.cleanup()
        if self.motion_process:
            self.motion_process.terminate()
        if self.auditory_process:
            self.auditory_process.terminate()
        # no need to clean up olfactory
        if self.video_capture:
            raise NotImplemented  # TODO: Put the LiDAR cleanup code here.
            # self.video_capture.release()
            # cv.destroyAllWindows()
        self.server.shutdown()
        self.server.server_close()
        print("Done.")

    def update_battery_info(self):
        voltage, current, temperature, remaining = self.get_battery_info()
        views.mainWindow.batteryVoltage.setText(f"{voltage:.2f} V")
        views.mainWindow.batteryCurrent.setText(f"{current:.2f} A")
        views.mainWindow.batteryTemperature.setText(f"{temperature:.2f} ℃")
        views.mainWindow.batteryRemaining.setValue(remaining)

    def on_go_button_clicked(self):
        self.set_speed(views.mainWindow.xSpeedSpinBox.value(),
                       views.mainWindow.zSpeedSpinBox.value())

    def on_stop_button_clicked(self):
        self.stop()
        self.stop_event.set()
        self.controlPanel.startButton.setText("Start the Algorithm")
        self.controlPanel.setEnabled(True)

    def on_start_button_clicked(self):
        self.stop_event.clear()
        self.controlPanel.startButton.setText("Algorithm Started. Press Stop to Stop...")
        self.controlPanel.setEnabled(False)
        self.start()

    def setup_controller(self):
        if self.controller is None:
            self.controller = Serial(f"/dev/serial/{self.controlPanel.controllerDropdown.currentText()}", 230400)
            self.controlPanel.controllerButton.setText("Disconnect")
            self.timer.start(1000)
        else:
            self.timer.stop()
            self.controller.close()
            self.controller = None
            self.controlPanel.controllerButton.setText("Connect")

    def setup_auditory(self):
        if self.auditory_process is None:
            self.auditory_process = subprocess.Popen(("/home/pi/BluetoothAoaLocator/build/BluetoothAoaLocator",
                                                      "-s", HOST, str(PORT),
                                                      "-u", f"/dev/serial/{self.controlPanel.auditoryDropdown.currentText()}"))
            print(f"BluetoothAoaLocator client loop running in process: {self.auditory_process.pid}...")
            self.controlPanel.auditoryButton.setText("Disconnect")
        else:
            self.auditory_process.terminate()
            self.auditory_process = None
            self.controlPanel.auditoryButton.setText("Connect")

    def setup_olfactory(self):
        if self.olfactory_thread is None:
            self.olfactory_thread = Olfactory(f"/dev/serial/{self.controlPanel.olfactoryDropdown.currentText()}", self)
            self.olfactory_thread.start()
            print(f"Gas sensor client loop running in thread {self.olfactory_thread.native_id}...")
            self.controlPanel.olfactoryButton.setText("Disconnect")
        else:
            self.olfactory_thread.stop()
            self.olfactory_thread = None  # also releases serial
            self.controlPanel.olfactoryButton.setText("Connect")

    def setup_motion(self):
        if self.motion_process is None:
            self.motion_process = subprocess.Popen(("/home/pi/Robot/JY901S_socket", HOST, str(PORT),
                                                    f"/dev/serial/{self.controlPanel.motionDropdown.currentText()}"))
            print(f"Motion sensor client loop running in process: {self.motion_process.pid}...")
            self.controlPanel.motionButton.setText("Disconnect")
        else:
            self.motion_process.terminate()
            self.motion_process = None
            self.controlPanel.motionButton.setText("Connect")

    def turn_degree(self, deg):
        """degree < 0: right, > 0: left"""
        Kp = 40
        goal = normalize_azimuth(int(self.motion_info.get()["azimuth"]["yaw"]) + deg)
        while True:
            current = int(self.motion_info.get()["azimuth"]["yaw"])
            yaw_err = normalize_azimuth(goal - current)

            print(f"goal: {goal},\tcurrent: {current},\tyaw_err: {yaw_err}")
            if abs(yaw_err) < 2:
                self.stop()
                return

            yaw_rate = Kp * abs(yaw_err) // 10
            # 限幅，一个最大的角速度，一个最小的可以转动的
            yaw_rate = yaw_rate if yaw_rate < 40 else 40
            yaw_rate = yaw_rate if yaw_rate > 20 else 20

            if yaw_err > 0:
                self.turn_left(abs(yaw_rate) / 40)
            else:
                self.turn_right(abs(yaw_rate) / 40)

    def set_speed(self, x, z):
        cmd = Robot.SPEED_CONTROL_STRUCT.pack_with_chksum(0xFE, 0xEF, 0x0D, 0x01, x, 0, z)
        self.controller.write(cmd)
        self.controller.read_all()

    def get_speed(self):
        """:returns x_speed, y_speed, z_speed"""
        cmd = Robot.INFO_CMD_STRUCT.pack_with_chksum(0xFE, 0xEF, 0x01, 0x02)
        self.controller.get().write(cmd)
        result = self.controller.get().read(17)
        return Robot.SPEED_INFO_STRUCT.unpack(result[4:-1])

    def get_battery_info(self):
        """:returns voltage, current, temperature, remaining"""
        cmd = Robot.INFO_CMD_STRUCT.pack_with_chksum(0xFE, 0xEF, 0x01, 0x03)
        self.controller.get().write(cmd)
        result = self.controller.get().read(12)
        voltage, current, temperature, remaining = Robot.BATTERY_INFO_STRUCT.unpack(result[4:-1])
        return voltage / 100, current / 100, temperature / 10, remaining

    def go_front(self, speed=1):
        self.set_speed(-speed, 0)  # 前后反向

    def go_back(self, speed=1):
        self.set_speed(speed, 0)  # 前后反向

    def turn_left(self, speed=1):
        self.set_speed(0, speed)

    def turn_right(self, speed=1):
        self.set_speed(0, -speed)

    def stop(self):
        self.set_speed(0, 0)

    def go_following_wall(self):
        raise NotImplemented  # TODO
        # if self.contours is None:
        #     self.go_front()
        # else:
        #     # assert len(self.contours) == 1
        #     action = self.get_action_from_contour(self.contours[0])
        #     action()

    def is_colliding_wall(self):
        raise NotImplemented  # TODO
        # return len(self.contours) >= 1

    def is_colliding_another_robot(self):
        raise NotImplemented  # TODO
        # return self.is_colliding_left() or self.is_colliding_right()

    def is_visiting_turning_point(self):
        return self.turning_point

    def is_revisiting_places(self):
        return self.olfactory_info.get()["value"] > 400  # TODO

    def is_leaving_gathering_circle(self):
        raise NotImplemented  # TODO

    def is_finish_gathering(self):
        raise NotImplemented  # TODO

    def is_found_injuries(self):
        raise NotImplemented  # TODO

    def is_colliding_left(self):
        raise NotImplemented  # TODO

    def is_colliding_right(self):
        raise NotImplemented  # TODO

    def get_action_from_contour(self, c):
        raise NotImplemented  # TODO

    def get_azimuths_from_sound(self):
        self.stop()
        return_value = set()
        while len(return_value) == 0:
            infos = self.auditory_info.get()
            for info in infos:
                return_value.add(info["azimuth"])
        return return_value

    def setup_vision(self):
        """Put the LiDAR setup code here."""
        raise NotImplemented  # TODO
        # if self.video_capture is None:
        #     self.video_capture = cv.VideoCapture(0)
        #     self.video_capture.set(cv.CAP_PROP_FRAME_WIDTH, WIDTH)
        #     self.video_capture.set(cv.CAP_PROP_FRAME_HEIGHT, HEIGHT)
        #     if not self.video_capture.isOpened():
        #         raise Exception("Cannot open camera.")

    def exec(self):
        self.mainWindow.show()
        sys.exit(self.app.exec())

    def run(self):
        """Main Loop for the robot's control logic. Put your code below `while True:`."""
        raise NotImplemented  # TODO
        self.setup_vision()

        while True:
            self.state.transfer_to_next_state()


if __name__ == '__main__':
    with Robot() as robot:
        robot.exec()
