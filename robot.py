#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from time import sleep
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
    INFO_CMD_STRUCT = Frame(">BBBB")
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

    def __init__(self, *args, **kwargs):
        super().__init__(daemon=True, *args, **kwargs)

        serial_devices = ['../../serial0']
        if os.path.exists('/dev/serial/by-id'):
            serial_devices += os.listdir('/dev/serial/by-id')

        QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
        QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
        self.app = QApplication(sys.argv)
        self.app.setWindowIcon(QIcon("plane.svg"))
        self.app.beep()
        self.mainWindow = views.MainWindow()
        self.controlPanel = self.mainWindow.controlPanel

        self.controlPanel.setControlButtonsEnabled(False)
        if len(serial_devices) == 0:
            self.controlPanel.setConnectionButtonsEnabled(False)

        self.controlPanel.controllerDropdown.addItems(serial_devices)
        self.controlPanel.visionDropdown.addItems(serial_devices)
        self.controlPanel.auditoryDropdown.addItems(serial_devices)
        self.controlPanel.olfactoryDropdown.addItems(serial_devices)
        self.controlPanel.motionDropdown.addItems(serial_devices)

        self.controlPanel.controllerButton.clicked.connect(self.setup_controller)
        self.controlPanel.visionButton.clicked.connect(self.setup_vision)
        self.controlPanel.auditoryButton.clicked.connect(self.setup_auditory)
        self.controlPanel.olfactoryButton.clicked.connect(self.setup_olfactory)
        self.controlPanel.motionButton.clicked.connect(self.setup_motion)

        self.controlPanel.goButton.clicked.connect(self.on_go_button_clicked)
        self.controlPanel.stopButton.clicked.connect(self.on_stop_button_clicked)
        self.controlPanel.startButton.clicked.connect(self.on_start_button_clicked)
        smallControl = self.controlPanel.smallControl
        smallControl.frontButton.pressed.connect(self.on_front_button_pressed)
        smallControl.backButton.pressed.connect(self.on_back_button_pressed)
        smallControl.leftButton.pressed.connect(self.on_left_button_pressed)
        smallControl.rightButton.pressed.connect(self.on_right_button_pressed)

        self.stop_event = Event()
        self.battery_timer = QTimer()
        self.battery_timer.timeout.connect(self.update_battery_info)  # must use QTimes to avoid QProgressBar segfault
        self.olfactory_timer = QTimer()
        self.olfactory_timer.timeout.connect(self.update_olfactory_info)

        # these infos must be created before the server is ready
        self.auditory_info = Shared(on_update=self.update_auditory_info)
        self.olfactory_info = Shared(on_update=self.update_olfactory_info)
        self.motion_info = Shared(on_update=self.update_motion_info)

        self.server = ThreadingTCPServer(self, (HOST, PORT), ThreadingTCPRequestHandler)
        self.server.allow_reuse_address = True
        # Start a thread with the server -- that thread will then start one more thread for each request
        server_thread = Thread(target=self.server.serve_forever, daemon=True)
        server_thread.start()
        print(f"Server loop running in thread: {server_thread.name}...")

        self.controller = None
        self.vision_process = None
        self.auditory_process = None
        self.olfactory_device = None
        self.motion_process = None

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
            with self.controller.lock:
                self.controller.shared.close()
        # GPIO.cleanup()
        if self.vision_process:
            self.vision_process.terminate()
            # self.video_capture.release()
            # cv.destroyAllWindows()
        if self.motion_process:
            self.motion_process.terminate()
        if self.auditory_process:
            self.auditory_process.terminate()
        # no need to clean up olfactory
        self.server.shutdown()
        self.server.server_close()
        print("Done.")

    def update_battery_info(self):
        voltage, current, temperature, remaining = self.get_battery_info()
        self.controlPanel.batteryVoltage.setText(f"{voltage:.2f} V")
        self.controlPanel.batteryCurrent.setText(f"{current:.2f} A")
        self.controlPanel.batteryTemperature.setText(f"{temperature:.2f} ℃")
        self.controlPanel.batteryRemaining.setValue(remaining)

    def update_auditory_info(self, info):
        tagId = info["tagId"]
        timeStamp = info["timeStamp"]
        if timeStamp % 10 == 0:
            self.mainWindow.soundDirectionView.updatePoints(tagId, info["azimuth"])
        if timeStamp % 20 == 0:
            timeStamp //= 20
            self.mainWindow.distanceView.update(timeStamp, tagId, info["distance"])
            self.mainWindow.elevationView.update(timeStamp, tagId, info["elevation"])

    def update_olfactory_info(self):
        self.olfactory_device.write(" ")
        info = self.olfactory_device.read_all()
        self.mainWindow.ethanolChartView.update(info["timeStamp"], "value", info["value"])

    def update_motion_info(self, info):
        self.mainWindow.compassView.updateCompass(info)

    def on_front_button_pressed(self):
        self.go_front(abs(self.controlPanel.xSpeedSpinBox.value()))
        # sleep(1)
        # self.stop()

    def on_back_button_pressed(self):
        self.go_back(abs(self.controlPanel.xSpeedSpinBox.value()))
        # sleep(1)
        # self.stop()

    def on_left_button_pressed(self):
        self.turn_left(abs(self.controlPanel.zSpeedSpinBox.value()))
        # sleep(1)
        # self.stop()

    def on_right_button_pressed(self):
        self.turn_right(abs(self.controlPanel.zSpeedSpinBox.value()))
        # sleep(1)
        # self.stop()

    def on_go_button_clicked(self):
        self.set_speed(self.controlPanel.xSpeedSpinBox.value(),
                       self.controlPanel.zSpeedSpinBox.value())

    def on_stop_button_clicked(self):
        self.stop()
        self.stop_event.set()
        self.controlPanel.startButton.setText("Start the Algorithm")
        self.controlPanel.startButton.setEnabled(True)
        self.controlPanel.setConnectionButtonsEnabled(True)

    def on_start_button_clicked(self):
        self.stop_event.clear()
        self.controlPanel.startButton.setText("Algorithm Started. Press Stop to Stop...")
        self.controlPanel.startButton.setEnabled(False)
        self.controlPanel.setConnectionButtonsEnabled(False)
        self.start()

    def setup_controller(self):
        if self.controller is None:
            self.controller = Shared(Serial(f"/dev/serial/by-id/{self.controlPanel.controllerDropdown.currentText()}", 230400))
            self.controlPanel.controllerButton.setText("Disconnect")
            self.battery_timer.start(1000)
            self.controlPanel.setControlButtonsEnabled(True)
        else:
            self.battery_timer.stop()
            with self.controller.lock:
                self.controller.shared.close()
            self.controller = None
            self.controlPanel.controllerButton.setText("Connect")
            self.controlPanel.setControlButtonsEnabled(False)

    def setup_vision(self):
        """Put the LiDAR setup code here."""
        if self.vision_process is None:
            with open("/home/pi/catkin_ws/src/rplidar_ros/launch/rplidar.launch", "w") as f:
                f.write(f"""<launch>
  <node name="rplidarNode"          pkg="rplidar_ros"  type="rplidarNode" output="screen">
  <param name="serial_port"         type="string" value="/dev/serial/by-id/{self.controlPanel.visionDropdown.currentText()}"/>
  <param name="serial_baudrate"     type="int"    value="115200"/><!--A1/A2 -->
  <!--param name="serial_baudrate"     type="int"    value="256000"--><!--A3 -->
  <param name="frame_id"            type="string" value="laser"/>
  <param name="inverted"            type="bool"   value="false"/>
  <param name="angle_compensate"    type="bool"   value="true"/>
  </node>
</launch>""")
            self.vision_process = subprocess.Popen(("roslaunch", "rplidar_ros", "rplidar.launch"))
            self.controlPanel.visionButton.setText("Disconnect")
        else:
            self.vision_process.terminate()
            self.vision_process = None
            self.controlPanel.visionButton.setText("Connect")
        # if self.video_capture is None:
        #     self.video_capture = cv.VideoCapture(0)
        #     self.video_capture.set(cv.CAP_PROP_FRAME_WIDTH, WIDTH)
        #     self.video_capture.set(cv.CAP_PROP_FRAME_HEIGHT, HEIGHT)
        #     if not self.video_capture.isOpened():
        #         raise Exception("Cannot open camera.")

    def setup_auditory(self):
        if self.auditory_process is None:
            self.auditory_process = subprocess.Popen(("/home/pi/BluetoothAoaLocator/build/BluetoothAoaLocator",
                                                      "-s", HOST, str(PORT),
                                                      "-u", f"/dev/serial/by-id/{self.controlPanel.auditoryDropdown.currentText()}"))
            print(f"BluetoothAoaLocator client loop running in process: {self.auditory_process.pid}...")
            self.controlPanel.auditoryButton.setText("Disconnect")
        else:
            self.auditory_process.terminate()
            self.auditory_process = None
            self.controlPanel.auditoryButton.setText("Connect")

    def setup_olfactory(self):
        if self.olfactory_device is None:
            self.olfactory_device = Serial(f"/dev/serial/by-id/{self.controlPanel.olfactoryDropdown.currentText()}", 115200)
            self.controlPanel.olfactoryButton.setText("Disconnect")
        else:
            self.olfactory_device.close()
            self.olfactory_device = None  # also releases serial
            self.controlPanel.olfactoryButton.setText("Connect")

    def setup_motion(self):
        if self.motion_process is None:
            self.motion_process = subprocess.Popen(("/home/pi/Robot/JY901S_socket", HOST, str(PORT),
                                                    f"/dev/serial/by-id/{self.controlPanel.motionDropdown.currentText()}"))
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
        with self.controller.lock:
            self.controller.shared.write(cmd)
            self.controller.shared.read_all()

    def get_speed(self):
        """:returns x_speed, y_speed, z_speed"""
        cmd = Robot.INFO_CMD_STRUCT.pack_with_chksum(0xFE, 0xEF, 0x01, 0x02)
        with self.controller.lock:
            self.controller.shared.write(cmd)
            result = self.controller.shared.read(17)
        return Robot.SPEED_INFO_STRUCT.unpack(result[4:-1])

    def get_battery_info(self):
        """:returns voltage, current, temperature, remaining"""
        cmd = Robot.INFO_CMD_STRUCT.pack_with_chksum(0xFE, 0xEF, 0x01, 0x03)
        with self.controller.lock:
            self.controller.shared.write(cmd)
            result = self.controller.shared.read(12)
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
