#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import traceback
from time import sleep
import math
import os
import subprocess
from typing import List

# import numpy as np
# import cv2 as cv
import RPi.GPIO as GPIO
import rospy
from sensor_msgs.msg import LaserScan
from laser_line_extraction.msg import LineSegmentList, LineSegment
from obstacle_detector.msg import Obstacles, CircleObstacle

from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QIcon
from PyQt5.QtWidgets import QApplication, QMessageBox

from state import *
from utils import *
import views


HOST = "127.0.0.1"
PORT = 8080
PUMP_CTRL_PIN = 17
# FRONT = scan_index(0)
# JUDGE_5 = scan_index(5)
# LEFT_FRONT_ANGLE = 40
# LEFT_FRONT = scan_index(LEFT_FRONT_ANGLE)
# LEFT = scan_index(90)
# BACK = scan_index(180)
# RIGHT = scan_index(270)
# RIGHT_FRONT = scan_index(360 - LEFT_FRONT_ANGLE)
# JUDGE_355 = scan_index(355)
# ANGLES = (FRONT, JUDGE_5, LEFT_FRONT, LEFT, BACK, RIGHT, RIGHT_FRONT, JUDGE_355)
WALL_CROSS_THRESH = 0.2
FRONT_WALL_MIN_DISTANCE = 0.5
SIDE_WALL_MIN_DISTANCE = 0.3
SIDE_WALL_MAX_DISTANCE = 1.0
# WALL_UPPER_THRESH = 0.5 / math.sin(LEFT_FRONT_ANGLE)
# WALL_LOWER_THRESH = 0.3 / math.sin(LEFT_FRONT_ANGLE)


class Robot(Thread):
    INFO_CMD_STRUCT = Frame(">BBBB")
    SPEED_CONTROL_STRUCT = Frame(">BBBBfff")
    SPEED_INFO_STRUCT = Frame(">fff")
    MOTION_INFO_STRUCT = Frame(">fffffffff")
    BATTERY_INFO_STRUCT = Frame(">HHHB")

    class JustStartedState(AbstractState):
        def __init__(self, robot):
            super().__init__(robot)

        def transfer_when_colliding_another_robot(self):
            super().transfer_when_colliding_another_robot()
            # self.__robot.go_back()  # TODO
            # self.__robot.turn_right()  # TODO
            robot: Robot = self.get_robot()
            robot.turn_degree(-90)

        def transfer_when_colliding_wall(self):
            super().transfer_when_colliding_wall()
            robot: Robot = self.get_robot()
            if robot.colliding_wall:
                if robot.auditory_process:
                    v = Vector2D(0, 0)
                    for a in robot.get_azimuths_from_sound():
                        v += Vector2D.from_azimuth(a)
                    azimuth = v.to_azimuth()
                    if azimuth < 0:  # need to turn left
                        robot.turn_degree(robot.colliding_wall.angle + 90, True)
                    else:  # need to turn right
                        robot.turn_degree(robot.colliding_wall.angle - 90, True)
                else:
                    if robot.colliding_wall.angle < 0:
                        angle = robot.colliding_wall.angle + 90
                        robot.turn_degree(angle, True)  # TODO
                    else:
                        angle = robot.colliding_wall.angle - 90
                        robot.turn_degree(angle, True)  # TODO
            robot.set_state(robot.following_wall_state)

        def transfer_when_not_following_wall(self):
            super().transfer_when_not_following_wall()
            robot: Robot = self.get_robot()
            if robot.auditory_process:
                v = Vector2D(0, 0)
                for a in robot.get_azimuths_from_sound():
                    v += Vector2D.from_azimuth(a)
                azimuth = v.to_azimuth()
                if azimuth > 0:
                    azimuth -= 180
                else:
                    azimuth += 180
                current = int(robot.get_yaw(robot.motion_info.get()))
                robot.turn_degree(normalize_azimuth(azimuth))  # TODO: check
            else:
                current = int(robot.get_yaw(robot.motion_info.get()))
                robot.turn_degree(normalize_azimuth(robot.initial_azimuth - current))
            robot.collide_turn_function = None

    class FollowingWallState(AbstractState):
        def __init__(self, robot):
            super().__init__(robot)

        def transfer_when_colliding_another_robot(self):
            super().transfer_when_colliding_another_robot()
            robot: Robot = self.get_robot()
            if robot.collide_turn_function == robot.turn_left:
                robot.turn_degree(90)
            else:
                robot.turn_degree(-90)
            # self.__robot.go_back()  # TODO
            # self.__robot.turn_right()  # TODO

        def transfer_when_colliding_wall(self):
            super().transfer_when_colliding_wall()
            robot: Robot = self.get_robot()
            if robot.collide_turn_function == robot.turn_left:
                robot.turn_degree(robot.colliding_wall.angle + 90)  # TODO
            else:
                robot.turn_degree(robot.colliding_wall.angle - 90)  # TODO

        def transfer_when_not_following_wall(self):
            super().transfer_when_not_following_wall()
            robot: Robot = self.get_robot()
            if robot.has_revisited_places:
                self.transfer_when_revisiting_places()
            else:
                robot.turn_according_to_wall()

        def transfer_when_revisiting_places(self):
            super().transfer_when_revisiting_places()
            robot: Robot = self.get_robot()
            robot.has_revisited_places = False
            robot.set_state(robot.just_started_state)
            robot.get_state().transfer_when_not_following_wall()

        def transfer_to_next_state(self):
            robot: Robot = self.get_robot()
            if robot.is_colliding_another_robot():
                self.transfer_when_colliding_another_robot()
            elif robot.is_visiting_turning_point():  # Needs Turning
                print("is_visiting_turning_point!")
                self.transfer_when_not_following_wall()
            elif robot.is_colliding_wall():
                self.transfer_when_colliding_wall()
            else:
                # print("go_following_wall!")
                robot.go_following_wall()

    def __init__(self, *args, **kwargs):
        super().__init__(daemon=True, *args, **kwargs)
        self.tag_id = "D2"  # TODO: "8B", "AC", "C3", "D2"
        self.pump_output = 0
        self.initial_azimuth = None
        self.initial_x_speed = 0.5

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(PUMP_CTRL_PIN, GPIO.OUT)

        # disable when using vc4-fkms-v3d on Raspberry Pi
        # QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
        # QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
        self.app = QApplication(sys.argv)
        self.app.setWindowIcon(QIcon("plane.svg"))
        self.app.beep()
        self.mainWindow = views.MainWindow()
        self.controlPanel = self.mainWindow.controlPanel

        self.controlPanel.setControlButtonsEnabled(False)
        self.controlPanel.controllerDropdown.update_signal.connect(self.update_device_list)
        self.controlPanel.visionDropdown.update_signal.connect(self.update_device_list)
        self.controlPanel.auditoryDropdown.update_signal.connect(self.update_device_list)
        self.controlPanel.olfactoryDropdown.update_signal.connect(self.update_device_list)
        self.controlPanel.motionDropdown.update_signal.connect(self.update_device_list)
        self.update_device_list()
        
        self.controlPanel.controllerButton.clicked.connect(self.setup_controller)
        self.controlPanel.visionButton.clicked.connect(self.setup_vision)
        self.controlPanel.auditoryButton.clicked.connect(self.setup_auditory)
        self.controlPanel.olfactoryButton.clicked.connect(self.setup_olfactory)
        self.controlPanel.motionButton.clicked.connect(self.setup_motion)

        self.controlPanel.olfactoryThresh.setValue(300)
        self.controlPanel.pumpOpenSec.setValue(0.35)
        self.controlPanel.pumpCloseSec.setValue(10)
        self.controlPanel.xSpeedSpinBox.setValue(self.initial_x_speed)
        self.controlPanel.zSpeedSpinBox.setValue(0.5)

        self.controlPanel.pumpDebugButton.clicked.connect(self.on_debug_pump_button_clicked)
        self.controlPanel.goButton.clicked.connect(self.on_go_button_clicked)
        self.controlPanel.stopButton.clicked.connect(self.on_stop_button_clicked)
        self.controlPanel.startButton.clicked.connect(self.on_start_button_clicked)
        self.controlPanel.exec_button.clicked.connect(self.on_exec_button_clicked)
        smallControl = self.controlPanel.smallControl
        smallControl.frontButton.pressed.connect(self.on_front_button_pressed)
        smallControl.backButton.pressed.connect(self.on_back_button_pressed)
        smallControl.leftButton.pressed.connect(self.on_left_button_pressed)
        smallControl.rightButton.pressed.connect(self.on_right_button_pressed)

        self.stop_event = Event()
        self.stop_event.set()
        self.battery_timer = QTimer()
        self.battery_timer.timeout.connect(self.update_battery_info)  # must use QTimes to avoid QProgressBar segfault
        self.olfactory_timer = QTimer()
        self.olfactory_timer.timeout.connect(self.update_olfactory_info)
        # self.pump_timer = QTimer()
        # self.pump_timer.timeout.connect(self.control_pump)
        self.pump_start_event = Event()  # used to start or stop the pump
        self.pump_stop_event = Event()  # used to stop the pump when exiting the program
        self.pump_thread = Thread(target=self.control_pump, daemon=True)
        self.pump_thread.start()

        # these infos must be created before the server is ready
        self.auditory_tags = {k: 0 for k in views.TAG_IDS}
        self.auditory_info = Shared(on_update=self.update_auditory_info, needs_update=True)
        # self.olfactory_info = Shared(on_update=self.update_olfactory_info)
        self.has_revisited_places = False
        self.motion_info = Shared(on_update=self.update_motion_info, needs_update=True)

        # self.distance = None
        self.walls: List[LineSegment] = None
        self.nearest_wall: LineSegment = None
        self.next_wall: LineSegment = None

        self.server = ThreadingTCPServer(self, (HOST, PORT), ThreadingTCPRequestHandler)
        self.server.allow_reuse_address = True
        # Start a thread with the server -- that thread will then start one more thread for each request
        server_thread = Thread(target=self.server.serve_forever, daemon=True)
        server_thread.start()
        print(f"Server loop running in thread: {server_thread.name}...")

        rospy.init_node('turtlebot_scan')
        self.controller = None
        self.vision_process = None
        self.vision_subscriber = None
        self.wall_subscriber = None
        self.auditory_process = None
        self.olfactory_device = None
        self.motion_process = None

        # self.saw_img = None
        # self.contours = None
        # self.distance = {k: 0 for k in ANGLES}
        # self.left_front_angles = []
        # self.right_front_angles = []
        self.turning_point = False
        self.collide_turn_function = None
        self.is_colliding_others = False

        self.just_started_state = self.JustStartedState(self)
        self.following_wall_state = self.FollowingWallState(self)
        self.__state = self.just_started_state

        self.in_room = False
        self.mission_complete = False

        # GPIO.setmode(GPIO.BCM)
        # GPIO.setup(self.DIN1, GPIO.OUT)

    def __enter__(self):
        return self

    def __exit__(self, *args):
        print("Cleaning up...")
        self.stop_event.set()
        self.pump_stop_event.set()
        GPIO.output(PUMP_CTRL_PIN, 0)
        GPIO.cleanup()
        if self.controller:
            self.stop()
            with self.controller.lock:
                self.controller.instance.close()
        if self.vision_subscriber:
            self.vision_subscriber.unregister()
        if self.wall_subscriber:
            self.wall_subscriber.unregister()
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

    def __str__(self):
        return f"state={self.__state}, collide_turn_func={self.collide_turn_function}"

    def set_state(self, state):
        self.__state = state
        self.mainWindow.setWindowTitle(f"{state}")

    def get_state(self):
        return self.__state

    def update_device_list(self):
        serial_devices = ['../../serial0']
        if os.path.exists('/dev/serial/by-id'):
            serial_devices += os.listdir('/dev/serial/by-id')
        if len(serial_devices) == 0:
            self.controlPanel.setConnectionButtonsEnabled(False)
        self.controlPanel.controllerDropdown.clear()
        self.controlPanel.visionDropdown.clear()
        self.controlPanel.auditoryDropdown.clear()
        self.controlPanel.olfactoryDropdown.clear()
        self.controlPanel.motionDropdown.clear()
        self.controlPanel.controllerDropdown.addItems(serial_devices)
        self.controlPanel.visionDropdown.addItems(serial_devices)
        self.controlPanel.auditoryDropdown.addItems(serial_devices)
        self.controlPanel.olfactoryDropdown.addItems(serial_devices)
        self.controlPanel.motionDropdown.addItems(serial_devices)
        for i, s in enumerate(serial_devices):
            if s == "usb-1a86_USB_Serial-if00-port0":
                self.controlPanel.controllerDropdown.setCurrentIndex(i)
            elif s == "usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0":
                self.controlPanel.visionDropdown.setCurrentIndex(i)
            elif s == "usb-Silicon_Labs_J-Link_Pro_OB_000440204772-if00":
                self.controlPanel.auditoryDropdown.setCurrentIndex(i)
            elif s == "usb-Seeed_Seeed_XIAO_M0_01EF4D4750555158372E3120FF072114-if00":
                self.controlPanel.olfactoryDropdown.setCurrentIndex(i)

    def update_battery_info(self):
        voltage, current, temperature, remaining = self.get_battery_info()
        self.controlPanel.batteryVoltage.setText(f"{voltage:.2f} V")
        self.controlPanel.batteryCurrent.setText(f"{current:.2f} A")
        self.controlPanel.batteryTemperature.setText(f"{temperature:.2f} ℃")
        self.controlPanel.batteryRemaining.setValue(remaining)

    def update_auditory_info(self, info):
        tagId = info["tagId"]
        if tagId != self.tag_id:
            timeStamp = max(self.auditory_tags.values())
            if timeStamp % 10 == 0:
                self.mainWindow.soundDirectionView.updatePoints(tagId, info["azimuth"])
            if timeStamp % 30 == 0:
                # print(info)
                timeStamp //= 30
                self.mainWindow.distanceView.update(timeStamp, tagId, info["distance"])
                self.mainWindow.elevationView.update(timeStamp, tagId, info["elevation"])
            self.auditory_tags[tagId] += 1

    def get_olfactory_info(self):
        with self.olfactory_device.lock:
            self.olfactory_device.instance.write(b" ")
            try:
                result = self.olfactory_device.instance.read_until(b"\r")
                return json.loads(result)
            except:
                return {"timeStamp": -1, "value": -1}

    def update_olfactory_info(self):
        info = self.get_olfactory_info()
        thresh = self.controlPanel.olfactoryThresh.value()
        if info["value"] > thresh:
            print(f"Olfactory higher than {thresh}!")
            self.has_revisited_places = True
        timestamp = info["timeStamp"]
        if timestamp % 10 == 0:
            timestamp //= 10
            self.mainWindow.ethanolChartView.update(timestamp, "value", info["value"])

    def update_motion_info(self, info):
        self.mainWindow.compassView.updateCompass(self.get_yaw(info))

    def control_pump(self):
        while not self.pump_stop_event.is_set() and self.pump_start_event.wait():
            GPIO.output(PUMP_CTRL_PIN, 1)
            if self.pump_stop_event.is_set():
                return
            sleep(self.controlPanel.pumpOpenSec.value())
            GPIO.output(PUMP_CTRL_PIN, 0)
            if self.pump_stop_event.is_set():
                return
            sleep(self.controlPanel.pumpCloseSec.value())

    def on_debug_pump_button_clicked(self):
        if self.pump_start_event.is_set():
            self.pump_start_event.clear()
            self.controlPanel.pumpDebugButton.setText("Start Pump")
        else:
            self.pump_start_event.set()
            self.controlPanel.pumpDebugButton.setText("Stop Pump")

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
        self.pump_start_event.clear()
        self.__state = self.just_started_state
        self.mainWindow.setWindowTitle(f"Robot Studio")
        self.has_revisited_places = False
        self.walls = None
        self.nearest_wall = None
        self.next_wall = None
        self.turning_point = False
        self.collide_turn_function = None
        self.is_colliding_others = False
        self.in_room = False
        self.mission_complete = False
        self.initial_azimuth = None
        self.controlPanel.startButton.setText("Start the Algorithm")
        self.controlPanel.startButton.setEnabled(True)
        self.controlPanel.setConnectionButtonsEnabled(True)

    def set_initial_azimuth(self):
        self.initial_azimuth = self.get_yaw(self.motion_info.get())

    def on_start_button_clicked(self):
        if self.stop_event.is_set():
            self.set_initial_azimuth()
            self.initial_x_speed = self.controlPanel.xSpeedSpinBox.value()
            self.stop_event.clear()
            self.mainWindow.setWindowTitle(f"{self.__state}")
            self.controlPanel.startButton.setText("Algorithm Started. Press Again to Pause")
            self.controlPanel.setConnectionButtonsEnabled(False)
            self.pump_start_event.set()
            # self.start()
        else:
            self.stop()
            self.stop_event.set()
            self.pump_start_event.clear()
            self.controlPanel.startButton.setText("Algorithm Paused. Press Again to Continue")

    def on_exec_button_clicked(self):
        try:
            eval(self.controlPanel.cmd_box.text())
        except:
            QMessageBox.critical(self.controlPanel.cmd_box, "Error", traceback.format_exc())

    def setup_controller(self):
        if self.controller is None:
            self.controller = Shared(Serial(f"/dev/serial/by-id/{self.controlPanel.controllerDropdown.currentText()}", 230400))
            self.controlPanel.controllerButton.setText("Disconnect")
            self.battery_timer.start(1000)
            self.controlPanel.setControlButtonsEnabled(True)
        else:
            self.battery_timer.stop()
            with self.controller.lock:
                self.controller.instance.close()
            self.controller = None
            self.controlPanel.controllerButton.setText("Connect")
            self.controlPanel.setControlButtonsEnabled(False)

    def setup_vision(self):
        """Put the LiDAR setup code here."""
        if self.vision_process is None:
            with open("/home/pi/catkin_ws/src/rplidar_ros-master/launch/rplidar.launch", "w") as f:
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
            self.vision_process = subprocess.Popen(("roslaunch", "obstacle_detector", "scanner_with_wall.launch"))
            # self.vision_process = subprocess.Popen(("roslaunch", "laser_line_extraction", "example.launch"))
            # self.vision_subscriber = rospy.Subscriber('/scan', LaserScan, self.on_scan, queue_size=1)
            self.wall_subscriber = rospy.Subscriber('/line_segments', LineSegmentList, self.on_wall, queue_size=1)
            self.obstacle_subscriber = rospy.Subscriber('/obstacles', Obstacles, self.on_obstacle, queue_size=1)
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

    def go_following_wall(self):
        assert self.collide_turn_function is not None
        if self.collide_turn_function == self.turn_left:
            not_parallel_walls = []
            walls = []
            for wall in self.walls:
                if wall.radius < 2 and -120 < wall.angle < -60:
                    if not -95 < wall.angle < -85:
                        not_parallel_walls.append(wall)
                    walls.append(wall)
            if walls:
                nearest_wall = min(walls, key=lambda x: x.radius)
                if nearest_wall.radius < SIDE_WALL_MIN_DISTANCE:
                    print(f"Right Wall Too Near!\n{nearest_wall}")
                    self.go_front_left(z_speed=0.1)
                    # sleep(0.75)
                    # self.go_front_right()
                    return
                elif nearest_wall.radius > SIDE_WALL_MAX_DISTANCE:
                    print(f"Right Wall Too Far!\n{nearest_wall}")
                    self.go_front_right(z_speed=0.1)
                    # sleep(0.75)
                    # self.go_front_left()
                    return
            if not_parallel_walls:
                nearest_wall = min(not_parallel_walls, key=lambda x: x.radius)
                # self.turn_degree(nearest_wall.angle + 90)
                if nearest_wall.angle > -90:
                    self.go_front_left(z_speed=0.1)
                else:
                    self.go_front_right(z_speed=0.1)
                print(f"Right Wall:\n{nearest_wall}")
            else:
                self.go_front()
        else:
            not_parallel_walls = []
            walls = []
            for wall in self.walls:
                if wall.radius < 2 and 60 < wall.angle < 120:
                    if not 85 < wall.angle < 95:
                        not_parallel_walls.append(wall)
                    walls.append(wall)
            if walls:
                nearest_wall = min(walls, key=lambda x: x.radius)
                if nearest_wall.radius < SIDE_WALL_MIN_DISTANCE:
                    print(f"Left Wall Too Near!\n{nearest_wall}")
                    self.go_front_right(z_speed=0.1)
                    # sleep(0.75)
                    # self.go_front_left()
                    return
                elif nearest_wall.radius > SIDE_WALL_MAX_DISTANCE:
                    print(f"Left Wall Too Far!\n{nearest_wall}")
                    self.go_front_left(z_speed=0.1)
                    # sleep(0.75)
                    # self.go_front_right()
                    return
            if not_parallel_walls:
                nearest_wall = min(not_parallel_walls, key=lambda x: x.radius)
                # self.turn_degree(nearest_wall.angle - 90)
                if nearest_wall.angle < 90:
                    self.go_front_right(z_speed=0.1)
                else:
                    self.go_front_left(z_speed=0.1)
                print(f"Left Wall:\n{nearest_wall}")
            else:
                self.go_front()
    
    def dump(self):
        with open("dump.txt", "a+") as f:
            f.write(str(self.ranges))
            f.write("\n")

    def test(self):
        self.collide_turn_function = self.turn_left
        while True:
            print(self.is_visiting_turning_point())

    def on_scan(self, data):
        self.ranges = data.ranges
        self.dump()
        # isinf = math.isinf
        # for i in ANGLES:
        #     if not isinf(self.ranges[i]):
        #         self.distance[i] = self.ranges[i]
        # self.left_front_angles.append(self.distance[LEFT_FRONT])
        # self.right_front_angles.append(self.distance[RIGHT_FRONT])

        # self.debug()
        # print(f"Front: {self.distance[JUDGE_5] - self.distance[JUDGE_355]}")
        # print(f"Left: {self.get_left_wall_angle()}")
        # print(f"right: {self.get_right_wall_angle()}")
        # self.controlPanel.startButton.setText(f"{self.state}")
        # print(self.state)
        # self.state.transfer_to_next_state()

    def on_wall(self, data):
        self.walls = data.line_segments
        if self.walls:
            self.nearest_wall = min(self.walls, key=lambda x: x.radius)
        if not self.stop_event.is_set():
            # print(f"Nearest:\n{self.nearest_wall}")
            self.__state.transfer_to_next_state()
        # print(f"angle: {normalize_azimuth(self.nearest_wall.angle * 180 / math.pi - 180)}")
        # start_x, start_y = self.nearest_wall.start
        # print(f"start: [{start_y}, {-start_x}]")
        # end_x, end_y = self.nearest_wall.end
        # print(f"end: [{end_y}, {-end_x}]\n")

    def change_speed_according_to_front(self, d):
        speed = self.controlPanel.xSpeedSpinBox.value()
        # print(f"min_d: {d}, cur_speed: {speed}")
        if d < 1:
            if speed > 0.3:
                self.controlPanel.xSpeedSpinBox.setValue(speed - 0.1)
        else:
            if speed < self.initial_x_speed:
                self.controlPanel.xSpeedSpinBox.setValue(speed + 0.1)


    def on_obstacle(self, data: Obstacles):
        min_ob = float('inf')
        for c in data.circles:
            x, y = c.center.x, c.center.y
            d = dist((x, y), (0, 0))
            if x > 0 and -0.577 < y / x < 0.577:
                if d < min_ob:
                    min_ob = d
                if d < 0.7:
                    # print(f"Obstacle Found: {c}")
                    self.is_colliding_others = True
                    return
        self.change_speed_according_to_front(min_ob)
        self.is_colliding_others = False

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
            self.olfactory_device = Shared(Serial(f"/dev/serial/by-id/{self.controlPanel.olfactoryDropdown.currentText()}", 115200))
            self.olfactory_timer.start(100)
            self.controlPanel.olfactoryButton.setText("Disconnect")
        else:
            self.olfactory_timer.stop()
            with self.olfactory_device.lock:
                self.olfactory_device.instance.close()
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

    @staticmethod
    def get_yaw(info) -> float:
        return info["azimuth"]["yaw"]

    def turn_degree(self, deg, update_collide_turn_function=False):
        """degree < 0: right, > 0: left"""
        print(f"Turning {deg}!")
        if update_collide_turn_function:
            if deg < 0:
                self.collide_turn_function = self.turn_right
            else:
                self.collide_turn_function = self.turn_left
        Kp = 0.01
        goal = normalize_azimuth(int(self.get_yaw(self.motion_info.get())) + deg)
        while True:
            # self.stop()
            # eval(input())
            current = int(self.get_yaw(self.motion_info.get()))
            yaw_err = normalize_azimuth(goal - current)

            # print(f"goal: {goal},\tcurrent: {current},\tyaw_err: {yaw_err}")
            if abs(yaw_err) < 2:
                self.stop()
                print("Turning Finished.")
                return

            yaw_rate = Kp * abs(yaw_err)
            # 限幅，一个最大的角速度，一个最小的可以转动的
            if yaw_rate < 0.1:
                yaw_rate = 0.1
            elif yaw_rate > 1.0:
                yaw_rate = 1.0
            # print(yaw_rate)
            if yaw_err > 0:
                self.turn_left(abs(yaw_rate))
            else:
                self.turn_right(abs(yaw_rate))

    def set_speed(self, x, z):
        cmd = Robot.SPEED_CONTROL_STRUCT.pack_with_chksum(0xFE, 0xEF, 0x0D, 0x01, x, 0, z)
        with self.controller.lock:
            self.controller.instance.write(cmd)
            self.controller.instance.read_all()

    def get_speed(self):
        """:returns x_speed, y_speed, z_speed"""
        cmd = Robot.INFO_CMD_STRUCT.pack_with_chksum(0xFE, 0xEF, 0x01, 0x02)
        with self.controller.lock:
            self.controller.instance.write(cmd)
            result = self.controller.instance.read(17)
        return Robot.SPEED_INFO_STRUCT.unpack(result[4:-1])

    def get_motion(self):
        cmd = Robot.INFO_CMD_STRUCT.pack_with_chksum(0xFE, 0xEF, 0x01, 0x04)
        with self.controller.lock:
            self.controller.instance.write(cmd)
            result = self.controller.instance.read(41)
        return Robot.MOTION_INFO_STRUCT.unpack(result[4:-1])

    def get_battery_info(self):
        """:returns voltage, current, temperature, remaining"""
        cmd = Robot.INFO_CMD_STRUCT.pack_with_chksum(0xFE, 0xEF, 0x01, 0x03)
        with self.controller.lock:
            self.controller.instance.write(cmd)
            result = self.controller.instance.read(12)
        voltage, current, temperature, remaining = Robot.BATTERY_INFO_STRUCT.unpack(result[4:-1])
        return voltage / 100, current / 100, temperature / 10, remaining

    def go_front(self, speed=None):
        if speed is None:
            speed = self.controlPanel.xSpeedSpinBox.value()
        self.set_speed(-speed, 0)  # 前后反向

    def go_back(self, speed=None):
        if speed is None:
            speed = self.controlPanel.xSpeedSpinBox.value()
        self.set_speed(speed, 0)  # 前后反向

    def turn_left(self, speed=None):
        if speed is None:
            speed = self.controlPanel.zSpeedSpinBox.value()
        self.set_speed(0, speed)

    def turn_right(self, speed=None):
        if speed is None:
            speed = self.controlPanel.zSpeedSpinBox.value()
        self.set_speed(0, -speed)

    def go_front_left(self, x_speed=None, z_speed=None):
        if x_speed is None:
            x_speed = self.controlPanel.xSpeedSpinBox.value()
        if z_speed is None:
            z_speed = self.controlPanel.zSpeedSpinBox.value()
        self.set_speed(-x_speed, z_speed)

    def go_front_right(self, x_speed=None, z_speed=None):
        if x_speed is None:
            x_speed = self.controlPanel.xSpeedSpinBox.value()
        if z_speed is None:
            z_speed = self.controlPanel.zSpeedSpinBox.value()
        self.set_speed(-x_speed, -z_speed)

    def stop(self):
        self.set_speed(0, 0)

    def turn_according_to_wall(self):
        assert self.collide_turn_function is not None
        # self.go_front()
        # sleep(0.5 / self.controlPanel.xSpeedSpinBox.value())
        if self.collide_turn_function == self.turn_left:
            self.turn_degree(normalize_azimuth(self.next_wall.angle + 90))
        else:
            self.turn_degree(normalize_azimuth(self.next_wall.angle - 90))
        # self.go_front()
        # sleep(1)

    def is_colliding_wall(self):
        min_d = float('inf')
        if self.__state == self.just_started_state:
            for wall in self.walls:
                if -90 < wall.angle < 90:
                    if wall.radius < min_d:
                        min_d = wall.radius
                    if wall.radius < FRONT_WALL_MIN_DISTANCE:
                        self.colliding_wall = wall
                        print(f"Colliding Wall:\n{wall}")
                        return wall
        else:
            for wall in self.walls:
                start_x, _ = wall.start
                end_x, _ = wall.end
                if start_x * end_x < 0 and -45 < wall.angle < 45:
                    if wall.radius < min_d:
                        min_d = wall.radius
                    if wall.radius < FRONT_WALL_MIN_DISTANCE:
                        self.colliding_wall = wall
                        print(f"Colliding Wall:\n{wall}")
                        return wall
        self.change_speed_according_to_front(min_d)
        return None
            # walls = []
            # if self.collide_turn_function == self.turn_left:
            #     for wall in self.walls:
            #         start_x, start_y = wall.start
            #         end_x, end_y = wall.end
            #         if start_x > 0 and start_y > 0 or end_x > 0 and end_y > 0:
            #             walls.append(wall)
            # else:
            #     for wall in self.walls:
            #         start_x, start_y = wall.start
            #         end_x, end_y = wall.end
            #         if start_x < 0 and start_y > 0 or end_x < 0 and end_y > 0:
            #             walls.append(wall)
            # for i in walls:
            #     for j in walls:
            #         if abs(normalize_azimuth(i.angle - j.angle)) < 30:
            #             continue
            #         if dist(i.start, j.start) < NEAR_THRESH or \
            #         dist(i.start, j.end) < NEAR_THRESH or \
            #         dist(i.end, j.start) < NEAR_THRESH or \
            #         dist(i.end, j.end) < NEAR_THRESH:
            #             if i.start[1] > 0 and i.end[1] > 0 and (j.start[1] < 0 or j.end[1] < 0):
            #                 self.next_wall = i
            #                 print(f"Corner:\n{i}\n{j}")
            #                 return i
            #             if j.start[1] > 0 and j.end[1] > 0 and (i.start[1] < 0 or i.end[1] < 0):
            #                 self.next_wall = j
            #                 print(f"Corner:\n{i}\n{j}")
            #                 return j

    def is_colliding_another_robot(self):
        return self.is_colliding_others  # TODO

    def is_visiting_turning_point(self):
        if self.collide_turn_function is None:
            return False
        else:
            walls = []
            if self.collide_turn_function == self.turn_left:
                for wall in self.walls:
                    start_x, start_y = wall.start
                    end_x, end_y = wall.end
                    if start_x > 0 and start_y < 0 or end_x > 0 and end_y < 0:
                        walls.append(wall)
                for i in walls:
                    for j in walls:
                        if abs(normalize_azimuth(i.angle - j.angle)) < 30:
                            continue
                        if dist(i.start, j.start) < WALL_CROSS_THRESH or \
                        dist(i.start, j.end) < WALL_CROSS_THRESH or \
                        dist(i.end, j.start) < WALL_CROSS_THRESH or \
                        dist(i.end, j.end) < WALL_CROSS_THRESH:
                            start_x, start_y = i.start
                            end_x, end_y = i.end
                            if -120 < i.angle < -60 and start_x > 0 and start_y < 0 and end_x > 0 and end_y < 0:
                                self.next_wall = j
                                print(f"Corner:\n{i}\n{j}")
                                return j
                            start_x, start_y = j.start
                            end_x, end_y = j.end
                            if -120 < j.angle < -60 and start_x > 0 and start_y < 0 and end_x > 0 and end_y < 0:
                                self.next_wall = i
                                print(f"Corner:\n{i}\n{j}")
                                return i
            else:
                for wall in self.walls:
                    start_x, start_y = wall.start
                    end_x, end_y = wall.end
                    if start_x < 0 and start_y < 0 or end_x < 0 and end_y < 0:
                        walls.append(wall)
                for i in walls:
                    for j in walls:
                        if abs(normalize_azimuth(i.angle - j.angle)) < 30:
                            continue
                        if dist(i.start, j.start) < WALL_CROSS_THRESH or \
                        dist(i.start, j.end) < WALL_CROSS_THRESH or \
                        dist(i.end, j.start) < WALL_CROSS_THRESH or \
                        dist(i.end, j.end) < WALL_CROSS_THRESH:
                            start_x, start_y = i.start
                            end_x, end_y = i.end
                            if 60 < i.angle < 120 and start_x < 0 and start_y < 0 and end_x < 0 and end_y < 0:
                                self.next_wall = j
                                print(f"Corner:\n{i}\n{j}")
                                return j
                            start_x, start_y = j.start
                            end_x, end_y = j.end
                            if 60 < j.angle < 120 and start_x < 0 and start_y < 0 and end_x < 0 and end_y < 0:
                                self.next_wall = i
                                print(f"Corner:\n{i}\n{j}")
                                return i
            return None
        
    # def is_revisiting_places(self):
    #     # return False  # TODO
    #     return self.get_olfactory_info()["value"] > 100  # TODO

    def is_leaving_gathering_circle(self):
        raise NotImplemented  # TODO

    def is_finish_gathering(self):
        raise NotImplemented  # TODO

    def is_found_injuries(self):
        raise NotImplemented  # TODO

    def get_azimuths_from_sound(self):
        """left positive right negative TODO: not finished"""
        self.stop()
        return_value = set()
        while len(return_value) == 0:
            info = self.auditory_info.get()
            if info["tagId"] != self.tag_id:
                return_value.add(info["azimuth"])
        return return_value

    def exec(self):
        self.mainWindow.show()
        sys.exit(self.app.exec())

    def run(self):
        """Main Loop for the robot's control logic. Put your code below `while True:`."""
        while True:
            self.__state.transfer_to_next_state()


if __name__ == '__main__':
    with Robot() as robot:
        robot.exec()
