#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from time import sleep
import math
from threading import Thread
import subprocess

import RPi.GPIO as GPIO
import numpy as np
import cv2 as cv

from hardware import PCA9685
from state import *
from utils import *


HOST = "127.0.0.1"
PORT = 8080

WIDTH = 160
HEIGHT = 120

CENTER = WIDTH // 2
IMG_LEFT_THRESH = WIDTH // 3
IMG_RIGHT_THRESH = WIDTH * 2 // 3
IMG_UP_THRESH = HEIGHT // 3
IMG_DOWN_THRESH = HEIGHT * 2 // 3
SOUND_RIGHT_THRESH = -math.pi / 4
SOUND_LEFT_THRESH = math.pi / 4
CROPPED_HEIGHT = HEIGHT // 2
OPENING_KERNEL_SIZE = 5
OPENING_KERNEL = np.ones((OPENING_KERNEL_SIZE, OPENING_KERNEL_SIZE), np.uint8)
CLOSING_KERNEL_SIZE = 25
CLOSING_KERNEL = np.ones((CLOSING_KERNEL_SIZE, CLOSING_KERNEL_SIZE), np.uint8)


class Robot(Thread):
    class JustStartedState(AbstractState):
        def __init__(self, robot):
            super().__init__(robot)

        def transfer_when_colliding_wall(self):
            super().transfer_when_colliding_wall()
            robot: Robot = self.get_robot()
            action = robot.get_action_from_contour(robot.contours[0])
            if action == robot.go_front:
                robot.state = robot.following_wall_state
            else:
                robot.go_front()

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

    def __init__(self, enable_motion=False, enable_vision=False, enable_auditory=False, enable_olfactory=False, debug_type=None, *args, **kwargs):
        super().__init__(daemon=True, *args, **kwargs)

        self.debug_type = debug_type
        if debug_type == Debug.OTHER:
            import visualization
            self.visualization = visualization

        self.video_capture = None
        if enable_vision:
            self.enable_vision()

        self.track_info = SharedInfo()
        self.pot_info = SharedInfo()
        self.concentration_info = SharedInfo()
        self.motion_info = SharedInfo()

        self.server = ThreadingTCPServer(self, (HOST, PORT), ThreadingTCPRequestHandler)
        self.server.allow_reuse_address = True
        # Start a thread with the server -- that thread will then start one more thread for each request
        server_thread = Thread(target=self.server.serve_forever, daemon=True)
        server_thread.start()
        print(f"Server loop running in thread: {server_thread.name}...")
        
        if enable_motion:
            self.motion_process = subprocess.Popen(("/home/pi/Robot/JY901S_socket", HOST, str(PORT)))
            print(f"Motion sensor client loop running in process: {self.motion_process.pid}...")
        else:
            self.motion_process = None
        
        if enable_auditory:
            self.odas_process = subprocess.Popen(("/home/pi/odas/bin/odaslive", "-c", "/home/pi/odas/bin/odas.cfg"))
            print(f"ODAS system client loop running in process: {self.odas_process.pid}...")
        else:
            self.odas_process = None
        
        if enable_olfactory:
            self.gas_sensor_process = subprocess.Popen(("/home/pi/sgp30-7.1.2/gas_sensor", HOST, str(PORT)))
            print(f"Gas sensor client loop running in process {self.gas_sensor_process.pid}...")
        else:
            self.gas_sensor_process = None

        self.saw_img = None
        self.contours = None
        self.turning_point = False

        self.just_started_state = self.JustStartedState(self)
        self.following_wall_state = self.FollowingWallState(self)
        self.state = self.just_started_state

        self.in_room = False
        self.mission_complete = False

        self.PWMA = 0
        self.AIN1 = 2
        self.AIN2 = 1

        self.PWMB = 5
        self.BIN1 = 3
        self.BIN2 = 4

        self.PWMC = 6
        self.CIN2 = 7
        self.CIN1 = 8

        self.PWMD = 11
        self.DIN1 = 25
        self.DIN2 = 24

        self.GREEN_LED   = 5
        self.RED_LED     = 6
        
        self.LEFT_INFARED = 12
        self.RIGHT_INFARED = 16

        self.PITCH_SERVO = 12
        self.YAW_SERVO   = 13
        
        self.BUTTON      = 19
        
        self.pwm = PCA9685(0x40, debug=False)
        self.pwm.set_PWM_freq(50)
        
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        
        GPIO.setup(self.DIN1, GPIO.OUT)
        GPIO.setup(self.DIN2, GPIO.OUT)
        
        GPIO.setup(self.LEFT_INFARED, GPIO.IN)
        GPIO.setup(self.RIGHT_INFARED, GPIO.IN)
        
        GPIO.setup(self.GREEN_LED, GPIO.OUT)  # 设置绿色Led引脚模式输出
        GPIO.setup(self.RED_LED, GPIO.OUT)    # 设置红色Led引脚模式输出
        
        GPIO.setup(self.BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # 设置输入BtnPin模式，拉高至高电平(3.3V) 
        
    def __enter__(self):
        return self

    def __exit__(self, *args):
        print("Cleaning up...")
        self.stop()
        GPIO.cleanup()
        if self.motion_process is not None:
            self.motion_process.terminate()
        if self.odas_process is not None:
            self.odas_process.terminate()
        if self.gas_sensor_process is not None:
            self.gas_sensor_process.terminate()
        if self.video_capture is not None:
            self.video_capture.release()
            cv.destroyAllWindows()
        self.server.shutdown()
        self.server.server_close()
        print("Done.")

    def turn_degree(self, deg):
        """degree < 0: right, > 0: left"""
        Kp = 40
        goal = normalize_azimuth(int(self.motion_info.get()["azimuth"]["yaw"]) + deg)
        while True:
            current = int(self.motion_info.get()["azimuth"]["yaw"])
            yaw_err = normalize_azimuth(goal - current)
            
            print(f"goal: {goal},\tcurrent: {current},\tyaw_err: {yaw_err}")
            if abs(yaw_err) < 2:
                self.stop(0)
                return
            
            yaw_rate = Kp * abs(yaw_err) // 10
            # 限幅，一个最大的角速度，一个最小的可以转动的
            yaw_rate = yaw_rate if yaw_rate < 40 else 40
            yaw_rate = yaw_rate if yaw_rate > 20 else 20

            if yaw_err > 0:
                self.turn_left(abs(yaw_rate))
            else:
                self.turn_right(abs(yaw_rate))

    def motor_run(self, motor, direction, speed):
        if speed > 100:
            return
        if motor == 0:
            self.pwm.set_duty_cycle(self.PWMA, speed)
            if direction == Direction.FRONT:
                self.pwm.set_level(self.AIN1, 0)
                self.pwm.set_level(self.AIN2, 1)
            else:
                self.pwm.set_level(self.AIN1, 1)
                self.pwm.set_level(self.AIN2, 0)
        elif motor == 1:
            self.pwm.set_duty_cycle(self.PWMB, speed)
            if direction == Direction.FRONT:
                self.pwm.set_level(self.BIN1, 1)
                self.pwm.set_level(self.BIN2, 0)
            else:
                self.pwm.set_level(self.BIN1, 0)
                self.pwm.set_level(self.BIN2, 1)
        elif motor == 2:
            self.pwm.set_duty_cycle(self.PWMC, speed)
            if direction == Direction.FRONT:
                self.pwm.set_level(self.CIN1, 1)
                self.pwm.set_level(self.CIN2, 0)
            else:
                self.pwm.set_level(self.CIN1, 0)
                self.pwm.set_level(self.CIN2, 1)
        elif motor == 3:
            self.pwm.set_duty_cycle(self.PWMD, speed)
            if direction == Direction.FRONT:
                GPIO.output(self.DIN1, 0)
                GPIO.output(self.DIN2, 1)
            else:
                GPIO.output(self.DIN1, 1)
                GPIO.output(self.DIN2, 0)

    def motor_stop(self, motor):
        if motor == 0:
            self.pwm.set_duty_cycle(self.PWMA, 0)
        elif motor == 1:
            self.pwm.set_duty_cycle(self.PWMB, 0)
        elif motor == 2:
            self.pwm.set_duty_cycle(self.PWMC, 0)
        elif motor == 3:
            self.pwm.set_duty_cycle(self.PWMD, 0)

    def go_front(self, speed=40, time=0):
        self.motor_run(0, Direction.FRONT, speed)
        self.motor_run(1, Direction.FRONT, speed)
        self.motor_run(2, Direction.FRONT, speed)
        self.motor_run(3, Direction.FRONT, speed)
        sleep(time)
        # self.stop(0)

    def go_back(self, speed=40, time=0):
        self.motor_run(0, Direction.BACK, speed)
        self.motor_run(1, Direction.BACK, speed)
        self.motor_run(2, Direction.BACK, speed)
        self.motor_run(3, Direction.BACK, speed)
        sleep(time)
        # self.stop(0)

    def go_left(self, speed=40, time=0):
        self.motor_run(0, Direction.BACK, speed)
        self.motor_run(1, Direction.FRONT, speed)
        self.motor_run(2, Direction.FRONT, speed)
        self.motor_run(3, Direction.BACK, speed)
        sleep(time)
        # self.stop(0)

    def go_right(self, speed=40, time=0):
        self.motor_run(0, Direction.FRONT, speed)
        self.motor_run(1, Direction.BACK, speed)
        self.motor_run(2, Direction.BACK, speed)
        self.motor_run(3, Direction.FRONT, speed)
        sleep(time)
        # self.stop(0)

    def turn_left(self, speed=40, time=0):
        self.motor_run(0, Direction.BACK, speed)
        self.motor_run(1, Direction.FRONT, speed)
        self.motor_run(2, Direction.BACK, speed)
        self.motor_run(3, Direction.FRONT, speed)
        sleep(time)
        # self.stop(0)

    def turn_right(self, speed=40, time=0):
        self.motor_run(0, Direction.FRONT, speed)
        self.motor_run(1, Direction.BACK, speed)
        self.motor_run(2, Direction.FRONT, speed)
        self.motor_run(3, Direction.BACK, speed)
        sleep(time)
        # self.stop(0)

    def go_left_front(self, speed=40, time=0):
        self.motor_stop(0)
        self.motor_run(1, Direction.FRONT, speed)
        self.motor_run(2, Direction.FRONT, speed)
        self.motor_stop(0)
        sleep(time)
        # self.stop(0)

    def go_right_front(self, speed=40, time=0):
        self.motor_run(0, Direction.FRONT, speed)
        self.motor_stop(1)
        self.motor_stop(2)
        self.motor_run(3, Direction.FRONT, speed)
        sleep(time)
        # self.stop(0)

    def go_left_back(self, speed=40, time=0):
        self.motor_run(0, Direction.BACK, speed)
        self.motor_stop(1)
        self.motor_stop(2)
        self.motor_run(3, Direction.BACK, speed)
        sleep(time)
        # self.stop(0)

    def go_right_back(self, speed=40, time=0):
        self.motor_stop(0)
        self.motor_run(1, Direction.BACK, speed)
        self.motor_run(2, Direction.BACK, speed)
        self.motor_stop(3)
        sleep(time)
        # self.stop(0)

    def stop(self, time=0):
        self.motor_stop(0)
        self.motor_stop(1)
        self.motor_stop(2)
        self.motor_stop(3)
        sleep(time)

    def set_servo_pulse(self, channel, pulse):
        pulse_length = 1000000    # 1,000,000 us per second
        pulse_length //= 60       # 60 Hz
        print('{0}us per period'.format(pulse_length))
        pulse_length //= 4096     # 12 bits of resolution
        print('{0}us per bit'.format(pulse_length))
        pulse *= 1000
        pulse //= pulse_length
        self.pwm.set_PWM(channel, 0, pulse)

    def set_servo_angle(self, channel, angle):
        angle = 4096 * (angle * 11 + 500) // 20000
        self.pwm.set_PWM(channel, 0, angle)

    def wait_for_button_pressed(self):
        print("When ready, press the button on the robot to start...")
        pressed = GPIO.input(self.BUTTON)
        if self.video_capture is None:
            while not pressed:
                pressed = GPIO.input(self.BUTTON)
            while pressed:
                pressed = GPIO.input(self.BUTTON)
                if pressed:
                    GPIO.output(self.RED_LED, 1)
                else:
                    GPIO.output(self.RED_LED, 0)
        else:
            while not pressed:
                pressed = GPIO.input(self.BUTTON)
                ret, frame = self.video_capture.read()
                if not ret:
                    print("Can't receive frame (stream end?). Exiting...")
                    exit(0)
                if self.debug_type == Debug.VISION:
                    cv.imshow("Vision", frame)
                    if cv.waitKey(1) == ord('q'):
                        break
            while pressed:
                pressed = GPIO.input(self.BUTTON)
                if pressed:
                    GPIO.output(self.RED_LED, 1)
                else:
                    GPIO.output(self.RED_LED, 0)
                _, frame = self.video_capture.read()
                if not ret:
                    print("Can't receive frame (stream end?). Exiting...")
                    exit(0)
                if self.debug_type == Debug.VISION:
                    cv.imshow("Vision", frame)
                    if cv.waitKey(1) == ord('q'):
                        break
            if self.debug_type == Debug.OTHER:
                self.visualization.mainWindow.setWindowTitle("JustStartedState")
            else:
                print(self)

    def go_following_wall(self):
        if self.contours is None:
            self.go_front()
        else:
            # assert len(self.contours) == 1
            action = self.get_action_from_contour(self.contours[0])
            action()

    def is_colliding_wall(self):
        return len(self.contours) >= 1

    def is_colliding_another_robot(self):
        return False
        # return self.is_colliding_left() or self.is_colliding_right()

    def is_visiting_turning_point(self):
        return self.turning_point

    def is_revisiting_places(self):
        return self.concentration_info.get()["CO2"] > 2000
        # return False  # TODO

    def is_leaving_gathering_circle(self):
        return False  # TODO

    def is_finish_gathering(self):
        return False  # TODO

    def is_found_injuries(self):
        return False  # TODO

    def is_colliding_left(self):
        return not GPIO.input(self.LEFT_INFARED)
    
    def is_colliding_right(self):
        return not GPIO.input(self.RIGHT_INFARED)

    def get_action_from_contour(self, c):
        M = cv.moments(c)
        if M['m00'] == 0:
            return self.go_front
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        
        cv.line(self.saw_img, (cx, 0), (cx, HEIGHT), BLUE, 1)
        cv.line(self.saw_img, (0, cy), (WIDTH, cy), BLUE, 1)

        if cx > IMG_RIGHT_THRESH:
            return self.turn_right
        elif cx < IMG_LEFT_THRESH:
            return self.turn_left
        else:
            return self.go_front
    
    def get_azimuths_from_sound(self):
        self.stop(0)
        return_value = set()
        while len(return_value) == 0:
            infos = self.track_info.get()
            for info in infos:
                if info["tag"] == "dynamic" and info["activity"] > 0.5:
                    return_value.add(int(math.atan2(info["y"], info["x"])))
        return return_value

    def enable_vision(self):
        if self.video_capture is None:
            self.video_capture = cv.VideoCapture(0)
            self.video_capture.set(cv.CAP_PROP_FRAME_WIDTH, WIDTH)
            self.video_capture.set(cv.CAP_PROP_FRAME_HEIGHT, HEIGHT)
            if not self.video_capture.isOpened():
                exit("Cannot open camera.")

    def run(self):
        self.enable_vision()
        # self.video_capture.set(cv.CAP_PROP_FRAME_WIDTH, WIDTH) 
        # self.video_capture.set(cv.CAP_PROP_FRAME_HEIGHT, HEIGHT)
        self.set_servo_angle(self.PITCH_SERVO, 105)
        self.set_servo_angle(self.YAW_SERVO, 100)
        sleep(20)
        # self.wait_for_button_pressed()
        
        while True:
            ret, self.saw_img = self.video_capture.read()
            if not ret:
                print("Can't receive frame (stream end?). Exiting...")
                return
            # cropped_img = frame[CROPPED_HEIGHT:HEIGHT, 0:WIDTH]
            grey_img = cv.cvtColor(self.saw_img, cv.COLOR_BGR2GRAY)
            blur_img = cv.GaussianBlur(grey_img, (5, 5), 0)
            _, thresh_img = cv.threshold(blur_img, 60, 255, cv.THRESH_BINARY_INV)  # Auto thresholding)
            morph_img = cv.morphologyEx(thresh_img, cv.MORPH_OPEN, OPENING_KERNEL)  # Remove background dots
            morph_img = cv.morphologyEx(morph_img, cv.MORPH_CLOSE, CLOSING_KERNEL)  # Remove foreground dots
            cropped_img = morph_img[CROPPED_HEIGHT:HEIGHT, 0:WIDTH]
            self.contours, _ = cv.findContours(cropped_img, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
            edge_img = cv.Canny(morph_img, 100, 300, 3)
            edge_contours, _ = cv.findContours(edge_img, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
            if len(edge_contours) > 0:
                cv.drawContours(self.saw_img, edge_contours, -1, GREEN, 1)
            for contour in edge_contours:
                if len(contour) > 25:  # Can be changed
                    S1 = cv.contourArea(contour)
                    ellipse = cv.fitEllipse(contour)
                    S2 = math.pi * ellipse[1][0] * ellipse[1][1]
                    if S2 != 0 and (S1 / S2) > 0.2:  # Can be changed
                        cv.ellipse(self.saw_img, ellipse, RED, 1)
                        self.turning_point = True
                        print("Turning Point!")
                        break
            else:
                self.turning_point = False
            self.state.transfer_to_next_state()
            # if len(contours) == 1:
            #     action = self.get_action_from_contour(cropped_img, contours[0])
            #     action()
            #     cv.drawContours(cropped_img, contours, -1, GREEN, 1)
            # elif len(contours) > 1:
            #     # c = max(contours, key=cv2.contourArea)
            #     possible_actions = {self.get_action_from_contour(cropped_img, c) for c in contours}
            #     less_preferred_actions = self.get_less_preferred_actions_from_sound()
            #     preferred_possible_actions = possible_actions - less_preferred_actions
            #     if (len(preferred_possible_actions)) != 0:
            #         action = preferred_possible_actions.pop()
            #     else:
            #         action = possible_actions.pop()
            #     action()
            #     cv.drawContours(cropped_img, contours, -1, GREEN, 1)
            # else:
            #     self.stop()
            #     print("I can't see any lines!")
            if self.debug_type == Debug.VISION:
                cv.imshow("Vision", self.saw_img)
                if cv.waitKey(1) == ord('q'):
                    break


if __name__ == '__main__':
    with Robot(enable_motion=True, enable_vision=True, enable_auditory=True, enable_olfactory=True, debug_type=Debug.VISION) as robot:
        if robot.debug_type == Debug.OTHER:
            robot.start()
            robot.visualization.run()
        else:
            robot.run()
