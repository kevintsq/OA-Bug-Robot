#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from robot import *
from geometry_msgs.msg import Twist


class RobotUsingRos(Robot):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.publisher = None
    
    def __exit__(self, *args):
        print("Cleaning up...")
        if self.controller:
            self.stop()
            self.controller.terminate()
        if self.publisher:
            self.publisher.unregister()
        # GPIO.cleanup()
        if self.subscriber:
            self.subscriber.unregister()
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

    def setup_controller(self):
        if self.controller is None:
            self.controller = subprocess.Popen(("roslaunch", "lingao_bringup", "bringup.launch"))
            self.controlPanel.controllerButton.setText("Disconnect")
            self.controlPanel.setControlButtonsEnabled(True)
            self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        else:
            self.controller.terminate()
            self.controller = None
            self.controlPanel.controllerButton.setText("Connect")
            self.controlPanel.setControlButtonsEnabled(False)

    def set_speed(self, x, z):
        vel_msg = Twist()
        vel_msg.linear.x = x
        vel_msg.angular.z = z
        self.publisher.publish(vel_msg)


if __name__ == '__main__':
    with RobotUsingRos() as robot:
        robot.exec()
