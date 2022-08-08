#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from robot import *
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class RobotUsingRos(Robot):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.controlloer_publisher = None
    
    def __exit__(self, *args):
        print("Cleaning up...")
        self.stop_event.set()
        self.pump_stop_event.set()
        GPIO.output(PUMP_CTRL_PIN, 0)
        GPIO.cleanup()
        if self.controller:
            self.stop()
            self.controller.terminate()
        if self.controlloer_publisher:
            self.controlloer_publisher.unregister()
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

    def setup_controller(self):
        if self.controller is None:
            self.controller = subprocess.Popen(("roslaunch", "lingao_bringup", "bringup.launch"))
            self.controlPanel.controllerButton.setText("Disconnect")
            self.controlPanel.setControlButtonsEnabled(True)
            self.controlloer_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        else:
            self.controller.terminate()
            self.controller = None
            self.controlloer_publisher.unregister()
            self.controlloer_publisher = None
            self.controlPanel.controllerButton.setText("Connect")
            self.controlPanel.setControlButtonsEnabled(False)

    def set_speed(self, x, z):
        vel_msg = Twist()
        vel_msg.linear.x = x
        vel_msg.angular.z = z
        self.controlloer_publisher.publish(vel_msg)


if __name__ == '__main__':
    with RobotUsingRos() as robot:
        robot.exec()
