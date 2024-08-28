#!/usr/bin/env python
# encoding: utf-8

import rospy
import threading
import cv2 as cv
from time import sleep, time
from transport_common import ROSNav
from dynamic_reconfigure.server import Server
from dynamic_reconfigure.client import Client
from arm_autopilot.cfg import AutoPilotPIDConfig

class AutoColorTransport:
    def __init__(self):
        rospy.init_node('auto_color_transport', anonymous=False)
        
        # ROS navigation and control
        self.ros_nav = ROSNav()
        self.ros_ctrl = ROSCtrl()
        
        # Initialize parameters
        self.model = "Init"
        self.color_name = {}
        self.Grip_status = False
        
        # Arm and motion control setup
        self.joints_init = [90, 120, 0, 0, 90, 30]
        self.ros_ctrl.pubArm(self.joints_init)

        # HSV color settings for detection
        self.color_hsv_ranges = {
            'red': ((0, 100, 100), (10, 255, 255)),
            'yellow': ((23, 100, 100), (56, 255, 255)),
            'green': ((35, 100, 100), (78, 255, 255)),
            'blue': ((100, 100, 100), (124, 255, 255))
        }
        
        # Set up OpenCV window
        self.windows_name = 'frame'
        cv.namedWindow(self.windows_name, cv.WINDOW_AUTOSIZE)

    def detect_color(self, img):
        # Convert image to HSV and detect colors
        hsv_img = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        detected_color = None
        
        for color, (lower_hsv, upper_hsv) in self.color_hsv_ranges.items():
            mask = cv.inRange(hsv_img, lower_hsv, upper_hsv)
            if cv.countNonZero(mask) > 0:
                detected_color = color
                break
        
        return detected_color

    def process_frame(self, frame, action):
        if self.model == "Grip":
            if not self.Grip_status:
                detected_color = self.detect_color(frame)
                if detected_color:
                    self.color_name = detected_color
                    threading.Thread(target=self.grip_target).start()
        elif self.model == "Transport":
            if self.color_name in self.ros_nav.color_pose.keys():
                self.ros_nav.PubTargetPoint(self.ros_nav.color_pose[self.color_name])
                self.model = "Grip_down"
        elif self.model == "Grip_down":
            if self.ros_nav.goal_result == 3:
                threading.Thread(target=self.grip_down).start()
        elif self.model == "come_back":
            if self.ros_nav.goal_result == 3:
                threading.Thread(target=self.buzzer_loop).start()
                self.reset()
        elif action == 32 or self.ros_ctrl.joy_action == 2:
            self.model = "Grip"
            self.ros_ctrl.pubArm(self.joints_init)
        elif action == ord('r') or action == ord('R'):
            self.reset()
        elif action == ord('q') or action == ord('Q'):
            self.cancel()
        
        cv.putText(frame, f'Model: {self.model}', (30, 450), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1)
        self.ros_nav.pubImg(frame)
        return frame

    def grip_down(self):
        self.model = "Grip_down"
        self.ros_nav.goal_result = 0
        joints = [90, 2.0, 60.0, 40.0, 90, 140]
        self.ros_nav.pubArm(joints, run_time=1000)
        sleep(1)
        self.ros_nav.pubArm([], 6, 30)
        sleep(0.5)
        joints = [90, 145, 0, 45, 90, 30]
        self.ros_nav.pubArm(joints, run_time=1000)
        sleep(1)
        self.comeback()

    def grip_target(self):
        self.model = "Grip_Target"
        self.Grip_status = True
        self.buzzer_loop()
        joints = [90, 145, 0, 45, 90, 30]
        self.ros_nav.pubArm(joints, run_time=1000)
        sleep(0.5)
        self.buzzer_loop()
        self.ros_nav.pubArm([], 6, 146)
        sleep(1)
        self.model = "Transport"

    def buzzer_loop(self):
        self.ros_nav.pubBuzzer(True)
        sleep(1)
        self.ros_nav.pubBuzzer(False)
        sleep(1)

    def comeback(self):
        self.ros_nav.PubTargetPoint(self.ros_nav.start_point)
        self.model = "come_back"

    def reset(self):
        self.ros_nav.goal_result = 0
        self.model = "Init"
        self.Grip_status = False
        self.color_name = {}

    def cancel(self):
        self.reset()
        self.ros_nav.cancel()
        print("Shutting down this node.")

if __name__ == '__main__':
    color_transport = AutoColorTransport()
    capture = cv.VideoCapture(1)
    cv_edition = cv.__version__
    if cv_edition[0] == '3':
        capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*'XVID'))
    else:
        capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter.fourcc('Y', 'U', 'Y', 'V'))

    capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)

    while capture.isOpened():
        start = time()
        ret, frame = capture.read()
        action = cv.waitKey(10) & 0xFF
        if action == ord('q') or action == 113:
            break
        frame = color_transport.process_frame(frame, action)
        if color_transport.ros_nav.img_show:
            cv.imshow("frame", frame)
    capture.release()
    cv.destroyAllWindows()
#despite the fact disastr management technologies have advanced over the years