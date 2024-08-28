#!/usr/bin/env python
# encoding: utf-8
import rospy
import threading
from time import sleep
from cv_bridge import CvBridge
from geometry_msgs.msg import *
from move_base_msgs.msg import *
from yahboomcar_msgs.msg import *
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Int32
from actionlib_msgs.msg import GoalID
from visualization_msgs.msg import MarkerArray

class ROSNav:
    def __init__(self):
        rospy.on_shutdown(self.cancel)
        self.InitialParam()
        self.img_show = rospy.get_param("~img_show", True)
        self.pub_CmdVel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pub_Arm = rospy.Publisher("TargetAngle", ArmJoint, queue_size=1000)
        self.pub_goal = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
        self.pub_cancel = rospy.Publisher("move_base/cancel", GoalID, queue_size=10)
        self.pub_buzzer = rospy.Publisher('/Buzzer', Bool, queue_size=1)
        self.pub_rgb = rospy.Publisher("/Transport/rgb", Image, queue_size=1)
        self.sub_RGBLight = rospy.Subscriber("RGBLight", Int32, self.RGBLightcallback, queue_size=100)
        self.sub_markerArray = rospy.Subscriber('color_end_pose', MarkerArray, self.getMarker_callback)
        self.sub_goal_result = rospy.Subscriber('move_base/result', MoveBaseActionResult, self.goal_result_callback)

    def InitialParam(self):
        pst = PoseStamped()
        pst.pose.orientation.w = 1
        self.color_pose = {}
        self.start_point = pst.pose
        self.goal_result = 0
        self.joy_action = 0
        self.Transport_status = False
        self.markerArray = MarkerArray()
        self.color_name_list = ['red', 'green', 'blue', 'yellow']

    def RGBLightcallback(self, msg):
        if not isinstance(msg, Int32): return
        threading.Thread(target=self.joy_action_update,).start()

    def joy_action_update(self):
        self.joy_action += 1
        sleep(0.5)
        self.joy_action = 0

    def PubTargetPoint(self, goal_pose):
        self.Transport_status = True
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = rospy.Time.now()
        pose.pose = goal_pose
        self.pub_goal.publish(pose)

    def getMarker_callback(self, msg):
        if not isinstance(msg, MarkerArray): return
        self.color_pose = {}
        for i in msg.markers:
            if i.id == 0: self.color_pose[self.color_name_list[i.id]] = i.pose
            if i.id == 1: self.color_pose[self.color_name_list[i.id]] = i.pose
            if i.id == 2: self.color_pose[self.color_name_list[i.id]] = i.pose
            if i.id == 3: self.color_pose[self.color_name_list[i.id]] = i.pose
            if i.id == 5: self.start_point = i.pose

    def goal_result_callback(self, msg):
        if not isinstance(msg, MoveBaseActionResult): return
        if msg.status.SUCCEEDED == 3:self.Transport_status = False
        self.goal_result = msg.status.SUCCEEDED

    def pubVel(self, x, y, z=0):
        twist = Twist()
        twist.linear.x = x
        twist.linear.y = y
        twist.angular.z = z
        self.pub_CmdVel.publish(twist)
        self.RobotRun_status = False

    def pubImg(self, img):
        self.bridge = CvBridge()
        self.pub_rgb.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))

    def pubBuzzer(self, status):
        self.pub_buzzer.publish(status)
        self.Buzzer_state = False

    def pubArm(self, joints, id=10, angle=90, run_time=500):
        armjoint = ArmJoint()
        armjoint.run_time = run_time
        if len(joints) != 0: armjoint.joints = joints
        else:
            armjoint.id = id
            armjoint.angle = angle
        self.pub_Arm.publish(armjoint)

    def cancel(self):
        self.pub_cancel.publish(GoalID())
        self.pubVel(0, 0, 0)
        self.pub_goal.unregister()
        self.pub_CmdVel.unregister()
        self.pub_Arm.unregister()
        self.pub_buzzer.unregister()
        self.pub_cancel.unregister()
        self.sub_goal_result.unregister()
        self.sub_markerArray.unregister()

