#!/usr/bin/env python
# encoding: utf-8
import rospy
from numpy import *
from geometry_msgs.msg import *
from actionlib_msgs.msg import GoalID
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import quaternion_from_euler, euler_from_quaternion

class DrawMarker:
    def __init__(self):
        rospy.on_shutdown(self.cancel)
        self.InitialParam()
        self.pub_markerArray = rospy.Publisher('color_end_pose', MarkerArray, queue_size=100)
        self.pub_cancel = rospy.Publisher("move_base/cancel", GoalID, queue_size=10)
        self.sub_navgoal = rospy.Subscriber('transport/goal', PoseStamped, self.navgoal_callback)
        self.sub_start_pt = rospy.Subscriber('/start_point', PoseWithCovarianceStamped, self.startpt_callback)
        self.sub_click_pt = rospy.Subscriber('clicked_point', PointStamped, self.click_pt_callback)

    def InitialParam(self):
        pst = PoseStamped()
        pst.pose.orientation.w = 1
        self.id = 0
        self.start_point = pst
        self.goal_result = None
        self.markerArray = MarkerArray()

    def startpt_callback(self, msg):
        if not isinstance(msg, PoseWithCovarianceStamped): return
        self.start_point = msg.pose.pose
        scale = [0.3, 0.01, 0.01]
        self.add_marker(0, scale, msg.pose.pose, [1, 0, 0], 5)
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        roll, pitch, yaw = euler_from_quaternion([x, y, z, w])
        quat = quaternion_from_euler(roll, pitch, yaw + pi / 2)
        stamped = PoseStamped()
        q = Quaternion()
        q.x, q.y, q.z, q.w = quat
        stamped.pose.position = msg.pose.pose.position
        stamped.pose.orientation = q
        self.add_marker(0, scale, stamped.pose, [0, 1, 0], 6)

    def navgoal_callback(self, msg):
        if not isinstance(msg, PoseStamped): return
        scale = [0.2, 0.03, 0.03]
        if self.id == 0: rgb = [1, 0, 0]
        elif self.id == 1: rgb = [0, 1, 0]
        elif self.id == 2: rgb = [0, 0, 1]
        else: rgb = [1, 1, 0]
        self.add_marker(0, scale, msg.pose, rgb, self.id)
        if self.id >= 3: self.id = 0
        else: self.id += 1

    def add_marker(self, type, scale, pose, rgb, id):
        marker = Marker()
        marker.header.frame_id = 'map'
        # marker model
        marker.type = type
        marker.action = marker.ADD
        # the size of the marker
        marker.scale.x = scale[0]
        marker.scale.y = scale[1]
        marker.scale.z = scale[2]
        # marker ColorRGBA
        marker.color.r = rgb[0]
        marker.color.g = rgb[1]
        marker.color.b = rgb[2]
        marker.color.a = 1
        # marker position XYZ
        marker.pose = pose
        marker.id = id
        for i in self.markerArray.markers:
            if i.id == id: self.markerArray.markers.pop(self.markerArray.markers.index(i))
        self.markerArray.markers.append(marker)

    def click_pt_callback(self, msg):
        if not isinstance(msg, PointStamped): return
        self.InitialParam()
        marker = Marker()
        marker.action = marker.DELETEALL
        self.markerArray.markers.append(marker)
        self.pub_markerArray.publish(self.markerArray)
        self.pub_cancel.publish(GoalID())
        self.markerArray = MarkerArray()

    def cancel(self):
        self.pub_markerArray.unregister()
        self.sub_start_pt.unregister()
        self.sub_click_pt.unregister()

if __name__ == '__main__':
    rospy.init_node("draw_marker", anonymous=False)
    draw_marker = DrawMarker()
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        r.sleep()
        draw_marker.pub_markerArray.publish(draw_marker.markerArray)

