#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import PoseStamped
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray
rospy.init_node("attention_pose_set")
pub = rospy.Publisher("/tablecloth_bb", BoundingBoxArray)
r = rospy.Rate(10)
theta = 0
while not rospy.is_shutdown():
    boxes = BoundingBoxArray()
    theta = math.fmod(theta + 0.1, math.pi * 2)
    theta = 0
    box = BoundingBox()
    box.header.stamp = rospy.Time.now()
    box.header.frame_id = "map"
    box.pose.orientation.w = 1
    box.pose.position.x = 0.8 * math.cos(theta)
    box.pose.position.y = 0.8 * math.sin(theta)
    box.pose.position.z = 1.1
    box.dimensions.x = 1.2
    box.dimensions.y = 1.2
    box.dimensions.z = 1.2
    box2 = BoundingBox()
    box2.header.stamp = rospy.Time.now()
    box2.header.frame_id = "map"
    box2.pose.orientation.w = 1
    box2.pose.position.x = 0.8 * math.cos(theta)
    box2.pose.position.y = 0.8 * math.sin(theta)
    box2.pose.position.z = 0.9
    box2.dimensions.x = 0.1
    box2.dimensions.y = 0.1
    box2.dimensions.z = 0.1
    boxes.boxes = [box, box2]
    boxes.header.frame_id = "map"
    boxes.header.stamp = rospy.Time.now()
    pub.publish(boxes)
    r.sleep()
