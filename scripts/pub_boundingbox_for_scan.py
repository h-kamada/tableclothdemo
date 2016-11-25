#!/usr/bin/env python
import rospy
from geometry_msgs.msg import *
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray

def sub_bb (msg):
    bba.header = msg.header
    bb = msg.boxes[0]
    bb.dimensions.x = bb.dimensions.x + margin_x
    bb.dimensions.y = bb.dimensions.y + margin_y
    bb.dimensions.z = bb.dimensions.z + margin_z
    bba.boxes = [bb]
    pub.publish(bba)

if __name__ == '__main__':
    rospy.init_node('pub_boundingbox_for_scan')
    pub = rospy.Publisher("~box", BoundingBoxArray)
    bba = BoundingBoxArray()
    bb = BoundingBox()
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        bb.header.stamp = rospy.Time.now()
        bb.header.frame_id = "/map"
        bb.pose.position.x = 3.7
        bb.pose.position.y = 8.3
        bb.pose.position.z = 0.3
        bb.pose.orientation.x = 0
        bb.pose.orientation.y = 0
        bb.pose.orientation.z = 0
        bb.pose.orientation.w = 1
        bb.dimensions.x = 1.8
        bb.dimensions.y = 2.0
        bb.dimensions.z = 0.2
        bba.boxes = [bb]
        bba.header = bb.header
        pub.publish(bba)
        r.sleep()

