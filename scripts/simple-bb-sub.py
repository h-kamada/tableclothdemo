#!/usr/bin/env python
import rospy
import math
import tf
from geometry_msgs.msg import PoseStamped
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray

def sub_bb (msg):
    rospy.loginfo ("sub_bb cb")
    bbpub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('sub_bb')
    rospy.Subscriber("/raw_tablecloth_bb", BoundingBoxArray , sub_bb)
    bbpub = rospy.Publisher ('bb_repub' , BoundingBoxArray)
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        rospy.spin()
        r.sleep()
