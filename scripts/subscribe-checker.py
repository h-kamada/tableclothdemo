#!/usr/bin/env python
import rospy
import math
import tf
from geometry_msgs.msg import PoseStamped

def sub_checker (msg):
    rospy.loginfo ("x:%s", msg.pose.position.x)
    rospy.loginfo ("y:%s", msg.pose.position.y)
    rospy.loginfo ("z:%s", msg.pose.position.z)
    rospy.loginfo ("msg:%s", msg)
    rospy.loginfo ("trans pose:%s", listener.transformPose('/base_footprint', msg))
    posepub.publish(listener.transformPose('/base_footprint', msg))

if __name__ == '__main__':
    rospy.init_node('checkerboard_listener')
    rospy.Subscriber("/checkerboard_detector0/objectdetection_pose", PoseStamped, sub_checker)
    # rospy.Subscriber("/checkerboard_detector1/objectdetection_pose", PoseStamped, sub_checker)
    posepub = rospy.Publisher ('checkerboard_pose' , PoseStamped)
    listener = tf.TransformListener()
    # rate = rospy.Rate(10)
    rospy.spin()
