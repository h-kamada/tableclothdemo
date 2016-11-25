#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import *
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray

def sub_bb (msg):
    br = tf.TransformBroadcaster()
    bb = msg.boxes[0]
    br.sendTransform((bb.pose.position.x, bb.pose.position.y, bb.pose.position.z),
                     (bb.pose.orientation.x, bb.pose.orientation.y, bb.pose.orientation.z, bb.pose.orientation.w), bb.header.stamp, "/tablecloth_boundingbox", "/map")

if __name__ == '__main__':
    rospy.init_node('boundingbox_to_transform_stamped')
    rospy.Subscriber("/tablecloth_bb", BoundingBoxArray, sub_bb)
    rospy.spin()
