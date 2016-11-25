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
    rospy.init_node('boundingbox_to_transform_stamped')
    rospy.Subscriber("/raw_tablecloth_bb", BoundingBoxArray, sub_bb)
    pub = rospy.Publisher("/tablecloth_bb", BoundingBoxArray)
    margin_x = 0.5
    margin_y = 0
    margin_z = 0.5
    bba = BoundingBoxArray()
    bb = BoundingBox()
    rospy.spin()
