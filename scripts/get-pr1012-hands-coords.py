#!/usr/bin/env python
import rospy
# import math
import tf
# from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point, Quaternion
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray

if __name__ == '__main__':
    rospy.init_node('checkerboard_listener')
    listener = tf.TransformListener()
    rate = rospy.Rate(10)
    box = BoundingBox()
    bba = BoundingBoxArray()
    bbapub = rospy.Publisher ('boundingboxarray', BoundingBoxArray)
    box.dimensions.x = 0.1
    box.dimensions.y = 1
    box.dimensions.z = 1
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('/map', '/r_gripper_palm_link', rospy.Time(0))
            # import ipdb; ipdb.set_trace();
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        rospy.loginfo ("trans:%s rot:%s", trans , rot)
        box.header.stamp = rospy.Time.now()
        box.header.frame_id = "/map"
        box.pose.position = Point(*trans)
        print box.pose.position
        box.pose.position.x += 0
        box.pose.position.y += (box.dimensions.y / 2)
        box.pose.position.z += (box.dimensions.z / 2)
        print box.pose.position
        # box.pose.position = Point(x=trans[0], y=trans[1], z=trans[2])
        box.pose.orientation = Quaternion(*rot)
        print box
        bba.header = box.header
        bba.boxes = [box]
        bbapub.publish(bba)
        rate.sleep()
