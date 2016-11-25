#!/usr/bin/env python
import rospy
import math
import tf
from geometry_msgs.msg import *
from visualization_msgs.msg import MarkerArray, Marker

def sub_table (msg):
    rospy.loginfo ("sub table")
    marker = msg.markers[1]
    if len(xl) < 10:
        xl.append(marker.pose.position.x)
        yl.append(marker.pose.position.y)
        rl.append(marker.pose.orientation.z)
    else:
        static_x = sum(xl) / len(xl)
        static_y = sum(yl) / len(yl)
        static_r = sum(rl) / len(rl)
        pose.position.x = static_x
        pose.position.y = static_y
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = static_r
        pose.orientation.w = math.sqrt(1 - static_r ** 2)
        # transform
        posestamped.header = marker.header
        posestamped.pose = pose
        listener.waitForTransform("/map", "/base_footprint", marker.header.stamp, rospy.Duration(4.0))
        tmp_posestamped = listener.transformPose('/map', posestamped)
        posearray.poses = [tmp_posestamped.pose]
        posearray.header.stamp = marker.header.stamp
        posearray.header.frame_id = "/map"
        pub.publish(posearray)
        norm = (marker.pose.position.x - static_x) ** 2 + (marker.pose.position.y - static_y) ** 2
        rospy.loginfo ("norm:%s", norm)
        # set new value
        xl.pop(0)
        yl.pop(0)
        rl.pop(0)
        xl.append(marker.pose.position.x)
        yl.append(marker.pose.position.y)
        rl.append(marker.pose.orientation.z)

if __name__ ==  '__main__':
    xl = []
    yl = []
    rl = []
    posearray = PoseArray()
    pose = Pose()
    posestamped = PoseStamped()
    norm_thre = 15
    static_x = 0
    static_y = 0
    rospy.init_node("table_to_static")
    pub = rospy.Publisher("/table_static", PoseArray)
    rospy.Subscriber("/table_marker_array", MarkerArray, sub_table)
    listener = tf.TransformListener()
    r = rospy.Rate(10)
    # import ipdb; ipdb.set_trace();
    rospy.spin()
