#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose, PoseArray, Point
import sensor_msgs.point_cloud2 as pc2

def pc_cb (msg):
    flag = True
    for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
        if flag:
            pose.position= Point(*point)
            up_left.position = pose.position
            up_right.position = pose.position
            down_left.position = pose.position
            down_right.position = pose.position
            flag = False
        if (  point[1] + point[2] >=      up_left.position.y +    up_left.position.z):
            up_left.position = Point(*point)
        if ( -point[1] + point[2] >=    -up_right.position.y +   up_right.position.z):
            up_right.position = Point(*point)
        if (  point[1] - point[2] >=    down_left.position.y -  down_left.position.z):
            down_left.position = Point(*point)
        if (- point[1] - point[2] >= - down_right.position.y - down_right.position.z):
            down_right.position = Point(*point)
    # import ipdb; ipdb.set_trace();
    posearray.header = msg.header
    upleft_array.header = msg.header
    upright_array.header = msg.header
    downleft_array.header = msg.header
    downright_array.header = msg.header
    posearray.poses = [up_left, up_right, down_left, down_right]
    upleft_array.poses = [up_left]
    upright_array.poses = [up_right]
    downleft_array.poses = [down_left]
    downright_array.poses = [down_right]
    # rospy.loginfo("   up left: %s %s %s", up_left.position.x, up_left.position.y, up_left.position.z)
    # rospy.loginfo("  up right: %s %s %s", up_right.position.x, up_right.position.y, up_right.position.z)
    # rospy.loginfo(" down left: %s %s %s", down_left.position.x, down_left.position.y, down_left.position.z)
    # rospy.loginfo("down right: %s %s %s", down_right.position.x, down_right.position.y, down_right.position.z)
    pub.publish(posearray)
    upleft.publish(upleft_array)
    upright.publish(upright_array)
    downleft.publish(downleft_array)
    downright.publish(downright_array)

rospy.init_node("attention_pose_set")
r = rospy.Rate(1)
sub = rospy.Subscriber ("/transformed_points", PointCloud2, pc_cb)

pose = Pose()
up_left = Pose()
up_right = Pose()
down_left = Pose()
down_right = Pose()
pose.orientation.x = 0
pose.orientation.y = 0
pose.orientation.z = 0
pose.orientation.w = 1
posearray = PoseArray()
upleft_array = PoseArray()
upright_array = PoseArray()
downleft_array = PoseArray()
downright_array = PoseArray()
up_left.orientation = pose.orientation
up_right.orientation = pose.orientation
down_left.orientation = pose.orientation
down_right.orientation = pose.orientation
pub = rospy.Publisher ("four_corners", PoseArray)
upleft = rospy.Publisher ("up_left_corner", PoseArray)
upright = rospy.Publisher ("up_right_corner", PoseArray)
downleft = rospy.Publisher ("down_left_corner", PoseArray)
downright = rospy.Publisher ("down_right_corner", PoseArray)

rospy.spin()

