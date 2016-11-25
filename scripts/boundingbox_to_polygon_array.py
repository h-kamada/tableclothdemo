#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import *
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray, PolygonArray

def sub_bb (msg):
    rospy.loginfo("sub_bb cb")
    bb = msg.boxes[0]
    x1 = bb.pose.position.x - bb.dimensions.x
    x2 = bb.pose.position.x + bb.dimensions.x
    y1 = bb.pose.position.y - bb.dimensions.y
    y2 = bb.pose.position.y + bb.dimensions.y
    z1 = bb.pose.position.z - bb.dimensions.z
    z2 = bb.pose.position.z + bb.dimensions.z
    ps = PolygonStamped()
    if face == "xy":
        ps.polygon.points.append(Point32(x1, y1, z1))
        ps.polygon.points.append(Point32(x1, y2, z1))
        ps.polygon.points.append(Point32(x2, y2, z1))
        ps.polygon.points.append(Point32(x2, y1, z1))
    elif face == "yz":
        ps.polygon.points.append(Point32(x1, y1, z1))
        ps.polygon.points.append(Point32(x1, y1, z2))
        ps.polygon.points.append(Point32(x1, y2, z2))
        ps.polygon.points.append(Point32(x1, y2, z1))
    elif face == "zx":
        ps.polygon.points.append(Point32(x1, y1, z1))
        ps.polygon.points.append(Point32(x1, y1, z2))
        ps.polygon.points.append(Point32(x2, y1, z2))
        ps.polygon.points.append(Point32(x2, y1, z1))
    else:
        rospy.loginfo ("Face Error!")
    ps.header = msg.header
    pa = PolygonArray()
    pa.header = msg.header
    pa.polygons = [ps]
    rospy.loginfo('pa:%s', pa)
    pub.publish(pa)

if __name__ ==  '__main__':
    rospy.init_node("bounding_box_to_polygon_array")
    pub = rospy.Publisher("output", PolygonArray)
    rospy.Subscriber("/tablecloth_bb", BoundingBoxArray, sub_bb)
    r = rospy.Rate(10)
    bb = BoundingBox()
    pa = PolygonArray()
    face = rospy.get_param("face", "xy")
    rospy.spin()
