#!/usr/bin/env python

#-*- coding: utf-8 -*-

import sys
import roslib
import rospy
import os.path
import math
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
from std_msgs.msg import Int16
import time
import signal
from std_msgs.msg import String


def arCallback(data):
    rospy.logdebug(rospy.get_name() + ": Received Quaternion")
    rospy.logdebug(data)

    if not data.markers:
        return
    data = data.markers[0]

    print "Position: (x, y, z)->(% 6.4f,% 6.4f,% 6.4f)" % (data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z)

    print "Rotation: (q, x, y, z) -> (%6.4f, % 6.4f,% 6.4f,% 6.4f)" %  (data.pose.pose.orientation.w, data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z)

    if data.pose.pose.orientation.z < 0:
        Angle = (math.acos(data.pose.pose.orientation.x) + -1 * math.asin(data.pose.pose.orientation.z)) * 180 / math.pi
    else:
        Angle = (math.acos(data.pose.pose.orientation.x) + math.asin(data.pose.pose.orientation.z)) * 180 / math.pi
    distOut = math.cos(Angle) * data.pose.pose.position.z * 3.28 * 12;
    distOffset = math.sin(Angle) * data.pose.pose.position.z * 3.28 * 12;
    distCrow = data.pose.pose.position.z * 3.28 * 12;

    CrowError = (0.094 * distCrow + .0411)
    distCrow = distCrow - CrowError;

    print "Angle = %6.3f Degrees, True Distance from Marker = %6.3f\"" % (Angle, distCrow)
    print ""
    #print "Downrange Distance from Marker = %6.3f\"" % (distOut)
    pub.publish("hello world")



if __name__ == '__main__':
    rospy.init_node('MoonAR_interpreter', anonymous=True)
    pub = rospy.Publisher('AlexAR', String, queue_size=10)
    rospy.Subscriber("/ar_pose_marker", AlvarMarkers, arCallback)

    rospy.spin()

