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
from std_msgs.msg import Float32MultiArray

def arCallback(data):
    rospy.logdebug(rospy.get_name() + ": Received Quaternion")
    rospy.logdebug(data)

    if not data.markers:
        return
    data = data.markers[0]

    posX = data.pose.pose.position.x
    posY = data.pose.pose.position.y
    posZ = data.pose.pose.position.z
    
    angW = data.pose.pose.orientation.w
    angX = data.pose.pose.orientation.x
    angY = data.pose.pose.orientation.y
    angZ = data.pose.pose.orientation.z

    if data.id == 0:
        print "15cm Tag"
        print "Position: (x, y, z)->(% 6.4f,% 6.4f,% 6.4f)" % (posX, posY, posZ)
        print "Rotation: (q, x, y, z) -> (%6.4f, % 6.4f,% 6.4f,% 6.4f)" %  (angW, angX, angY, angZ)

        if angZ < 0:
            Angle = (math.acos(angX) + -1 * math.asin(angZ)) * 180 / math.pi
        else:
            Angle = (math.acos(angX) + math.asin(angZ)) * 180 / math.pi

        distCrow = posZ * 3.28 * 12
        distX = posX * 3.28 * 12 * (5 / 16.2)


        CrowError = (0.054 * distCrow + .0411)
        distCrow = distCrow - CrowError;

        if distX < 0: 
            distX = distX + (-0.0662 * distX + .368)
        else:
            distX = distX - (-0.0662 * distX + .368)
        if Angle < 0:
            Angle = (Angle + (-.0167 * Angle + 2.21))
        else:
            Angle = (Angle - (-.0167 * Angle + 2.21))



        print "Angle = %6.3f Degrees, True Distance from Marker = %6.3f\"" % (Angle, distCrow)
        print "Distance Over = %6.3f Inches" % (distX)

        #First Triangle
        # z1 = Dist Over * tan 90 - Theta => radians
        trueAngle = (90 - Angle) * math.pi / 180
        z1 = abs(distX * math.tan(trueAngle))
        #TrueZ1 = sqrt(dist over^2 + (z - z1)^2)
        TrueZ1 = math.sqrt( (distX * distX) + (z1 * z1) )
        print "(TrueZ1, Z1) = (%6.3f, %6.3f) Inches" % (TrueZ1, z1)

        #Second Triangle
        #Find second part of Z
        z2 = (distCrow - z1)
        #TrueZ2 = z2 * sin( 90 - theta)
        TrueZ2 = z2 * math.sin(trueAngle)
        TrueX = z2 * math.cos(trueAngle)
        print "(TrueZ2, TrueX) = (%6.3f, %6.3f) Inches" % (TrueZ2, TrueX)

        #Finish Math
        TrueZ = TrueZ1 + TrueZ2
        TrueDistance = math.sqrt(TrueZ*TrueZ + TrueX*TrueX)
        print "(Away, Offset) = (%6.3f, %6.3f) in = %6.3f" % (TrueZ, TrueX,TrueDistance)

    if data.id == 4:
        print "5cm Tag"
        print "Position: (x, y, z)->(% 6.4f,% 6.4f,% 6.4f)" % (posX, posY, posZ)
        print "Rotation: (q, x, y, z) -> (%6.4f, % 6.4f,% 6.4f,% 6.4f)" %  (angW, angX, angY, angZ)


        if angZ < 0:
            Angle = (math.acos(angX) + -1 * math.asin(angZ)) * 180 / math.pi
        else:
            Angle = (math.acos(angX) + math.asin(angZ)) * 180 / math.pi

        distCrow = posZ * 3.28 * 12 * ( 5 / 16.2 )
        distX = posX * 3.28 * 12 * (5 / 16.2)

        CrowError = (0.094 * distCrow + .0411)
        distCrow = distCrow - CrowError;

        if distX < 0: 
            distX = distX + (-0.0662 * distX + .368)
        else:
            distX = distX - (-0.0662 * distX + .368)
        if Angle < 0:
            Angle = (Angle + (-.0167 * Angle + 2.21))
        else:
            Angle = (Angle - (-.0167 * Angle + 2.21))

        print "Angle = %6.3f Degrees, True Distance from Marker = %6.3f\"" % (Angle, distCrow)
        print "Distance Over = %6.3f Inches" % (distX)


        #First Triangle
        # z1 = Dist Over * tan 90 - Theta => radians
        trueAngle = (90 - Angle) * math.pi / 180
        z1 = abs(distX * math.tan(trueAngle))
        #TrueZ1 = sqrt(dist over^2 + (z - z1)^2)
        TrueZ1 = math.sqrt( (distX * distX) + (z1 * z1) )
        print "(TrueZ1, Z1) = (%6.3f, %6.3f) Inches" % (TrueZ1, z1)

        #Second Triangle
        #Find second part of Z
        z2 = (distCrow - z1)
        #TrueZ2 = z2 * sin( 90 - theta)
        TrueZ2 = z2 * math.sin(trueAngle)
        TrueX = z2 * math.cos(trueAngle)
        print "(TrueZ2, TrueX) = (%6.3f, %6.3f) Inches" % (TrueZ2, TrueX)

        #Finish Math
        TrueZ = TrueZ1 + TrueZ2
        TrueDistance = math.sqrt(TrueZ*TrueZ + TrueX*TrueX)
        print "(Away, Offset) = (%6.3f, %6.3f) in = %6.3f" % (TrueZ, TrueX,TrueDistance)

    Packet = Float32MultiArray()
    #Packet.data = []
    #Packet.data[0] = Angle
    #Packet.data[1] = distCrow


    #print "Downrange Distance from Marker = %6.3f\"" % (distOut)
    pub.publish(Packet)



if __name__ == '__main__':
    rospy.init_node('MoonAR_interpreter', anonymous=True)
    pub = rospy.Publisher('AlexAR', Float32MultiArray, queue_size=10)
    rospy.Subscriber("/ar_pose_marker", AlvarMarkers, arCallback)

    rospy.spin()

