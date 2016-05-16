#!/usr/bin/env python

#-*- coding: utf-8 -*-

import sys
import roslib
import rospy
import os.path
import math
from Tkinter import *
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
from std_msgs.msg import Int16
import time
import signal
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
import threading

import time

global myX
global myZ
SmallTagTrustDistance = 70
LargeTagTrustDistance = 70

class Gui(object):
    def __init__(self, master):
        rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.arCallback)
        self.pub = rospy.Publisher('AlexAR', Float32MultiArray, queue_size=10)
        self.master = master
        master.wm_title("AR Data Map")
        global w
        w = Canvas(self.master, width=700, height=700)
        w.pack()
        self.lbl = Label(root, text="")
        #StartX, StartY, EndX, EndY                   
        w.create_rectangle(100,  25, 600, 485)
        w.create_rectangle(100,  25, 600, 255)
        w.create_rectangle(100,  25, 600, 600)
        w.create_rectangle(325, 600, 375, 620)
        self.angleText = w.create_text(200, 650, text = "Angle:")
        self.awayText = w.create_text(450, 650, text = "Away:")
        self.offsetText = w.create_text(550, 650, text = "Offset:")
        self.myX = 0
        self.myZ = 0
        self.negativeAngle = 0
        self.angle = 0
    

        xScale = 350 - self.myX
        zScale = 600 - self.myZ

        # 175, 200 - Length of Z scaled, 175 - Length of X scaled, 200 - Length of Z scaled
        self.xLine = w.create_line(350, zScale, xScale, zScale, fill="black", dash=(4,4))

        # 175, Z scaled, 175, 200
        self.zLine = w.create_line(350, zScale, 350, 600, fill="black", dash=(4,4))

        #175, 200, 175 - Length of X scaled, 200 - Length of Z scaled 
        self.cLine = w.create_line(350, 600, xScale, zScale, fill="black", dash=(4,4))
        self.updateGUI()
        self.readSensor()

    def run(self):
        self.lbl.pack()
        self.lbl.after(1000, self.updateGUI)
        self.master.mainloop()

    def updateGUI(self):

        print "now here"
        print "(%d, %d)" % (self.myX, self.myZ)

        if self.angle < 3:
            self.negativeAngle = 0

        zScale = 600 - 1.94 * self.myZ
        if self.negativeAngle == 1:
            w.itemconfigure(self.angleText, text="Angle: -"   + str(int(self.angle)) + " degrees")
            xScale = 350 - 3.34 * self.myX
        else:
            w.itemconfigure(self.angleText, text="Angle: "   + str(int(self.angle)) + " degrees")
            xScale = 350 + 5 * self.myX
        w.coords(self.cLine, 350, 600, xScale, zScale)
        w.coords(self.xLine, 350, zScale, xScale, zScale)
        w.coords(self.zLine, 350, 600, 350, zScale)

        w.itemconfigure(self.awayText, text="Away: "     + str(int(self.myZ)) + " in")
        w.itemconfigure(self.offsetText, text="Offset: " + str(int(self.myX)) + " in")    
        self.master.update()
        self.master.after(1000, self.updateGUI)

    def readSensor(self):

        self.master.update()
        self.master.after(527, self.readSensor)

    def arCallback(self, data):
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
            print "30cm Tag"
            print "Position: (x, y, z)->(% 6.4f,% 6.4f,% 6.4f)" % (posX, posY, posZ)
            print "Rotation: (q, x, y, z) -> (%6.4f, % 6.4f,% 6.4f,% 6.4f)" %  (angW, angX, angY, angZ)

            if angZ < 0:
                Angle = (math.acos(angX) + -1 * math.asin(angZ)) * 180 / math.pi
                self.negativeAngle = 1
            else:
                Angle = (math.acos(angX) + math.asin(angZ)) * 180 / math.pi
                self.negativeAngle = 0

            if angY < 0:
                AngleY = (math.acos(angX) + -1 * math.asin(angY)) * 180 / math.pi
            else:
                AngleY = (math.acos(angX) + math.asin(angY)) * 180 / math.pi

            print "AngleY = %6.3f Degrees" % (AngleY)
            Angle = Angle - AngleY

            distCrow = posZ * 39.37# * 1.167 * 1.1
            distX    = posX * 39.37# * 1.167 * 1.1


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

            #Screw It All
            negFlag = 0
            myNewFavoriteNumber = (distCrow * distCrow) - (TrueZ * TrueZ)
            if myNewFavoriteNumber < 0:
                negFlag = 1
            
            TrueX = math.sqrt(abs(myNewFavoriteNumber))
            if negFlag == 1:
                TrueX = (-1) * TrueX
            TrueDistance = math.sqrt(TrueZ*TrueZ + TrueX*TrueX)
            print "(Away, Offset) = (%6.3f, %6.3f) in = %6.3f" % (TrueZ, TrueX,TrueDistance)
            

            self.angle = Angle
            if self.angle < 1.0:
                self.myZ = TrueZ
                self.myX = distX
            else:
                self.myZ = TrueZ
                self.myX = TrueX    

            print "(%d, %d)" % (self.myX, self.myZ)

        myTrueAngle = math.tan(self.myX)
        packet = Float32MultiArray()
        packet.data.append(self.myX)
        packet.data.append(self.myZ)
        packet.data.append(self.angle)
        packet.data.append(self.negativeAngle)


        #print "Downrange Distance from Marker = %6.3f\"" % (distOut)
        self.pub.publish(packet)
class Demo1:
    def __init__(self, master):
        self.master = master
        # Subscribe to the recognizer output and set the callback function
        self.map = 0
        rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.talkback)
        self.w = tk.Label(master, text=self.map)
        self.frame = tk.Frame(self.master)
        self.button1 = tk.Button(self.frame, text = 'Quit', width = 25, command = self.close_windows)
        self.button1.pack()
        self.w.pack()
        self.frame.pack()
    def close_windows(self):
        self.master.destroy()


    def check_battery(self, msg):
        rospy.logwarn("Battery low - level: " + str(int(msg.data)))
        self.map = str(int(msg.data))
    def talkback(self, msg):
        # republis input to task coord or chat engine
        #self.map= msg.pose.pose.position.x
        rospy.logwarn(msg.data)

if __name__ == '__main__':

    rospy.init_node('MoonAR_interpreter', anonymous=True)

    root = Tk()

    root.geometry("900x700")
    app = Gui(root).run()
    root.mainloop()
    #Gui().run()
    #SensorThread().start()
