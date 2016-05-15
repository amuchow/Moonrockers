#!/usr/bin/env python
 
import rospy                        #for interacting with ROS topics and parameters
import sys, getopt                  #for parameters and sys.exit()
from Tkinter import *               #for GUI elements
from std_msgs.msg import Float32MultiArray, MultiArrayDimension



class Gui(object):
    def __init__(self, master):
        self.master = master
        master.wm_title("AR Data Creator")

        self.pub = rospy.Publisher('AlexAR', Float32MultiArray, queue_size=10)

        w = Label(master, text="Z-Value!")
        w.pack()
        ZValue = StringVar()
        self.ZValueText = Entry(master, textvariable=ZValue)
        self.ZValueText.pack()

        x = Label(master, text="X-Value")
        x.pack()
        XValue = StringVar()
        self.XValueText = Entry(master, textvariable=XValue)
        self.XValueText.pack()

        ZValue.set("1")
        XValue.set("1")

        B = Button(master, text ="Send", command = self.helloCallBack)
        B.pack()
        #self.update()

    def helloCallBack(self):

        self.master.update()

        myZ = self.ZValueText.get()
        myX = self.XValueText.get()



        packet = Float32MultiArray()
        packet.data.append(int(myX))
        packet.data.append(int(myZ))
        Packet = Float32MultiArray()

        self.pub.publish(packet)



    def callback():
        print e.get()
if __name__ == '__main__':
    rospy.init_node('AR_Creation')
    root = Tk()

    root.geometry("300x150")
    app = Gui(root)
    root.mainloop()

