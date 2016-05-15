#!/usr/bin/env python

#-*- coding: utf-8 -*-

import sys
import roslib
import rospy
import os.path
import math
import socket
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
import time
from Tkinter import *
from std_msgs.msg import Int16
import time
import signal
import threading

#Global Standards for State Machine
RobotSize                = 46
StandardThreshold        = 6
DepositingThreshold      = 3
StartMiningDistance      = 180
StopMiningDistance       = 290
CloseDepositDistance     = 30
DepositDistance          = 20
AllowableChangeThreshold = .15
OutlierDataThreshold     = 5


#Global States for State Machine
StartState          = 0
TraversingOutState  = 1
LowerConveyorState  = 2
MineSpinState       = 3
MiningState         = 4
RearConveyorState   = 5
FinishMineSpinState = 6
TraversingBackState = 7
TraversingDepState  = 8
TimeToDepositState  = 9

#TCP Globals
TCP_IP = '127.0.0.1'
TCP_PORT = 5000
BUFFER_SIZE = 12

#State Globals

RaiseAndSpin    = "300100100000"

moveDumpIn      = "310000000000" 
convRev         = "301000000000"
convOut         = "300100000000" 
moveDumpOut     = "300010000000"
convIn          = "300001000000" 
convFor         = "300000100000"
lWhFor          = "300000010000" 
lWhRev          = "300000001000"
rWhFor          = "300000000100" 
rWhRev          = "300000000010"
miningModeOn    = "300000000001" 

StateOneZero    = "300000010100"
StateOneOne     = "300000000100"
StateOneTwo     = "300000010000"

StateTwoZero    = "300000110100"
StateTwoOne     = "301100000100"
StateTwoTwo     = "301100010000"

StateThreeZero  = "300100001010"
StateThreeOne   = "300100001000"
StateThreeTwo   = "300100000010"

StateFourZero   = "300000001010"
StateFourOne    = "300000001000"
StateFourTwo    = "300000000010"
StateFourThree  = "300010000000"

class Gui(object):
    def __init__(self, master):
        #Class Variables
        self.STATE = 0
        self.Z = -1
        self.X = -1
        self.time = time.time()
        self.changeCounter = 0        
        self.ConveyorOne    = 0
        self.ConveyorTwo    = 0
        self.ConveyorThree  = 0

        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((TCP_IP, TCP_PORT))
        
        rospy.Subscriber("/AlexAR", Float32MultiArray, self.callback)
        self.master = master
        master.wm_title("AR State Machine")
        global canvas
        canvas = Canvas(self.master, width=700, height=700)
        canvas.pack()
        self.lbl = Label(root, text="")
        
        #StartX, StartY, EndX, EndY                  
        self.StateZeroRectangle = canvas.create_rectangle(5, 85, 35, 115)
        self.StateZeroArrow = canvas.create_line(35, 100, 50, 100, arrow=LAST)
        self.StateZeroText = canvas.create_text(20, 100, text = "Start")
        
        self.StateOneRectangle = canvas.create_rectangle(50, 50, 150, 150)
        self.StateOneArrow = canvas.create_line(150, 100, 200, 100, arrow=LAST)
        self.StateOneText = canvas.create_text(100, 100, text = "       State 1:\nTraversing Out")

        self.StateTwoRectangle = canvas.create_rectangle(200, 50, 300, 150)
        self.StateTwoArrow = canvas.create_line(300, 100, 350, 100, arrow=LAST)
        self.StateTwoText = canvas.create_text(250, 100, text = "        State 2:\nLower Conveyor")
                   
        self.StateThreeRectangle = canvas.create_rectangle(350, 50, 450, 150)
        self.StateThreeArrow = canvas.create_line(450, 100, 500, 100, arrow=LAST)
        self.StateThreeText = canvas.create_text(400, 100, text = "   State 3:\nMine Spin")
                   
        self.StateFourRectangle = canvas.create_rectangle(500, 50, 600, 150)
        self.StateFourLineOne = canvas.create_line(600, 100, 650, 100)
        self.StateFourLineTwo = canvas.create_line(650, 100, 650, 400)
        self.StateFourLineThree = canvas.create_line(650, 400, 5, 400)
        self.StateFourLineFour = canvas.create_line(5, 400, 5, 500)
        self.StateFourLineFive = canvas.create_line(5, 500, 50, 500, arrow=LAST)
        self.StateFourText = canvas.create_text(550, 100, text = " State 4:\n Mining")

        self.StateFiveRectangle = canvas.create_rectangle(500, 200, 600, 300)
        self.StateFiveLineOne = canvas.create_line(525, 150, 525, 200, arrow=LAST)
        self.StateFiveLineTwo = canvas.create_line(575, 200, 575, 150, arrow=LAST)
        self.StateFiveText = canvas.create_text(550, 250, text = "        State 5:\n Rear Conveyor")
        
        self.StateSixRectangle = canvas.create_rectangle(50, 450, 150, 550)
        self.StateSixArrow = canvas.create_line(150, 500, 200, 500, arrow=LAST)
        self.StateSixText = canvas.create_text(100, 500, text = "   State 6:\n Finish Spin\n  and Raise")
        
        self.StateSevenRectangle = canvas.create_rectangle(200, 450, 300, 550)
        self.StateSevenArrow = canvas.create_line(300, 500, 350, 500, arrow=LAST)
        self.StateSevenText = canvas.create_text(250, 500, text = "   State 7:\n Traversing Back")

        self.StateEightRectangle = canvas.create_rectangle(350, 450, 450, 550)
        self.StateEightArrow = canvas.create_line(450, 500, 500, 500, arrow=LAST)
        self.StateEightText = canvas.create_text(400, 500, text = "   State 8:\n Traversing\n Depositing")

        self.StateNineRectangle = canvas.create_rectangle(500, 450, 600, 550)
        self.StateNineArrow = canvas.create_line(600, 500, 650, 500, arrow=LAST)
        self.StateNineText = canvas.create_text(550, 500, text = "   State 9:\n Depositing")
        

        self.zText = canvas.create_text(100, 200,  text = "    Current Z :")
        self.xText = canvas.create_text(100, 225,   text = "    Current X :")
        self.stateText = canvas.create_text(100, 250, text = "Current State:")

	self.splitLine = canvas.create_line(175, 175, 175, 300)
        self.stateGoalSpotText = canvas.create_text(300, 175,  text = "Goal:")
        self.stateGoalText = canvas.create_text(350, 200,  text = "")
        
        self.update()

    def run(self):
        self.lbl.pack()
        self.lbl.after(1000, self.update)
        self.master.mainloop()

    def update(self):

        #STATE NUMBER 0
        if self.STATE == StartState:
            canvas.itemconfig(self.StateNineRectangle, outline = "#000000")
            canvas.itemconfig(self.StateZeroRectangle, outline = "#FF0000")
            canvas.itemconfig(self.stateGoalText, text = "Skip to Traversing")
            self.Start()
        else:
            canvas.itemconfig(self.StateZeroRectangle, outline = "#000000")

        #STATE NUMBER 1
        if self.STATE == TraversingOutState:
            canvas.itemconfig(self.StateNineRectangle, outline = "#000000")
            canvas.itemconfig(self.StateZeroRectangle, outline = "#000000")
            canvas.itemconfig(self.StateOneRectangle, outline = "#FF0000")
            canvas.itemconfig(self.stateGoalText, text = "Go forward until you get to " + str(StartMiningDistance - RobotSize) + " in " )
            self.TraversingOut()
        else:
            canvas.itemconfig(self.StateOneRectangle, outline = "#000000")
            
        #STATE NUMBER 2
        if self.STATE == LowerConveyorState :
            canvas.itemconfig(self.StateOneRectangle, outline = "#000000")
            canvas.itemconfig(self.StateTwoRectangle, outline = "#FF0000")
            canvas.itemconfig(self.stateGoalText, text = "Lowe Conveyor: Count for 3 seconds")
            self.LowerConveyor()
        else:
            canvas.itemconfig(self.StateTwoRectangle, outline = "#000000")
        
        #STATE NUMBER 3
        if self.STATE == MineSpinState:
            canvas.itemconfig(self.StateTwoRectangle, outline = "#000000")
            canvas.itemconfig(self.StateThreeRectangle, outline = "#FF0000")
            canvas.itemconfig(self.stateGoalText, text = "MineSpin: Count for 3 seconds")
            self.MineSpin()
        else:
            canvas.itemconfig(self.StateThreeRectangle, outline = "#000000")
            
        #STATE NUMBER 4
        if self.STATE == MiningState:
            canvas.itemconfig(self.StateThreeRectangle, outline = "#000000")
            canvas.itemconfig(self.StateFourRectangle, outline = "#FF0000")
            canvas.itemconfig(self.stateGoalText, text = "Go forward until you get to " + str(StopMiningDistance - RobotSize) + " in to go to Finish Spin\nand Raise State\n" + "For Rear Conveyor, go until " + str(StopMiningDistance - RobotSize - 30) + " in , " + str(StopMiningDistance - RobotSize - 60) + " in , or " + str(StopMiningDistance - RobotSize - 90) + " in.")
            self.Mining()
        else:
            canvas.itemconfig(self.StateFourRectangle, outline = "#000000")
            
        #STATE NUMBER 5
        if self.STATE == RearConveyorState:
            canvas.itemconfig(self.StateFourRectangle, outline = "#000000")
            canvas.itemconfig(self.StateFiveRectangle, outline = "#FF0000")
            canvas.itemconfig(self.stateGoalText, text = "Rear Conveyor: Count for 2 seconds")
            self.RearConveyor()
        else:
            canvas.itemconfig(self.StateFiveRectangle, outline = "#000000")
            
        #STATE NUMBER 6
        if self.STATE == FinishMineSpinState:
            canvas.itemconfig(self.StateFiveRectangle, outline = "#000000")
            canvas.itemconfig(self.StateSixRectangle, outline = "#FF0000")
            canvas.itemconfig(self.stateGoalText, text = "FinishMineSpin: Count for 3 seconds")
            self.FinishMineSpin()
        else:
            canvas.itemconfig(self.StateSixRectangle, outline = "#000000")
            
        #STATE NUMBER 7
        if self.STATE == TraversingBackState:
            canvas.itemconfig(self.StateSixRectangle, outline = "#000000")
            canvas.itemconfig(self.StateSevenRectangle, outline = "#FF0000")
            canvas.itemconfig(self.stateGoalText, text = "Go Backward until you get to " + str(CloseDepositDistance) + " in " )
		
            self.TraversingBack()
        else:
            canvas.itemconfig(self.StateSevenRectangle, outline = "#000000")
            
        #STATE NUMBER 8
        if self.STATE == TraversingDepState:
            canvas.itemconfig(self.StateSevenRectangle, outline = "#000000")
            canvas.itemconfig(self.StateEightRectangle, outline = "#FF0000")
            canvas.itemconfig(self.stateGoalText, text = "Go Backward until you get to " + str(DepositDistance) + " in " )
            self.TraversingDep()
        else:
            canvas.itemconfig(self.StateEightRectangle, outline = "#000000")
            
        #STATE NUMBER 9
        if self.STATE == TimeToDepositState:
            canvas.itemconfig(self.StateEightRectangle, outline = "#000000")
            canvas.itemconfig(self.StateNineRectangle, outline = "#FF0000")
            canvas.itemconfig(self.stateGoalText, text = "FinishMineSpin: Count for 7 seconds")
            self.TimeToDeposit()
        else:
            canvas.itemconfig(self.StateNineRectangle, outline = "#000000")
            
        #Set AR Variables
        canvas.itemconfigure(self.zText,     text= "    Current Z :    "  + str(self.Z) + " in")
        canvas.itemconfigure(self.xText,     text= "    Current X :    "  + str(self.X) + " in")
        canvas.itemconfigure(self.stateText, text= "Current State:     "   + str(self.STATE))

        self.master.update()
        self.master.after(500, self.update)

    def Start(self):
        rospy.loginfo("Start State")
        #Need Code for Centering Here But Right now, Set State to traversing
        self.STATE = TraversingOutState
        self.time = time.time()
        rospy.loginfo("State: %d", self.STATE)
        return

    def TraversingOut(self):
        rospy.loginfo("Traversing Out State")    

        #Check if Finished Driving Out
        if self.Z > (StartMiningDistance - RobotSize):
            self.STATE = LowerConveyorState
            self.time = time.time()
            rospy.loginfo("Robot has reached the mining area. STATE change to Lower(2)")
            return
  
        #Drive forward
        self.s.send(StateOneZero)

        #Too Far left
        if self.X < (-1) * StandardThreshold:
            rospy.loginfo("Robot is too far to the left of the centerline. Correcting...") 
            #Toggle Right High on and off to correct
            #self.s.send(StateOneTwo)
            #self.s.send(StateOneZero)
            return
        
        #Too far right
        if self.X > StandardThreshold: 
            rospy.loginfo("Robot is too far to the right of the centerline. Correcting...")
            #Toggle Left High on and off to correct
            #self.s.send(StateOneOne)
            #self.s.send(StateOneZero)
            return
        
    def LowerConveyor(self):
        rospy.loginfo("Lower State")
        
        self.s.send(convIn)
        time_spent = time.time() - self.time

        if time_spent > 3:
            self.STATE = MineSpinState
            self.time = time.time()
            rospy.loginfo("Robot has finished Lowering. STATE change to MineSpin(3)")
            return
        
    def MineSpin(self):
        rospy.loginfo("MineSpin State")
        self.s.send(convFor)
        time_spent = time.time() - self.time

        if time_spent > 3:
            self.STATE = MiningState
            self.time = time.time()
            self.ConveyorOne   = 1
            self.ConveyorTwo   = 1
            self.ConveyorThree = 1
            rospy.loginfo("Robot has finished Spinning. STATE change to MiningState(4)")
            return
        
    def Mining(self):
        rospy.loginfo("Mining State")

        adjustedMiningDistance = StopMiningDistance - RobotSize

        #First Rear Conveyor Shift
        if (self.Z > adjustedMiningDistance - 90)  and self.ConveyorOne == 1:
            self.STATE = RearConveyorState
            self.ConveyorOne   = 0
            self.time = time.time()
            return
        
        #Second Rear Conveyor Shift
        if (self.Z > adjustedMiningDistance - 60)  and self.ConveyorTwo == 1:
            self.STATE = RearConveyorState
            self.ConveyorTwo   = 0
            self.time = time.time()
            return
        
        #Third Rear Conveyor Shift
        if (self.Z > adjustedMiningDistance - 30)  and self.ConveyorThree == 1:
            self.STATE = RearConveyorState
            self.ConveyorThree   = 0
            self.time = time.time()
            return

        #Finished Mining
        if self.Z > adjustedMiningDistance:
            self.STATE = FinishMineSpinState 
            self.time = time.time()
            rospy.loginfo("Robot has finished mining. STATE change to FinishMineSpinState(6)")
            return

        #Drive Forward
        self.s.send(StateTwoZero)

        #Too Far Right
        if self.X < (-1) * StandardThreshold:
            rospy.loginfo("While Mining, Robot is too far to the left of the centerline. Correcting...")
            #Toggle Right High on and off to correct
            #self.s.send(StateTwoTwo)
            #self.s.send(StateTwoZero)
            return
        
        #Too Far Left
        if self.X > StandardThreshold: 
            rospy.loginfo("While Mining, Robot is too far to the right of the centerline. Correcting...")
            #Toggle Left High on and off to correct
            #self.s.send(StateTwoOne)
            #self.s.send(StateTwoZero)
            return

    def RearConveyor(self):
        rospy.loginfo("RearConveyor State")

        self.s.send(moveDumpOut)
        time_spent = time.time() - self.time
        if time_spent > 2:
            self.STATE = MiningState
            self.time = time.time()
            rospy.loginfo("Robot has finished RearConveyor spinning. STATE change to Mining(1)")
            return
        
    def FinishMineSpin(self):
        rospy.loginfo("FinishMineSpin State")
        self.s.send(RaiseAndSpin)
        time_spent = time.time() - self.time

        if time_spent > 3:
            self.STATE = TraversingBackState
            self.time = time.time()
            rospy.loginfo("Robot has finished Raising. STATE change to TraversingBack(3)")
            return
        
    def TraversingBack(self):
        rospy.loginfo("Traversing Back State")  

        #Finished Driving Out
        if self.Z < CloseDepositDistance :
            self.STATE = TraversingDepState
            self.time = time.time()
            #rospy.loginfo("Robot has almost reached area. STATE change to TraversingDepState(8)")
            return
        
        #Drive forward
        self.s.send(StateThreeZero)

        #Too Far left
        if self.X < (-1) * StandardThreshold:
            rospy.loginfo("Robot is too far to the left of the centerline. Correcting...")
            #Toggle Right High on and off to correct
            #self.s.send(StateThreeOne)
            #self.s.send(StateThreeZero)
            return
        
        #Too far right
        if self.X > StandardThreshold: 
            rospy.loginfo("Robot is too far to the right of the centerline. Correcting...")
            #Toggle Left High on and off to correct
            #self.s.send(StateThreeOne)
            #self.s.send(StateThreeZero)
            return
    def TraversingDep(self):
        rospy.loginfo("TraversingDepState State")
        
        #Finished Driving Out
        if self.Z < DepositDistance:
            self.STATE = TimeToDepositState
            self.time = time.time()
            rospy.loginfo("Robot has reached the depositing area. STATE change to TimeToDepositState(9)")
            return
        
        #Drive forward 
        self.s.send(StateFourZero)

        #Too Far left
        if self.X < (-1) * DepositingThreshold:
            rospy.loginfo("Robot is too far to the left of the centerline. Correcting...")
            #Toggle Right High on and off to correct
            #self.s.send(StateFourOne)
            #self.s.send(StateFourZero
            return
        
        #Too far right
        if self.X > DepositingThreshold: 
            rospy.loginfo("Robot is too far to the right of the centerline. Correcting...")
            
            #Toggle Left High on and off to correct
            #self.s.send(StateFourOne)
            #self.s.send(StateFourZero)
            return

    def TimeToDeposit(self):
        rospy.loginfo("TimeToDeposit State")
                    
        self.s.send(moveDumpOut)
        time_spent = time.time() - self.time

        if time_spent > 7:
            self.STATE = TraversingOutState
            self.time = time.time()
            rospy.loginfo("Robot has finished Dumping. STATE change to TraversingOut(1)")
            return





    def callback(self, data):
        
        rospy.loginfo("I heard %s",data.data)
        if self.changeCounter < OutlierDataThreshold:
            if self.X != -1 and self.Z != -1:

                #Check for Reasonable amount of change.
                xChange = abs(self.X - data.data[0])
                xChangePercent = abs( xChange / self.X )
                zChange = abs(self.Z - data.data[1])
                zChangePercent = abs( zChange / self.Z )
                
                if xChangePercent > AllowableChangeThreshold and  xChange > 10:       
                    rospy.loginfo("X Changed too rapidly, ignoring outlier.")
                    #Increment the rejected data counter
                    self.changeCounter += 1
                    return
                
                elif zChangePercent > AllowableChangeThreshold and zChange > 10:   
                    rospy.loginfo("Z Changed too rapidly, ignoring outlier.")
                    #Increment the rejected data counter
                    self.changeCounter += 1   
                    return
                
                else:
                    #Reset the rejected data counter
                    self.changeCounter = 0
            else:
                rospy.loginfo("First Data Set.")
                print("First Data Set.")
        else:
            rospy.loginfo("Outliers seen 10x in a row. Outlier accepted.")
            self.changeCounter = 0

        self.X = data.data[0]
        self.Z = data.data[1]

if __name__ == '__main__':
    rospy.init_node('MoonBrain')
    root = Tk()

    root.geometry("900x700")
    app = Gui(root).run()
    root.mainloop()










