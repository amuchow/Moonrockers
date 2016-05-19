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
import Queue


#Global Standards for State Machine
RobotSize                = 46
StandardThreshold        = 6
DepositingThreshold      = 3
StartMiningDistance      = 190
StopMiningDistance       = 290
CloseDepositDistance     = 30
DepositDistance          = 20
AllowableChangeThreshold = .15
OutlierDataThreshold     = 5
CanConveyor = 1


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
AngleAdjustForwardState = 19
RotateFaceRightState = 20
RotateFaceLeftState = 21
AngleAdjustReverseState = 22
RotateFaceRightReverseState = 23
RotateFaceLeftReverseState = 24


#TCP Globals
#TCP_IP = '127.0.0.1'
TCP_IP = '192.168.1.136'
TCP_PORT = 5000
BUFFER_SIZE = 12

#State Globals

RaiseAndSpin    = "300100100002"
nop             = "300000000002" 
moveDumpIn      = "310000000002" 
convRev         = "301000000002"
convOut         = "300100000002" 
moveDumpOut     = "300010000002"
convIn          = "300001000002" 
convFor         = "300000100002"
lWhFor          = "300000010002" 
lWhRev          = "300000001002"
rWhFor          = "300000000102" 
rWhRev          = "300000000012"
miningModeOn    = "300000000002"

#Robot Spin States
LeftForwardRightBackward = "300000001102"
RightForwardLeftBackward = "300000010012"

StateOneZero    = "300000010102"
StateOneOne     = "300000000102"
StateOneTwo     = "300000010002"
StateTwoZero    = "300000110102"
StateTwoOne     = "301100000102"
StateTwoTwo     = "301100010002"

StateThreeZero  = "300100001012"
StateThreeOne   = "300100001002"
StateThreeTwo   = "300100000012"
StateFourZero   = "300000001012"
StateFourOne    = "300000001002"
StateFourTwo    = "300000000012"
StateFourThree  = "300010000002"

class Gui(object):
    def __init__(self, master):
        #Class Variables
        self.STATE = 0
        self.Z = -1
        self.X = -1
        self.Angle = -1
        self.time = time.time()
        self.changeCounter = 0        
        self.ConveyorOne    = 0
        self.ConveyorTwo    = 0
        self.ConveyorThree  = 0
        self.AngleFlag = -1
        #Global Canvas for drawing
        global canvas

        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((TCP_IP, TCP_PORT))
        
        rospy.Subscriber("/AlexAR", Float32MultiArray, self.callback)
        self.master = master
        master.wm_title("AR State Machine")
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
        self.angleText = canvas.create_text(100, 275, text = "Angle: ")
        self.splitLine = canvas.create_line(175, 175, 175, 300)
        self.stateGoalSpotText = canvas.create_text(300, 175,  text = "Goal:")
        self.stateGoalText = canvas.create_text(350, 200,  text = "")
        
        self.update()

    def run(self):
        self.lbl.pack()
        self.lbl.after(250, self.update)
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

        #STATE NUMBER 19
        if self.STATE == AngleAdjustForwardState:
            canvas.itemconfig(self.StateNineRectangle, outline = "#000000")
            canvas.itemconfig(self.StateZeroRectangle, outline = "#000000")
            canvas.itemconfig(self.StateOneRectangle, outline = "#FF0000")
            canvas.itemconfig(self.stateGoalText, text = "Goto Rotate States if second waited.\nElse if ><+-2.5in, adjust" )
            self.AngleAdjustForward()

        #STATE NUMBER 20
        if self.STATE == RotateFaceRightState:
            canvas.itemconfig(self.StateNineRectangle, outline = "#000000")
            canvas.itemconfig(self.StateZeroRectangle, outline = "#000000")
            canvas.itemconfig(self.StateOneRectangle, outline = "#FF0000")
            canvas.itemconfig(self.stateGoalText, text = "Wait .25s" )
            self.RotateFaceRight()

        #STATE NUMBER 21
        if self.STATE == RotateFaceLeftState:
            canvas.itemconfig(self.StateNineRectangle, outline = "#000000")
            canvas.itemconfig(self.StateZeroRectangle, outline = "#000000")
            canvas.itemconfig(self.StateOneRectangle, outline = "#FF0000")
            canvas.itemconfig(self.stateGoalText, text = "Wait .25s" )
            self.RotateFaceLeft()
        #STATE NUMBER 22
        if self.STATE == AngleAdjustReverseState:
            canvas.itemconfig(self.StateNineRectangle, outline = "#000000")
            canvas.itemconfig(self.StateZeroRectangle, outline = "#000000")
            canvas.itemconfig(self.StateOneRectangle, outline = "#FF0000")
            canvas.itemconfig(self.stateGoalText, text = "Reverse Angle Adjust Crap" )
            self.AngleAdjustReverse()

        #STATE NUMBER 23
        if self.STATE == RotateFaceRightReverseState:
            canvas.itemconfig(self.StateNineRectangle, outline = "#000000")
            canvas.itemconfig(self.StateZeroRectangle, outline = "#000000")
            canvas.itemconfig(self.StateOneRectangle, outline = "#FF0000")
            canvas.itemconfig(self.stateGoalText, text = "Wait .25s" )
            self.RotateFaceRightReverse()

        #STATE NUMBER 24
        if self.STATE == RotateFaceLeftReverseState:
            canvas.itemconfig(self.StateNineRectangle, outline = "#000000")
            canvas.itemconfig(self.StateZeroRectangle, outline = "#000000")
            canvas.itemconfig(self.StateOneRectangle, outline = "#FF0000")
            canvas.itemconfig(self.stateGoalText, text = "Wait .25s" )
            self.RotateFaceLeftReverse()
            
        #Set AR Variables
        canvas.itemconfigure(self.zText,     text= "    Current Z :    "  + str(self.Z) + " in")
        canvas.itemconfigure(self.xText,     text= "    Current X :    "  + str(self.X) + " in")
        canvas.itemconfigure(self.stateText, text= "Current State:     "   + str(self.STATE))
        canvas.itemconfigure(self.angleText, text = "Angle: " + str(self.Angle))

        self.master.update()
        self.master.after(250, self.update)

    def Start(self):
        rospy.loginfo("Start State")
        #Need Code for Centering Here But Right now, Set State to traversing
        self.STATE = TraversingOutState
        self.time = time.time()
        rospy.loginfo("State: %d", self.STATE)
        return

    #State 19
    def AngleAdjustForward(self):
        rospy.loginfo("Angle Adjust Forward State")  

        time_spent = time.time() - self.time

        #Drive Forward
        if time_spent < 3.0:
            self.s.send(StateOneZero)
        #Settle Robot Down
        if time_spent > 3.0 and time_spent < 4.0:
            self.s.send(nop)

        #Base Case for Rotations
        if time_spent > 4.0:
            #Base Case for returning to driving state
            if self.Angle < 2.0 and self.Angle > -2.0:
                self.STATE = TraversingOutState
                self.time = time.time()    
                rospy.loginfo("Robot is returning to Traversing Out. STATE change to TraversingState(2)")
                return
            if self.AngleFlag == 1:
                #Rotate LeftFace State
                self.STATE = RotateFaceLeftState
                self.time = time.time()    
                rospy.loginfo("Robot is adjusting for left face. STATE change to RotateFaceLeft(21)")
                return
            else:
                #Rotate RightFace State
                self.STATE = RotateFaceRightState
                self.time = time.time()    
                rospy.loginfo("Robot is adjusting for right face. STATE change to RotateFaceRight(20)")
                return

    #State 22
    def AngleAdjustReverse(self):
        rospy.loginfo("Angle Adjust Reverse State")  

        time_spent = time.time() - self.time
        #Drive Forward
        if time_spent < 3.0:
            self.s.send(StateThreeZero)
        #Settle Robot Down
        if time_spent > 3.0 and time_spent < 4.0:
            self.s.send(nop)
        
        #Base Case for Rotations
        if time_spent > 4.0:

            #Base Case for returning to driving state
            if self.Angle < 1.25 and self.Angle > -1.25:
                self.STATE = TraversingBackState
                self.time = time.time()    
                rospy.loginfo("Robot is returning to Traversing Back. STATE change to TraversingBackState(7)")
                return
            if self.AngleFlag == 1:
                #Rotate LeftFace State
                self.STATE = RotateFaceLeftReverseState
                self.time = time.time()    
                rospy.loginfo("Robot is adjusting for left face. STATE change to RotateFaceLeft(21)")
                return
            else:
                #Rotate RightFace State
                self.STATE = RotateFaceRightReverseState
                self.time = time.time()    
                rospy.loginfo("Robot is adjusting for right face. STATE change to RotateFaceRight(20)")
                return

    #State 20  
    def RotateFaceRight(self):
        rospy.loginfo("Rotate Face Right State")

        self.s.send(LeftForwardRightBackward)

        time_spent = time.time() - self.time
        if time_spent > .25:
            self.STATE = AngleAdjustForwardState
            self.time = time.time()    
            rospy.loginfo("Robot has adjusted right face. STATE change to AngleAdjustForward(19)")
            return

    #State 21
    def RotateFaceLeft(self):
        rospy.loginfo("Rotate Face Left State")  

        self.s.send(RightForwardLeftBackward)

        time_spent = time.time() - self.time
        if time_spent > .25:
            self.STATE = AngleAdjustForwardState
            self.time = time.time()    
            rospy.loginfo("Robot has adjusted left face. STATE change to AngleAdjustForward(19)")
            return
    #State 23
    def RotateFaceRightReverse(self):
        rospy.loginfo("Rotate Face Right Reverse State")

        self.s.send(LeftForwardRightBackward)

        time_spent = time.time() - self.time
        if time_spent > .25:
            self.STATE = AngleAdjustReverseState
            self.time = time.time()    
            rospy.loginfo("Robot has adjusted right face Reverse. STATE change to AngleAdjustReverse(19)")
            return

    #State 24
    def RotateFaceLeftReverse(self):
        rospy.loginfo("Rotate Face Left ReverseState")  

        self.s.send(RightForwardLeftBackward)

        time_spent = time.time() - self.time
        if time_spent > .25:
            self.STATE = AngleAdjustReverseState
            self.time = time.time()    
            rospy.loginfo("Robot has adjusted left face Reverse. STATE change to AngleAdjustReverse(19)")
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

        #Angle too big.
        if self.Angle > 2.0 or self.Angle < -2.0:
            self.STATE = AngleAdjustForwardState
            self.time = time.time()    
            rospy.loginfo("Robot is adjusting the Angle. STATE change to AngleAdjustForward(19)")
            return


            
                    


        #Too Far left
        if self.X < (-1) * StandardThreshold:
            rospy.loginfo("Robot is too far to the left of the centerline. Correcting...") 
            #while self.Angle > 5 and self.Angle < -5:
            #$    if self.Angle > 0:
                   
            #Toggle Right High on and off to correct
            #self.s.send(StateOneTwo)
            #self.s.send(StateOneZero)

        
        #Too far right
        if self.X > StandardThreshold:
            rospy.loginfo("Robot is too far to the right of the centerline. Correcting...")
            #Toggle Left High on and off to correct
            #self.s.send(StateOneOne)
            #self.s.send(StateOneZero)

        
    def LowerConveyor(self):
        rospy.loginfo("Lower State")

        if CanConveyor == 1:
            self.s.send(convIn)
        else:
            rospy.loginfo("Conveyor function turned off for testing.")

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
        if time_spent > 1.25:
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

        #Angle too big.
        if self.Angle > 1.25 or self.Angle < -1.25:
            self.STATE = AngleAdjustReverseState
            self.time = time.time()    
            rospy.loginfo("Robot is adjusting the Angle. STATE change to AngleAdjustReverse(22)")
            return

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
        self.Angle = data.data[2]
        self.AngleFlag = data.data[3]

if __name__ == '__main__':
    rospy.init_node('MoonBrain')
    root = Tk()

    root.geometry("900x700")
    app = Gui(root).run()
    root.mainloop()










