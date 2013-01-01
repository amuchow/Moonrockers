#!/usr/bin/env python
import rospy
import socket

from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
import math
import time
#           TCP Packet Guide        
#  -----------------------------------
#  1.  Controller Control - 000000000000
#  2.  Toggle Controller  - 100000000000
#  3.  Toggle Moonbrain   - 200000000000
#  4.  Moonbrain Control  - 300000000000
#  1.  Move Dump In       - 010000000000
#  2.  Conveyor Reverse   - 001000000000
#  3.  Conveyor Out       - 000100000000
#  4.  Move Dump Out      - 000010000000
#  5.  Conveyor In        - 000001000000
#  6.  Conveyor Forward   - 000000100000
#  7.  L-Wheel Forward    - 000000010000
#  8.  L-Wheel Reverse    - 000000001000
#  9.  R-Wheel Forward    - 000000000100
#  10. R-Wheel Reverse    - 000000000010
#  11. Mining Mode On     - 000000000001
#  12. Mining Mode Off    - 000000000002


#     Robot State Guide
#---------------------------
#  0 - Start State
#  1 - Traversing Out
#  2 - Mining
#  3 - Traversing Back
#  4 - Depositing
#---------------------------
#  Start State Substates
#---------------------------
#  0 - Start State
#---------------------------
#
#
#
#---------------------------
#  1   Traversing Out
#---------------------------
#  1.0 R/L High
#    1.0   "300000010100"
#  1.1 R High L Pulse
#    1.1   "300000000100"
#  1.2 L High R Pulse
#    1.2   "300000010000"
#--------------------------- 
#  2   Mining
#---------------------------
#  2.0 R/L High, Mine High, Conv High 
#    2.0   "300000110101"
#  2.1 R High L Pulse, Conv High
#    2.1   "300000100101"
#  2.2 L High R Pulse, Conv High
#    2.2   "300000110001"
#--------------------------- 
#  3   Traversing Back
#---------------------------
#  3.0 RR/LR High
#    3.0   "300000001010"
#  3.1 RR High LR Pulse
#    3.1   "300000001000"
#  3.2 LR High RR Pulse
#    3.2   "300000000010"
#--------------------------- 
#  4   Depositing
#---------------------------
#  4.0 RR/LR High, Mine High
#    4.0   "300000001011"
#  4.1 RR High LR Pulse
#    4.1   "300000001001"
#  4.2 LR High RR Pulse
#    4.2   "300000000011"
#  4.3 Dump Conveyor High
#    4.3   "300010000000"
#--------------------------- 
StartState = 0
TraversingOutState = 1
MiningState = 2
TraversingBackState = 3
DepositingState = 4
LowerState = 5
RaiseState = 6
ConveyorDumpState = 7
Spin = 8
global STATE



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


ConveyorMove = 0
FirstLower   = 0
FirstRaise   = 0
FirstDump    = 0
FirstSpin    = 0
MineTime     = 0



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
miningModeOff   = "300000000002"

TCP_IP = '192.168.1.136'
TCP_PORT = 5000
BUFFER_SIZE = 1024

def Lower():
    rospy.loginfo("Lower State")
    
    if FirstLower == 1:
        global firstTime
        firstTime = time.time();
        global FirstLower
        FirstLower = 0
    s.send(convIn)
    time_spent = time.time() - firstTime
    print(time_spent)   
    if time_spent > 3:
        global STATE
        STATE = Spin
        global FirstSpin 
        FirstSpin = 1
        print("State: %d", STATE)
        rospy.loginfo("Robot has finished Lowering. STATE change to Spinning(8)")
        return

def SpinState():
    rospy.loginfo("Spin State")
    print("SpinState")   
    if FirstSpin == 1:
        global firstTime
        firstTime = time.time();
        global FirstSpin
        FirstSpin = 0
    s.send(convFor)
    time_spent = time.time() - firstTime
    print(time_spent)
    if time_spent > 3:
        global STATE
        STATE = MiningState
        time_spent = 0
        global MineTime
        MineTime = 1
        rospy.loginfo("Robot has finished Spinning. STATE change to MiningState(2)")
        return


def Raise():
    rospy.loginfo("Raise State")
    
    if FirstRaise == 1:
        global firstTime
        firstTime = time.time();
        global FirstRaise
        FirstRaise = 0
    s.send(convOut)
    time_spent = time.time() - firstTime
    print(time_spent)
    if time_spent > 3:
        global STATE
        STATE = TraversingBackState
        rospy.loginfo("Robot has finished Raising. STATE change to TraversingBack(3)")
        return

def ConveyorDump():
    rospy.loginfo("Lower State")
    
    if FirstDump == 1:
        global firstDumpTime
        firstDumpTime = time.time();
        global FirstDump
        FirstDump = 0
    s.send(moveDumpOut)
    time_spent = time.time() - firstDumpTime
    print(time_spent)
    if time_spent > 7:
        global STATE
        STATE = TraversingOutState
        rospy.loginfo("Robot has finished Lowering. STATE change to TraversingOut(1)")
        return

def Start():
    rospy.loginfo("Start State")
    #Need Code for Centering Here But Right now, Set State to traversing
    global STATE
    STATE = TraversingOutState
    rospy.loginfo("State: %d", STATE)
    return

def TraversingOut(data):
    rospy.loginfo("Traversing Out State")
    XThreshold = 6
    i = 0;
    #Actual
    robotSize = 46		
    #robotSize = 12
    
    #Actual
    miningDistance = 180
    #miningDistance = 60
    

    #Finished Driving Out
    if data.data[1] > (miningDistance - robotSize):
        global STATE
        STATE = LowerState
        global FirstLower 
        FirstLower = 1
        rospy.loginfo("Robot has reached the mining area. STATE change to Lower(5)")
        return
    #Drive forward

    s.send(StateOneZero)


    #Too Far left
    if data.data[0] < (-1) * XThreshold:
        rospy.loginfo("Robot is too far to the left of the centerline. Correcting...")
        
        #Toggle Right High on and off to correct
        i = 0;
        while i < 100:
            if i < 50:
                a = 1
                #s.send(StateOneTwo)
            if i > 50:
                a = 1
                #s.send(StateOneZero)
            i+=1
        return
    #Too far right
    if data.data[0] > XThreshold: 
        rospy.loginfo("Robot is too far to the right of the centerline. Correcting...")
        
        #Toggle Left High on and off to correct
        i = 0;
        while i < 100:
            if i < 50:
                a = 1
                #s.send(StateOneOne)
            if i > 50:
                a = 1
                #s.send(StateOneZero)
            i+=1
        return

def Mining(data):
    rospy.loginfo("Mining State")
    miningThreshold = 6
    i = 0;

    if MineTime == 1:
        global MineTime
        MineTime = 0 
        global firstMineTime
        firstMineTime = time.time();

    #Actual
    robotSize = 46		
    #robotSize = 12

    #Actual
    miningMaxDistance = 290
    #miningMaxDistance = 84

    #Move the conveyor down into the dirt only once
    if ConveyorMove == 1:
        rospy.loginfo("Moving Conveyor Down into the Dirt")
        i = 0
        global ConveyorMove 
        ConveyorMove = 0;
        rospy.loginfo("Conveyor is Down in the Dirt")

    #Finished Mining
    if data.data[1] > (miningMaxDistance - robotSize - miningThreshold):
        global STATE
        STATE = RaiseState
        global FirstRaise 
        FirstRaise = 1
        #Turn Off Mining Mode
        #s.send(miningModeOff)
        rospy.loginfo("Robot has finished mining. STATE change to TraversingBack(3)")
        return

    #Drive Forward
    s.send(StateTwoZero)









    #Too Far Right
    if data.data[0] < (-1) * miningThreshold:
        rospy.loginfo("While Mining, Robot is too far to the left of the centerline. Correcting...")
        #Toggle Right High on and off to correct
        i = 0;
        while i < 100:
            if i < 50:
                a = 1
                #s.send(StateTwoTwo)
            if i > 50:
                a = 1
                #s.send(StateTwoZero)
            i+=1
        return
    #Too Far Left
    if data.data[0] > miningThreshold: 
        rospy.loginfo("While Mining, Robot is too far to the right of the centerline. Correcting...")
        #Toggle Left High on and off to correct
        i = 0;
        while i < 100:
            if i < 50:
                a = 1
                #s.send(StateTwoOne)
            if i > 50:
                a = 1
                #s.send(StateTwoZero)
            i+=1
        return

def TraversingBack(data):
    rospy.loginfo("Traversing Back State")
    XThreshold = 6
    i = 0;

    #Actual
    depositingDistance = 30	
    #depositingDistance =   18   

    #Finished Driving Out
    if data.data[1] < depositingDistance:
        global STATE
        STATE = DepositingState
        rospy.loginfo("Robot has reached the depositing area. STATE change to DEPOSITING(4)")
        return
    #Drive forward
    s.send(StateThreeZero)






    #Too Far left
    if data.data[0] < (-1) * XThreshold:
        rospy.loginfo("Robot is too far to the left of the centerline. Correcting...")
        
        #Toggle Right High on and off to correct
        i = 0;
        
        while i < 100:
            if i < 50:
                a = 1
                #s.send(StateThreeOne)
            if i > 50:
                a = 1
                #s.send(StateThreeZero)
            i+=1
        return
    #Too far right
    if data.data[0] > XThreshold: 
        rospy.loginfo("Robot is too far to the right of the centerline. Correcting...")
        
        #Toggle Left High on and off to correct
        i = 0;
        
        while i < 100:
            if i < 50:
                a = 1
                #s.send(StateThreeOne)
            if i > 50:
                a = 1
                #s.send(StateThreeZero)
            i+=1
        return

def Depositing(data):
    rospy.loginfo("Depositing State")
    XThreshold = 3
    i = 0;
    depositingDistance = 6     

    #Finished Driving Out
    if data.data[1] < depositingDistance:
        #Dumping Routine

        #End DUmping Routine:
        global STATE
        STATE = ConveyorDumpState

        global FirstDump 
        FirstDump = 1
        rospy.loginfo("Robot has reached the depositing area. STATE change to DEPOSITING(4)")
        return
    #Drive forward 
    s.send(StateFourZero)




    #Too Far left
    if data.data[0] < (-1) * XThreshold:
        rospy.loginfo("Robot is too far to the left of the centerline. Correcting...")
        
        #Toggle Right High on and off to correct
        i = 0;
        
        while i < 100:
            if i < 50:
                a = 1
                #s.send(StateFourOne)
            if i > 50:
                a = 1
                #s.send(StateFourZero)
            i+=1
        return
    #Too far right
    if data.data[0] > XThreshold: 
        rospy.loginfo("Robot is too far to the right of the centerline. Correcting...")
        
        #Toggle Left High on and off to correct
        i = 0;
        while i < 100:
            if i < 50:
                a = 1
                #s.send(StateFourOne)
            if i > 50:
                a = 1
                #s.send(StateFourZero)
            i+=1
        return


def TraversingOutTest(desiredDistance, data):
    TraversingOut(data)
    if data.data[1] > desiredDistance:
        rospy.loginfo("We made it to the spot. STATE CHANGE TO 2")
        STATE = 2
        sleep(20);
    else:
        rospy.loginfo("Keep going til you get there!")

def TraversingBackTest(desiredDistance, data):
    if data.data[1] < desiredDistance:
        rospy.loginfo("We made it home!. STATE CHANGE TO 3")
        STATE = 3
        sleep(20);
    else:
        rospy.loginfo("Keep going til you get there!")

def TCPCleanup():
    s.close()
changeCounter = 0

def callback(data):
    print(XChange)
    print(ZChange)
    rospy.loginfo("I heard %s",data.data)
    if changeCounter < 10:
        if XChange != -1 and ZChange != -1:
            #Is this a reasonable amount of change?
            print( ( abs(XChange - data.data[0]) / XChange ) > .15)
        
            if ( abs(XChange - data.data[0]) / XChange ) > .15 and abs(XChange - data.data[0]) > 6:
                rospy.loginfo("TOO MUCH X CHANGE")
                global changeCounter
                changeCounter += 1
                return
            elif ( abs(ZChange - data.data[1]) / ZChange ) > .15 and abs(ZChange - data.data[1]) > 6:
                rospy.loginfo("TOO MUCH Z CHANGE")
                global changeCounter
                changeCounter += 1
                return
            else:
                global changeCounter
                changeCounter = 0
    else:
        global changeCounter
        changeCounter = 0
              

    Test = 0
    #Test 1: Drive Forward until X
    if Test == 1:
        TraversingOutTest(80, data)
    elif Test == 2:
        TraversingBackTest(12, data) 
    elif Test == 3:
        hi
    elif Test == 4:
        bye
    else:

        if STATE == 0:
            Start()
        if STATE == 1:
            TraversingOut(data)
        if STATE == 2:
            Mining(data)
        if STATE == 3:
            TraversingBack(data)
        if STATE == 4:
            Depositing(data)
        if STATE == 5:
            Lower()
        if STATE == 6:
            Raise()
        if STATE == 7:
            ConveyorDump()
        if STATE == 8:
            SpinState()

        #sleep(1)
        global FirstDump

    global XChange
    XChange = data.data[0]
    global ZChange
    ZChange = data.data[1]

if __name__ == '__main__':
    
    STATE = StartState
    global ZChange
    ZChange = -1
    global XChange
    XChange = -1
    changeCounter = 0
    rospy.init_node('MoonBrain')
    rospy.Subscriber("/AlexAR", Float32MultiArray, callback)
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((TCP_IP, TCP_PORT))
    rospy.spin()

