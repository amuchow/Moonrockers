#!/usr/bin/env python
import rospy
import socket
from time import sleep
from std_msgs.msg import String
#           TCP Packet Guide        
#  -----------------------------------
#  1.  Move Dump In      - 10000000000
#  2.  Conveyor Reverse  - 01000000000
#  3.  Conveyor Out      - 00100000000
#  4.  Move Dump Out     - 00010000000
#  5.  Conveyor In       - 00001000000
#  6.  Conveyor Forward  - 00000100000
#  7.  L-Wheel Forward   - 00000010000
#  8.  L-Wheel Reverse   - 00000001000
#  9.  R-Wheel Forward   - 00000000100
#  10. R-Wheel Reverse   - 00000000010
#  11. Mining Mode On    - 00000000001
#  12. Mining Mode Off   - 00000000002

moveDumpIn = "10000000000"
convRev = "01000000000"
convOut = "00100000000"
moveDumpOut = "00010000000"
convIn = "00001000000"
convFor = "00000100000"
lWhFor = "00000010000"
lWhRev = "00000001000"
rWhFor = "00000000100"
rWhRev = "00000000010"
miningModeOn = "00000000001"
miningModeOff = "00000000002"

TCP_IP = '127.0.0.1'
TCP_PORT = 5005
BUFFER_SIZE = 1024
MESSAGE = "Hello, World!"

"""
	s.send(moveDumpIn)
	sleep(1);
	s.send(convRev)
	sleep(1);
	s.send(convOut)
	sleep(1);
	s.send(moveDumpOut)
	sleep(1);
	s.send(convIn)
	sleep(1);
	s.send(convFor)
	sleep(1);
	s.send(lWhFor)
	sleep(1);
	s.send(moveDumpIn)
	sleep(1);
	s.send(lWhRev)
	sleep(1);
	s.send(rWhRev)
	sleep(1);
	s.send(miningModeOn)
	sleep(1);
"""

def TCPCleanup():
	s.close()

def callback(data):
	rospy.loginfo("I heard %s",data.data)
	s.send(moveDumpIn)
	sleep(1);

if __name__ == '__main__':
	rospy.init_node('MoonBrain')
	rospy.Subscriber("/AlexAR", String, callback)
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.connect((TCP_IP, TCP_PORT))
	rospy.spin()

