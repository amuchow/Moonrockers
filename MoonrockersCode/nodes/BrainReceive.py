#!/usr/bin/env python

import socket


TCP_IP = '127.0.0.1'
TCP_PORT = 5005
BUFFER_SIZE = 11  # Normally 1024, but we want fast response

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((TCP_IP, TCP_PORT))
s.listen(1)

conn, addr = s.accept()
print 'Connection address:', addr
while 1:
    data = conn.recv(BUFFER_SIZE)
    if data:
        print "received data:", data
        if data[0] == '1':
            print "DumpIn HIGH\n"
        if data[1] == '1':
            print "ConvRev HIGH\n"
        if data[2] == '1':
            print "ConvOut HIGH\n"
        if data[3] == '1':
            print "DumpOut HIGH\n"
        if data[4] == '1':
            print "ConvIn HIGH\n"
        if data[5] == '1':
            print "ConvForward HIGH\n"
        if data[6] == '1':
            print "lWhFor HIGH\n"
        if data[7] == '1':
            print "lWhRev HIGH\n"
        if data[8] == '1':
            print "rWhFor HIGH\n"
        if data[9] == '1':
            print "rWhRev HIGH\n"
        if data[10] == '1':
            print "Mining mode HIGH\n"
conn.close()
