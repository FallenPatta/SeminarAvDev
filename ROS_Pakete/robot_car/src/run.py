#!/usr/bin/env python

import rospy
import socket
import errno
import sys
import threading
import time
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from commandSocket import CommandSocket
from commandClient import CommandProcessor

sock = CommandSocket(sock=None)
comProcessor = CommandProcessor(sock)

def sign(num):
	if num >= 0:
		return 1
	return -1

def callback(data):
    #print("callback")
    #rospy.loginfo(rospy.get_caller_id())
    #rospy.loginfo(data.header)
    #rospy.loginfo(data.axes)
    #rospy.loginfo(data.buttons)
    speed = 8.0
    curve = 15.0
    if(data.buttons[6] == 1):
        sock.setDriveCom(True)
        sock.radius = 0.0
        sock.velocity = 0.0
        print "DIRECT COM"
    if(data.buttons[7] == 1):
        sock.setDriveCom(False)
        sock.radius = 400.0
        sock.velocity = 0.0
        print "INDIRECT COM"

    if sock.getDriveCom():
        rad = -1*data.axes[0]
        if abs(rad) < 0.1:
            rad = 0
        vel = data.axes[4]
        if abs(vel) < 0.1:
            vel = 0
        sock.radius = rad
        sock.velocity = vel
    else:
        rad = 400.0
        vel = 0.0
        if(data.buttons[0] == 1):
            print("forward")
            vel = speed
        if(data.buttons[3] == 1):
            print("backward")
            vel = -1.0*speed
        if data.buttons[4] == 1 or data.buttons[5] == 1:
            rad = 0.0
        if(data.buttons[4] == 1):
            print("left")
            rad -= curve
        if(data.buttons[5] == 1):
            print("right")
            rad += curve
        sock.radius = rad
        sock.velocity = vel

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/joy", Joy, callback)
    rospy.Subscriber("/cmd_vel", Twist, comProcessor.commandCallback)

if __name__ == '__main__':
    print("Starting")
    listener()
    while not rospy.is_shutdown():
		print("Connecting")
		while not sock.connected and not rospy.is_shutdown():
			try:
				sock.connect("192.168.178.55", 5000)
			except:
				#print("Cannot connect")
				time.sleep(0.5)
		print("Running update-loop")
		while not rospy.is_shutdown() and sock.connected:
			#print("sending")
			sock.sendCommand()
			time.sleep(0.1)
			if (time.time()-sock.timeout_check) > 3:
				print("Timeout")
				sock.sock.close()
				sock.connected = False
				sock = CommandSocket(sock=None)
				comProcessor.commandSocket = sock
