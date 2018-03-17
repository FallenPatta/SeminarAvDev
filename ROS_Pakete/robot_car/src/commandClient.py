#!/usr/bin/env python

import rospy
import socket
import errno
import sys
import threading
import time
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class CommandProcessor:
	
	def __init__(self, comSocket):
		self.commandSocket = comSocket
		
	def commandCallback(self, data):
		vel = 10.0*data.linear.x
		rad = 400.0
		if abs(data.angular.z) > 0.01:
			rad = vel/data.angular.z
		else:
			rad = 400.0
		if abs(rad) < 10:
			rad = (rad/abs(rad))*10
		self.commandSocket.radius = rad
		self.commandSocket.velocity = vel
