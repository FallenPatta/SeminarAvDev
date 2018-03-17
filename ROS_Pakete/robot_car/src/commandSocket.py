#!/usr/bin/env python

import rospy
import socket
import errno
import sys
import threading
import time
from std_msgs.msg import String
from sensor_msgs.msg import Joy

class CommandSocket:

    def __init__(self, sock=None):
        if sock is None:
            self.sock = socket.socket(
                            socket.AF_INET, socket.SOCK_STREAM)
        else:
            self.sock = sock
        self.radius = 400.0
        self.velocity = 0.0
        self.connected = False
        self.driveCom = False
        self.timeout_check = time.time()

    def connect(self, host, port):
        self.sock.connect((host, port))
        self.sock.setblocking(0)
        self.connected = True
        thread = threading.Thread(target=self.receiveThread, args=())
        thread.daemon = True
        thread.start()

    def setDriveCom(self, setting):
        self.driveCom = setting

    def getDriveCom(self):
        return self.driveCom

    def mysend(self, msg):
		#self.sock.send(msg)
		try:
			self.sock.send(msg)
		except:
			print("connection dropped")
			self.connected = False

    def sendCommand(self, cmd=None):
        if not self.connected:
            print("Not connected")
            return
        cmd_str = ""
        if not self.driveCom:
            cmd_str = "{"
            cmd_str += "Vel:" + str(self.velocity)
            cmd_str += ","
            cmd_str += "Rad:" + str(self.radius)
            cmd_str += "}\n"
        else:
            cmd_str = "{"
            cmd_str += "Vel:" + str(self.velocity)
            cmd_str += ","
            cmd_str += "LR:" + str(self.radius)
            cmd_str += "}\n"
        self.sock.send(cmd_str)

    def receiveThread(self):
        buffer = ''
        while not rospy.is_shutdown() and self.connected:
            try:
                data = self.sock.recv(1024)
            except socket.error, e:
                err = e.args[0]
                if err == errno.EAGAIN or err == errno.EWOULDBLOCK:
                    if buffer != '':
                        buffer = ''
                    continue
            try:
				if data:
					self.timeout_check = time.time()
					buffer += data
				else:
					self.connected = False
					print("Timeout")
					break
            except:
				continue
