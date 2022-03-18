#!/usr/bin/env python3
#***********************************************************************
#
# NRS-6050 Ros node
#
# ----------------------------------------------------------------------
#
# Author: Kevin Galassi <kevin.galassi2@unibo.it>
# Date: 18-03-2022
#
# ----------------------------------------------------------------------
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
#
#***********************************************************************/
from operator import truediv
import struct
import socket

import rospy

from geometry_msgs.msg import WrenchStamped



from nordbo_lrs6.srv import startSensor, stopSensor, tareSensor, setDataRate

IP_ADDR	= '192.168.0.100';
PORT	= 2001;

CMD_TYPE_SENSOR_TRANSMIT 	= '07'
SENSOR_TRANSMIT_TYPE_START = '01'
SENSOR_TRANSMIT_TYPE_STOP 	= '00'

CMD_TYPE_SET_CURRENT_TARE 	= '15'
SET_CURRENT_TARE_TYPE_NEGATIVE	= '01'

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)	

class nordboForceSensor (object) :

	def __init__(self, force_topic_name='wrench_data', verbose = False) :

		print('Nordbo Force Sensor : Configuration Start')

		self.topic_name = force_topic_name
		self.verbose = verbose

		self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)	
		self.socket.settimeout(2.0)
		self.socket.connect((IP_ADDR, PORT))


		self.wrench_pub = rospy.Publisher('nordo/{}'.format(self.topic_name), WrenchStamped, queue_size=1)

		self.start_srv = rospy.Service('nordbo/start_sensor', startSensor, self.start_sensor_cb )
		self.stop_srv = rospy.Service('nordbo/stop_sensor', stopSensor, self.stop_sensor_cb )
		self.tare_srv = rospy.Service('nordbo/tare_sensor', tareSensor, self.tare_sensor_cb )

		self.sensor_active = True

		sendData = '03' + CMD_TYPE_SET_CURRENT_TARE + SET_CURRENT_TARE_TYPE_NEGATIVE
		sendData = bytearray.fromhex(sendData)
		self.socket.send(sendData)
		recvData = self.recvMsg()

		sendData = '03' + CMD_TYPE_SENSOR_TRANSMIT + SENSOR_TRANSMIT_TYPE_START
		sendData = bytearray.fromhex(sendData)
		self.socket.send(sendData)
		recvData = self.recvMsg()

		print('Nordbo Force Sensor : Configuration Completed')

		return

	def start_sensor_cb(self) :
		print('Nordbo Force Sensor : Start Requested')

		sendData = '03' + CMD_TYPE_SENSOR_TRANSMIT + SENSOR_TRANSMIT_TYPE_START
		sendData = bytearray.fromhex(sendData)
		self.socket.send(sendData)
		recvData = self.recvMsg()

		return

	def stop_sensor_cb(self) :
		print('Nordbo Force Sensor : Stop Requested')

		sendData = '03' + CMD_TYPE_SENSOR_TRANSMIT + SENSOR_TRANSMIT_TYPE_STOP
		sendData = bytearray.fromhex(sendData)
		self.socket.send(sendData)
		recvData = self.recvMsg()

		return 

	def tare_sensor_cb(self) :
		print('Nordbo Force Sensor : Tare Requested')

		sendData = '03' + CMD_TYPE_SET_CURRENT_TARE + SET_CURRENT_TARE_TYPE_NEGATIVE
		sendData = bytearray.fromhex(sendData)
		self.socket.send(sendData)
		recvData = self.recvMsg()

		print('Nordbo Force Sensor : Tare Ok')
		return

	def recvMsg(self):
		recvData = bytearray(self.socket.recv(2))

		while len(recvData) < recvData[0] :
			recvData += bytearray(self.socket.recv(recvData[0] - len(recvData)))

		if self.verbose : printMsg(recvData)

		return recvData



	def run(self):

		while not rospy.is_shutdown() :
			rospy.loginfo_throttle(5, 'Force Sensor State : Ok')
		
			if self.sensor_active :
				new_wrench = WrenchStamped()
				new_wrench.header.stamp = rospy.Time().now()

				recvData = self.recvMsg()
				new_wrench.wrench.force.x = struct.unpack('!d', recvData[2:10])[0]
				new_wrench.wrench.force.y = struct.unpack('!d', recvData[10:18])[0]
				new_wrench.wrench.force.z  = struct.unpack('!d', recvData[18:26])[0]
				new_wrench.wrench.torque.x =struct.unpack('!d', recvData[26:34])[0]
				new_wrench.wrench.torque.y = struct.unpack('!d', recvData[34:42])[0]
				new_wrench.wrench.torque.z = struct.unpack('!d', recvData[42:50])[0]
				#print(str(Fx) + " " + str(Fy) + " " + str(Fz) + " " + str(Tx) + " " + str(Ty) + " " + str(Tz))

				self.wrench_pub.publish(new_wrench)

			else :
				pass






def printMsg(msg):
	print("Msg len: " + str(msg[0]) + " Msg type: " + str(msg[1]) + "")
	
	dataStr = "DATA: "
	for i in range(msg[0] - 2):
		dataStr += str(msg[i + 2]) + " "

	print(dataStr)

if __name__ == "__main__":

	rospy.init_node('nordbo_sensor')

	sensor = nordboForceSensor()

	sensor.run()

