#!/usr/bin/env python3

import serial
import rospy as rpy

class SerialService():

	def __init__(self, port_name = None, baud_rate = 115200):
		self.serial_port = serial.Serial(port_name, baud_rate)
		if self.serial_port.isOpen():
			rpy.loginfo(rpy.get_caller_id() + "\n Serial Service is Ready Port[%s] Baudrate[%s]", 
											self.serial_port.name, self.serial_port._baudrate)
		else:
			rpy.logerr(rpy.get_caller_id() + "\n Serial Port Error Port[%s] Baudrate[%s]", 
											self.serial_port.name, self.serial_port._baudrate)
		while not rpy.is_shutdown():
			if rpy.has_param('cannyone_communication_data'): 
				rpy.Rate(10).sleep()
				self.cannyone_communication_data = rpy.get_param('cannyone_communication_data')
				rpy.logdebug("Communication Data Parameters received")
				break
			else:
				rpy.logerr_once("Communication Data Parameter are not launched")
				rpy.logwarn_once("Communication Data Parameter retry to launch")

	def msg_encoding(self, data_types, data):
		data_bytes = bytearray()
		for data_type, value in zip(data_types, data):
			if isinstance(value, list):
				for sub_data_type, sub_value in zip(data_type, value):
					padding = int(sub_data_type/8)
					data_bytes += sub_value.to_bytes(padding, 'big')
			else:		
				padding = int(data_type/8)
				data_bytes += value.to_bytes(padding, 'big')
		return(data_bytes)

	def bytes_sender(self, data_bytes):
		for data_byte in data_bytes:
			hex_byte = ("{0:02x}".format(data_byte))
			self.serial_port.write(bytearray.fromhex(hex_byte))

	def msg_sender(self, Topic, data):
		DataTyp = self.cannyone_communication_data['DataTyp'][Topic]
		if Topic == 'xaPathSegments':
			# Extend the DataTyp, of the second element Typ, 
			# length of the segment - the available one in DataTyp 
			DataTyp.extend( (data[0]-1) * [DataTyp[1]] )
		encoded_msg = self.msg_encoding(DataTyp, data)
		RegVal = bytearray([self.cannyone_communication_data['RegVal'][Topic]])
		rpy.logdebug_once(rpy.get_caller_id() + "\n Communication data ready Topic[%s] DataTyp[%s] RegVal[%s]",
												Topic, DataTyp, RegVal)
		rpy.logdebug_once(rpy.get_caller_id() + "\n Communication MSG[%s]",
												encoded_msg)
		self.serial_port.write(RegVal + encoded_msg)
