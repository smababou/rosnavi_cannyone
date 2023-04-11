#!/usr/bin/env python3

import rospy as rpy
import actionlib
import asyncio as asio
import serial_asyncio as serasio
from cannyone_communication import Publisher as Pub
from cannyone_communication import SerialService as CanOneSerSrv
from cannyone_navigation.msg import CannyOnePathSgmntAction, CannyOnePathSgmntGoal

class SetupSerialClient:
	"""Initialize Asyncio communication with Serial Port """
	def __init__(self, port_name=None, baud=115200):
		""" Initialize node, and connect to bus. """
		self.loop = asio.get_event_loop()
		self.coro = serasio.create_serial_connection(self.loop, Reader, port_name, baudrate=baud)

	def serial_async_loop(self):
		""" Loop on Serial Data Asyncio """
		self.loop.run_until_complete(self.coro)
		try:
			self.loop.run_forever()
		finally:
			self.loop.close()


class Reader(asio.Protocol):
	"""Asyncio Serial Reader communicates with the serial device. """
	def connection_made(self, transport):
		""" Store the serial transport and prepare to receive data. """
		self.transport = transport
		self.buf = bytearray()
		rpy.loginfo(rpy.get_caller_id() + '\n commROSMCU Reader connection created: %s', transport)
		self.cannyone_data_analyzer = CannyOneDataAnalyzer()
		self.transport.serial.rts = False

	def data_received(self, data):
		"""Store characters until a newline is received. """
		self.buf += data
		if not rpy.is_shutdown():
			self.cannyone_data_analyzer.data_parsing(data)
		else:
			rpy.logerr (rpy.get_caller_id() + '\n Serial loop is shuting down after ROS Node is down')
			asio.get_event_loop().stop()

	def connection_lost(self, exc):
		rpy.logerr ('commROSMCU Reader port closed')
		self.transport.close()
		asio.get_event_loop().stop()


class CannyOneDataAnalyzer():
	def __init__(self):
		self.buf = bytearray()
		_action_name = 'cannyone_segmenter_action'
		baudrate = int(rpy.get_param('~baud','115200'))
		port_name = rpy.get_param('~port','/dev/ttyAMA1')
		self.log_comm_type = rpy.get_param('~logcommType','eNaviLog_t')
		self.log_comm_topic = rpy.get_param('~logcommTopic','xNaviLog')
		self.status_comm_type = rpy.get_param('~statuscommType','eState_t')
		self.status_comm_topic = rpy.get_param('~statuscommTopic','xStatusPi')
		self.PosLOT_comm_topic = rpy.get_param('~poslotcommTopic','PositionLOT')
		while not rpy.is_shutdown():
			if rpy.has_param('cannyone_communication_data'): 
				rpy.Rate(10).sleep()
				self.cannyone_communication_data = rpy.get_param('cannyone_communication_data')
				rpy.logdebug(rpy.get_caller_id() + "\n Communication Data Parameters received")
				break
			else:
				rpy.logerr_once(rpy.get_caller_id() + "\n Communication Data Parameter are not launched")
				rpy.logwarn_once(rpy.get_caller_id() + "\n Communication Data Parameter retry to launch")
		self.cannyone_path_segmenter = actionlib.SimpleActionClient(rpy.get_namespace() + _action_name, 
																		CannyOnePathSgmntAction)
		rpy.logdebug(rpy.get_caller_id() + "\n Waiting for [%s] Server", _action_name)
		self.cannyone_path_segmenter.wait_for_server()
		rpy.logdebug(rpy.get_caller_id() + "\n Waiting for Topics Publisher")
		self.set_publisher = Pub.SetupPublisher()
		self.cannyone_serial_sender = CanOneSerSrv(port_name, baudrate)
		rpy.loginfo(rpy.get_caller_id() + "\n CannyOne PData Analyzer Is Running")

	def data_parsing(self, data):
		# goal = [3000, 3000, 0]
		# self.cannyone_path_segmenter.start_cannyone_plan(goal)
		self.buf += data
		while len(self.buf) > 0 and not rpy.is_shutdown():
			if self.buf[0] in self.cannyone_communication_data['RegVal'].values():
				RegVal_list = list(self.cannyone_communication_data['RegVal'].values())
				RegKey_list = list(self.cannyone_communication_data['RegVal'].keys())
				RegVal_index = RegVal_list.index(self.buf[0])
				RegKey = RegKey_list[RegVal_index]
				line_length = int(sum(self.cannyone_communication_data['DataTyp'][RegKey])/8)
				line = self.buf[1:line_length+1]
				del self.buf[0:line_length+1]
				self.case_switcher(RegKey, line)
			else:
				rpy.logerr(rpy.get_caller_id() + "\n Restart Communication")
				rpy.logerr(rpy.get_caller_id() + "\n Local Buff Queue: %s", self.buf)
				rpy.logwarn(rpy.get_caller_id() + "\n Local Buff Queue will be cleared")
				self.buf = bytearray()
				# xNaviLog [eNaviLog_t]: NAVI_ERR_UART
				xNaviLog = self.cannyone_communication_data[self.log_comm_type]['NAVI_ERR_UART']
				# self.cannyone_serial_sender.msg_sender(self.log_comm_topic, [xNaviLog])
				# xStatusPi [eState_t]: FAILED
				xStatusPi = self.cannyone_communication_data[self.status_comm_type]['FAILED']
				# self.cannyone_serial_sender.msg_sender(self.status_comm_topic, [xStatusPi])
				break
		if rpy.is_shutdown(): asio.get_event_loop().stop()

	def case_switcher (self, RegKey, line):
		rpy.logdebug(rpy.get_caller_id() + "\n RegKey: [%s]\n line: %s", RegKey, line)
		if RegKey == "ulNextTargetPosId":
			ulNextTargetPosId = str(int.from_bytes(line, 'little'))
			if ulNextTargetPosId in self.cannyone_communication_data[self.PosLOT_comm_topic].keys():
				goal = self.cannyone_communication_data[self.PosLOT_comm_topic][ulNextTargetPosId]
				rpy.logdebug(rpy.get_caller_id() + "\n New Path goal: %s", goal)
				goal = CannyOnePathSgmntGoal(	x	  = goal[0],
												y	  = goal[1], 
												theta = goal[2]
											)
				# self.cannyone_path_segmenter.cancel_all_goals()
				self.cannyone_path_segmenter.send_goal(goal)
			else:
				rpy.logerr(rpy.get_caller_id() + "\n ulNextTargetPosId [%s] NOT in PositionLOT", 
												ulNextTargetPosId)
		elif RegKey == "xNaviType":
			if line[0] in self.cannyone_communication_data['eCMD_t'].values():
				xNaviTypeVal_list = list(self.cannyone_communication_data['eCMD_t'].values())
				xNaviTypeKey_list = list(self.cannyone_communication_data['eCMD_t'].keys())
				xNaviTypeVal_index = xNaviTypeVal_list.index(line[0])
				xNaviTypeKey = xNaviTypeKey_list[xNaviTypeVal_index]
				rpy.loginfo(rpy.get_caller_id() + "\n CannyOne Navi Type: [%s]", xNaviTypeKey)
				topic_info = TopicInfo(RegKey, [xNaviTypeKey])
				self.set_publisher.check_keywords(topic_info)
			else:
				rpy.logerr(rpy.get_caller_id() + "\n CannyOne Navi Type [0x%s] is NOT Defined", 
												line.hex())
		elif RegKey == "xVelocity":
			xVelocity = []
			for DataTyp in self.cannyone_communication_data['DataTyp']['xVelocity']:
				line_length = int(DataTyp/8)
				xVelocity.append(int.from_bytes(line[0:line_length], 'little', signed=True))
				del line[0:line_length]
			rpy.logdebug(rpy.get_caller_id() + "\n CannyOne %s: %s", RegKey, xVelocity)
			topic_info = TopicInfo(RegKey, xVelocity)
			self.set_publisher.check_keywords(topic_info)
		else:
			pass


class TopicInfo(object):
    """TopicInfo sets the received Serial Data into an Object"""
    header = ''
    data = float()

    def __init__(self, x, y):
        TopicInfo.header = x
        TopicInfo.data = y


if __name__ == '__main__':
	"""Initiate CannyOne Data Analyzer Node
	"""
	rpy.init_node('cannyone_data_analyzer_node', log_level=rpy.DEBUG)
	rpy.loginfo(rpy.get_caller_id() +  '\n CannyOne Data Analyzer Node')

	port_name = rpy.get_param('~port','/dev/ttyAMA1')
	baudrate = int(rpy.get_param('~baud','115200'))

	rpy.loginfo("Connecting to %s at %d baud" % (port_name,baudrate))
	ser_client = SetupSerialClient(port_name, baudrate)
	try:
		ser_client.serial_async_loop()
	except Exception as err:
		rpy.logerr('SerialClient failed: %s',err)


	try:
		rpy.spin()
	except (rpy.ROSException, rpy.exceptions.ROSInterruptException, rpy.ServiceException) as err:
			rpy.logerr('Data Analyzer failed: %s \n %s', rpy.get_caller_id(), err)
	except KeyboardInterrupt:
		rpy.loginfo('Shutting down ROS %s', rpy.get_caller_id())
		print('Shutting down Data Analyzer module')