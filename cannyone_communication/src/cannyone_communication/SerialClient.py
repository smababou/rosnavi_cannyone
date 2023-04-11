#!/usr/bin/env python3

import asyncio as asio
import serial_asyncio as serasio
import rospy as rpy
from .Publisher import SetupPublisher as SetPub

class SetupSerialClient:
    """Initialize Asyncio communication with Serial Port """
    def __init__(self, port_name=None, baud=9600):
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
        self.buf = bytes()
        rpy.loginfo('commROSMCU Reader connection created: %s', self.transport)
        self.set_publisher = SetPub()
        self.transport.serial.rts = False
        self.transport.write(b'Hello Raspberry-Pi Shake Hand\n')

    def data_received(self, data):
        """Store characters until a newline is received. """
        self.buf += data
        if rpy.is_shutdown():
            rpy.logerr ('Serial loop is shuting down after ROS Node is down')
            asio.get_event_loop().stop()
        elif b'\n' in self.buf:
            lines = self.buf.split(b'\n')
            self.buf = lines[-1]
            for line in lines[:-1]:
                # print(f'Reader received: {line.decode()}')
                self.send_data(line)

    def send_data(self, data):
        """ Send Data to Publisher"""
        topic_info = TopicInfo('vel', [float(data), 1.7])
        # topic_info = Topic_Info('delete', 'vel')
        self.set_publisher.check_keywords(topic_info)

    def connection_lost(self, exc):
        rpy.logerr ('commROSMCU Reader port closed')
        self.transport.close()
        asio.get_event_loop().stop()


class TopicInfo(object):
    """TopicInfo sets the received Serial Data into an Object"""
    header = ''
    data = float()

    def __init__(self, x, y):
        TopicInfo.header = x
        TopicInfo.data = y