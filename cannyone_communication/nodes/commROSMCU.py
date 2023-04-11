#!/usr/bin/env python3

import rospy as rpy
from cannyone_communication import SetupSerialClient as SetSerClnt


if __name__ == '__main__':
    """
    Initiate ROS MCU Communication Node
    """
    rpy.init_node('serial_data_publisher_node', log_level=rpy.DEBUG)
    rpy.loginfo(rpy.get_caller_id() +  '\n ROS Serial Publisher Node')

    port_name = rpy.get_param('~port','/dev/ttyACM0')
    baud = int(rpy.get_param('~baud','9600'))

    rpy.loginfo("Connecting to %s at %d baud" % (port_name,baud))
    ser_client = SetSerClnt(port_name, baud)
    try:
        ser_client.serial_async_loop()
    except Exception as err:
        rpy.logerr('SerialClient failed: %s',err)