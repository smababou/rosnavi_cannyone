#!/usr/bin/env python3

"""Testing Python Script for launching ROS Publisher"""

import rospy
from roslib import load_manifest
from sys import path
from importlib import import_module
from functools import reduce


def loadPkgModule(package, directory):
    """Check if the ROS package in the python path, then import the Package """
    lpath = path
    try:
        import_module(package)
    except ImportError:
        load_manifest(package)
    try:
        m = __import__( package + '.' + directory )
    except ImportError:
        rospy.logerr( "Cannot import package : %s"% package )
        rospy.logerr( "sys.path was " + str(lpath) )
        return None
    return m

def loadMessage(package, message):
    """Load the ROS massage formate from its own Package """
    m = loadPkgModule(package, 'msg')
    m2 = getattr(m, 'msg')
    return getattr(m2, message)

def rSetAttr(obj, attr, val):
    pre, _, post = attr.rpartition('.')
    return setattr(rGetAttr(obj, pre) if pre else obj, post, val)

def rGetAttr(obj, attr, *args):
    def _getattr(obj, attr):
        return getattr(obj, attr, *args)
    return reduce(_getattr, [obj] + attr.split('.'))


class Publisher:
    """Publisher forwards messages from the serial device to ROS

    Publish the data to the topic. If the topic has no subscribers,
    the method will return without any affect. Access to publish()
    should be locked using acquire() and release() in order to
    ensure proper message publish ordering.
    
    """

    def __init__(self, topicInfo):
        """ Create a new publisher. """
        # Load Topic Parameters
        dataInfo = rospy.get_param(rospy.get_namespace() + 'data_info')
        topic, package, msgType, extra, self.topicAttr = dataInfo[topicInfo.header].values()
        # Find and load message type
        loadMsg = loadMessage(package, msgType)
        # initiate Publisher
        topic = rospy.get_namespace() + topicInfo.header
        self.publisher = rospy.Publisher(topic, 
                                        loadMsg, 
                                        queue_size=10)
        rospy.loginfo('ROS Topic will be published: %s [%s]', topic, msgType)
        rate = rospy.Rate(30) # 30hz
        self.message = loadMsg()
        # Load Topic Constants
        if extra:
            sensorInfo = rospy.get_param(rospy.get_namespace() + 'data_info' + 
                                                rospy.get_namespace() + extra)
            sensorAttr, sensorConstants = sensorInfo.values()
            try:
                [rSetAttr(self.message, i, sensorConstants[i]) for i in sensorAttr]
            except AttributeError as err:
                rospy.logerr('Sensor Constants AttributeError %s', err)
        rate.sleep()

    def handlePacket(self, topicInfo):
        """ Forward message to ROS network. """
        self.message.header.stamp = rospy.Time.now()
        self.message.header.frame_id = topicInfo.header
        try:
            # Parse data
            [rSetAttr(self.message, self.topicAttr[i], topicInfo.data[i]) for i in range(len(self.topicAttr))]
        except AttributeError as err:
            rospy.logerr('Topic AttributeError %s', err)
        try:
            self.publisher.publish(self.message)
        except (rospy.ROSSerializationException) as err:
            rospy.logerr('Publishing message failed: %s \n %s', rospy.get_caller_id(), err)


class commROSMCU:
    """commROSMCU attempt to negotiate topics."""

    def __init__(self):
        """Initialize publisher and callbacks dictionaries """

        self.publishers = dict()    # id:Publishers
        self.callbacks = dict()     # id:Publisher Callbacks

    def setupPublisher(self, topicInfo):
        """ Register a new publisher. """
        topic = rospy.get_namespace() + topicInfo.header
        try:
            # Check if the topic is already exist on ROS Master
            if topic not in next(zip(*rospy.get_published_topics())):
                rospy.loginfo('ROS Topic [%s] does not exist in the ROS Master', topic)
                rospy.loginfo('Setup publisher on %s', topicInfo.header)
                pub = Publisher(topicInfo)
                self.publishers[topicInfo.header] = pub
                self.callbacks[topicInfo.header] = pub.handlePacket
            else:
                self.callbacks[topicInfo.header](topicInfo)
        except Exception as err:
            rospy.logerr('Creation of publisher failed: %s', err)

    def run (self, cnt, x):
        srDataInstance = Topic_Info(x[0], x[1])
        self.setupPublisher(srDataInstance)


class Topic_Info(object):
    header = ''
    data = float()

    def __init__(self, x, y):
        Topic_Info.header = x
        Topic_Info.data = y


if __name__ == '__main__':

    srData= [['vel', [3.509, 1.5]], 
            ['dis_side_right',[4.509]], 
            ['imu', [4.509, 24.01, 123.2, 124.1, 42.3, 0.23, 21.12, 43.01, 0.142, 1.09]]
            ]

    rospy.init_node('serial_data_publisher_node', log_level=rospy.DEBUG)
    rospy.loginfo(rospy.get_caller_id() +  '\n ROS Serial Publisher Node')

    # Initiate ROS MCU Communication Node
    req = commROSMCU()

    while not rospy.is_shutdown():
        try:
            # Looping on Sensor Data
            reduce(req.run, srData, 0)
        except (rospy.ROSException, rospy.ROSInterruptException, rospy.ServiceException) as err:
            rospy.logerr('Creation ROS Serial Publisher Node failed: %s \n %s', rospy.get_caller_id(), err)
        except KeyboardInterrupt:
            rospy.loginfo('Shutting down ROS %s', rospy.get_caller_id())
            print("Shutting down ROS Serial Publisher module")