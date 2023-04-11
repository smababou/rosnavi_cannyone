#!/usr/bin/env python3

import rospy as rpy
import roslib as rlb
import sys
import importlib as implib
import functools as fntools

def load_pkg_module(package, directory):
    """Check if the ROS package in the python path, then import the Package """
    path = sys.path
    try:
        implib.import_module(package)
    except ImportError:
        rlb.load_manifest(package)
    try:
        m = __import__( package + '.' + directory )
    except ImportError:
        rpy.logerr( "Cannot import package : %s"% package )
        rpy.logerr( "sys.path was " + str(path) )
        return None
    return m

def load_message(package, message):
    """Load the ROS massage formate from its own Package """
    m = load_pkg_module(package, 'msg')
    m2 = getattr(m, 'msg')
    return getattr(m2, message)

def set_attributes(obj, attr, val):
    pre, _, post = attr.rpartition('.')
    return setattr(get_attributes(obj, pre) if pre else obj, post, val)

def get_attributes(obj, attr, *args):
    def _getattr(obj, attr):
        return getattr(obj, attr, *args)
    return fntools.reduce(_getattr, [obj] + attr.split('.'))


class TopicPublisher:
    """Publisher forwards messages from the serial device to ROS

    Publish the data to the topic. If the topic has no subscribers,
    the method will return without any affect. Access to publish()
    should be locked using acquire() and release() in order to
    ensure proper message publish ordering.
    
    """

    def __init__(self, topic_info):
        """Create a new publisher. """
        # Load Topic Parameters
        data_info = rpy.get_param('data_info')
        topic, self.frame_id, package, msg_type, extra, self.topic_attr = data_info[topic_info.header].values()
        # Find and load message type
        load_msg = load_message(package, msg_type)
        # initiate Publisher
        topic = rpy.get_namespace() + topic
        self.publisher = rpy.Publisher(topic, 
                                        load_msg, 
                                        queue_size=10)
        rpy.logdebug(rpy.get_caller_id() + '\n ROS Topic will be published: %s [%s]', 
                                            topic, msg_type)
        self.rate = rpy.Rate(10) # 10hz
        self.message = load_msg()
        # Load Topic Constants
        if extra:
            sensor_info = rpy.get_param(rpy.get_namespace() + 'data_info' + 
                                                rpy.get_namespace() + extra)
            sensor_attr, sensor_constants = sensor_info.values()
            try:
                [set_attributes(self.message, i, sensor_constants[i]) for i in sensor_attr]
            except AttributeError as err:
                rpy.logerr('Sensor Constants AttributeError %s', err)
        self.rate.sleep()

    def publish_msg(self, topic_info):
        """Forward message to ROS network. """
        self.message.header.stamp = rpy.Time.now()
        self.message.header.frame_id = self.frame_id
        try:
            # Parse data
            [set_attributes(self.message, self.topic_attr[i], topic_info.data[i]) for i in range(len(self.topic_attr))]
        except AttributeError as err:
            rpy.logerr('Topic AttributeError %s', err)
        try:
            self.publisher.publish(self.message)
        except (rpy.ROSSerializationException) as err:
            rpy.logerr('Publishing message failed: %s \n %s', rpy.get_caller_id(), err)
        self.rate.sleep()


class SetupPublisher:
    """Initialize node, and attempt to negotiate topics."""

    def __init__(self):
        """Initialize publisher and callbacks dictionaries """
        rate = rpy.Rate(10)         # 10hz
        self.publishers = dict()    # id:Publishers
        self.callbacks = dict()     # id:Publisher Callbacks
        while not rpy.is_shutdown():
            if rpy.has_param('data_info'): 
                rate.sleep()
                self.data_info = rpy.get_param('data_info')
                rpy.logdebug("Data Info Parameters received")
                break
            else:
                rpy.logerr_once("Data Info Parameter are not launched")
                rpy.logwarn_once("Data Info Parameter retry to launch")
        rpy.loginfo(rpy.get_caller_id() + '\n CannyOne Special Publisher Is Running')

    def register_publisher(self, topic_info):
        """Register a new publisher and callback dictionary. """
        topic = self.data_info[topic_info.header]['topic']
        try:
            topic = rpy.get_namespace() + topic
        except AttributeError as err:
            rpy.logerr('TopicInfo AttributeError: %s', err)
        try:
            # Check if the topic is already exist on ROS Master
            if topic not in next(zip(*rpy.get_published_topics())):
                rpy.logdebug(rpy.get_caller_id() + '\n ROS Topic [%s] does not exist in the ROS Master', 
                                                    topic)
                rpy.logdebug(rpy.get_caller_id() + '\n Setup New Topic publisher on %s', topic)
                pub = TopicPublisher(topic_info)
                self.publishers[topic_info.header] = pub
                self.callbacks[topic_info.header] = pub.publish_msg
                # Publish the 1st Msg, with sleep 
                # in order to reach all subscribers
                rpy.sleep(0.3)
                self.callbacks[topic_info.header](topic_info)
            else:
                self.callbacks[topic_info.header](topic_info)
        except Exception as err:
            rpy.logerr('Creation of publisher failed: %s', err)

    def check_keywords(self, topic_info):
        """Check if the topic should stop. """
        if topic_info.header == 'delete':
            try:
                self.publishers.pop(topic_info.data)
                self.callbacks.pop(topic_info.data)
            except KeyError as err:
                rpy.logwarn('%s Topic does not Exist on ROS Master', err)
        else:
            self.register_publisher(topic_info)

