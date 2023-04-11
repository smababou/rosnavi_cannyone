#!/usr/bin/env python3

import rospy as rpy
from nav_msgs.msg import Odometry

class TopicPublisher:

    def __init__(self):
        """Create a new publisher. """

        topic = rpy.get_namespace() + 'cannyone_position'
        self.publisher = rpy.Publisher(topic, 
                                        Odometry, 
                                        queue_size=10)
        rpy.loginfo('ROS Topic will be published: %s [Odometry Msg]', topic)
        self.rate = rpy.Rate(10) # 10hz
        self.message = Odometry()
        self.rate.sleep()

    def publish_msg(self):
        """Forward message to ROS network. """
        self.message.header.stamp = rpy.Time.now()
        self.message.header.frame_id = 'odom'
        self.message.child_frame_id = 'base_footprint'
        self.message.pose.pose.position.x = 7.0050
        self.message.pose.pose.position.y = 3.5100
        self.message.pose.pose.orientation.z = 0.000
        self.message.pose.pose.orientation.w = 1.000
        self.message.pose.covariance = [0.01, 0.0,  0.0,  0.0,  0.0,  0.0,
                                        0.0,  0.01, 0.0,  0.0,  0.0,  0.0,
                                        0.0,  0.0,  0.01, 0.0,  0.0,  0.0,
                                        0.0,  0.0,  0.0,  0.1,  0.0,  0.0,
                                        0.0,  0.0,  0.0,  0.0,  0.1,  0.0,
                                        0.0,  0.0,  0.0,  0.0,  0.0,  0.1]
        try:
            self.publisher.publish(self.message)
        except (rpy.ROSSerializationException) as err:
            rpy.logerr('Publishing message failed: %s \n %s', rpy.get_caller_id(), err)
        self.rate.sleep()


if __name__ == '__main__':
    """Initiate CannyOne Localization Node
    """
    rpy.init_node('cannyone_localization_node', log_level=rpy.INFO)
    rpy.loginfo(rpy.get_caller_id() +  '\n CannyOne Localization Node')

    obj = TopicPublisher()
    while not rpy.is_shutdown():
      obj.publish_msg()


    try:
        rpy.spin()
    except (rpy.ROSException, rpy.ROSInterruptException, rpy.ServiceException) as err:
            rpy.logerr('CannyOne Localization failed: %s \n %s', rpy.get_caller_id(), err)
    except KeyboardInterrupt:
        rpy.loginfo('Shutting down ROS %s', rpy.get_caller_id())
        print('Shutting down CannyOne Localization module')