#!/usr/bin/env python3

import rospy as rpy
from cannyone_localization import CannyOneLocationPublisher as CanOneLocPub

if __name__ == '__main__':

    rpy.init_node('cannyone_localization_node', log_level=rpy.DEBUG)
    rpy.loginfo(rpy.get_caller_id() +  '\n CannyOne Localization Node')

    CannyOneLocObj = CanOneLocPub()

    try:
        rpy.spin()
    except (rpy.ROSException, rpy.exceptions.ROSInterruptException, rpy.ServiceException) as err:
            rpy.logerr('CannyOne Localization failed: %s \n %s', rpy.get_caller_id(), err)
    except KeyboardInterrupt:
        rpy.loginfo('Shutting down ROS %s', rpy.get_caller_id())
        print('Shutting down CannyOne Localization module')