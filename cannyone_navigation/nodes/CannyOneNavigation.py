#!/usr/bin/env python3

import rospy as rpy
from cannyone_navigation import CannyOneGlobalPlanner as CanOneGlPlan
from cannyone_navigation import CannyOnePathSegmenter as CanOnePthSgmnt


if __name__ == '__main__':
    """Initiate CannyOne Global Planner Node
    """
    rpy.init_node('cannyone_global_planner_req_node', log_level=rpy.DEBUG)
    rpy.loginfo(rpy.get_caller_id() +  '\n CannyOne Global Planner Request Node')

    CanOneGlPlanObj = CanOneGlPlan()
    CanOnePthSgmntObj = CanOnePthSgmnt()
    
    try:
        rpy.spin()
    except (rpy.ROSException, rpy.ROSInterruptException, rpy.ServiceException) as err:
            rpy.logerr('Global Planner Request failed: %s \n %s', rpy.get_caller_id(), err)
    except KeyboardInterrupt:
        rpy.loginfo('Shutting down ROS %s', rpy.get_caller_id())
        print('Shutting down Global Planner Request module')