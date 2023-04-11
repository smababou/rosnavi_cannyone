#!/usr/bin/env python3

import rospy as rpy
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped
from cannyone_navigation.srv import CannyOnePlan
from rosgraph_msgs.msg import Log

class CannyOneGlobalPlanner():

    def __init__(self):
        
        odom_topic = rpy.get_param('~odom_topic', 
                                    'odom')
        gplanner_service_name = rpy.get_param('~gplanner_service_name',
                                                '/move_base/GlobalPlanner/make_plan')
        self.map_frame_id = rpy.get_param('~frame_id', 
                                            'map')
        self.rate = rpy.Rate(10)
        self.current_position = Odometry()
        self.ros_log = []
        rpy.Subscriber(rpy.get_namespace() + odom_topic, 
                        Odometry, 
                        self.current_pos)
        rpy.wait_for_message(rpy.get_namespace() + 'rosout', Log)
        rpy.Subscriber(rpy.get_namespace() + 'rosout', 
                        Log, 
                        self.rosgraph_log)
        # Call ROS Services
        rpy.wait_for_service(gplanner_service_name)
        try:
            self.global_planner_client = rpy.ServiceProxy(gplanner_service_name, 
                                                            GetPlan)
        except rpy.ServiceException as err:
            rpy.logerr('Service call failed: %s \n %s', rpy.get_caller_id(), err)
        
        s = rpy.Service('cannyone_planner', CannyOnePlan, self.cannyone_planner_server)
        rpy.loginfo(rpy.get_caller_id() + "\n CannyOne Planner Server Is Running")
        self.rate.sleep()

    def current_pos(self, data):
        self.current_position = data
    
    def rosgraph_log(self, data):
        self.ros_log.append(data) 

    def global_planner_request(self, request):
        start = PoseStamped()
        start.header = self.current_position.header
        start.header.frame_id = self.map_frame_id
        start.pose = self.current_position.pose.pose
        goal = request.goal
        goal.header = start.header
        goal.header.stamp = rpy.Time.now()
        plan_response = self.global_planner_client( start, 
                                                    goal, 
                                                    0.0)
        rpy.logdebug(rpy.get_caller_id() + "\n New Global Path is requested")
        return plan_response

    def cannyone_planner_server(self, req):
        generated_plan  = CannyOnePlan()
        generated_plan.plan = self.global_planner_request(req).plan
        generated_plan.status = ""
        if len(generated_plan.plan.poses) > 0:
            for i in range(len(generated_plan.plan.poses)):
                generated_plan.plan.poses[i].header.seq = i
            generated_plan.status = 'Plan Success'
            rpy.logdebug(rpy.get_caller_id() + "\n " + generated_plan.status)
        else:
            for log_data in self.ros_log:
                if log_data.name == "/move_base" and log_data.level == 8:
                    if log_data.line == 310:
                        generated_plan.status = 'Failed to get a plan, Goal is an Obstacle/Outboundary'
                        rpy.logerr(rpy.get_caller_id() + "\n " + generated_plan.status)
                    else:
                        generated_plan.status = "Please Replan a new Path"
                        rpy.logwarn(rpy.get_caller_id() + "\n " + log_data.msg)
        del self.ros_log[:]
        return (generated_plan.plan, generated_plan.status)