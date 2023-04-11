#!/usr/bin/env python3

import rospy as rpy
from geometry_msgs.msg import PoseStamped
from cannyone_navigation.srv import CannyOnePlan
from tf_conversions import transformations as angle_trans
from itertools import groupby as gpby
from operator import itemgetter as itmgtr
from tabulate import tabulate


M_PI = 3.141592653589793

stp_distance = 10
no_points_segment = 5
# New_Goal = [3000, 0, 0]
New_Goal = [3000, 3000, 0]
# New_Goal = [3000, 3000, 4712]

class CannyOnePathSegmenter():

    def __init__(self):
        
        gplanner_service_name = '/cannyone_planner'
        self.map_frame_id = rpy.get_param('~frame_id', 
                                            'map')
        self.rate = rpy.Rate(10)
        self.step_theta_control_list = []
        rpy.wait_for_service(gplanner_service_name)
        try:
            self.global_planner_client = rpy.ServiceProxy(gplanner_service_name, 
                                                            CannyOnePlan)
        except rpy.ServiceException as err:
            rpy.logerr('Service call failed: %s \n %s', rpy.get_caller_id(), err)
        
        rpy.loginfo(rpy.get_caller_id() + "\n CannyOne Path Segmenter Is Running")
        self.rate.sleep()

    def cannyone_planner_request(self, goal_request):
        goal_msg = PoseStamped()
        goal_msg.header.stamp = rpy.Time.now()
        goal_msg.header.frame_id = self.map_frame_id
        goal_msg.pose.position.x = goal_request[0] / 1000                                   # X-position in meter
        goal_msg.pose.position.y = goal_request[1] / 1000                                   # Y-position in meter
        goal_request[2] = (goal_request[2] / 1000 + M_PI) % (2 * M_PI) - M_PI               # orientation in radian [-π, π]
        goal_request_orientation = angle_trans.quaternion_from_euler(0.0, 
                                                                     0.0, 
                                                                     goal_request[2]
                                                                    )
        goal_msg.pose.orientation.z = goal_request_orientation[2]
        goal_msg.pose.orientation.w = goal_request_orientation[3]
        for i in range(5):
            rpy.logdebug(rpy.get_caller_id() + "\n CannyOne Global Path Request Trial: %s", i+1)
            plan_response = self.global_planner_client(goal_msg)
            if plan_response.status == "Plan Success": break
            elif "Replan" in plan_response.status: 
                rpy.logwarn(rpy.get_caller_id() + "\n Recalculating new Path")
            elif "Failed" in plan_response.status:
                rpy.logerr(rpy.get_caller_id() + "\n " + plan_response.status)
                break
        rpy.logdebug(rpy.get_caller_id() + "\n New CannyOne Global Path is requested\n Goal (x: %s, y: %s, theta: %s)", 
                                            goal_msg.pose.position.x, goal_msg.pose.position.y, goal_request[2])
        listed_path = []
        if plan_response.status == "Plan Success":
            for point in plan_response.plan.poses:
                position = point.pose.position
                orientation = point.pose.orientation
                orientation = angle_trans.euler_from_quaternion([orientation.x, 
                                                                 orientation.y, 
                                                                 orientation.z, 
                                                                 orientation.w
                                                                ])[2]
                orientation %= 2 * M_PI
                listed_path.append ([ point.header.seq,
                                    int(position.x * 1000),                                 # integer X-position in millimeter
                                    int(position.y * 1000),                                 # integer Y-position in millimeter
                                    int(orientation * 1000)                                 # integer orientation in milliradian [0, 2π]
                            ])
        rpy.logdebug(rpy.get_caller_id() + "\n New CannyOne Global Listed Path Length: %s",  len(listed_path))
        return listed_path

    def path_steps_control(self, full_path, step_distance = 1):
        rpy.logdebug(rpy.get_caller_id() + "\n Path step control distance: %s",  step_distance)
        step_control_list = full_path[::step_distance]
        if full_path[-1] != step_control_list[-1]: step_control_list.append(full_path[-1])
        return step_control_list

    def path_turns_control(self, full_path, theta_threshold = M_PI/30 * 1000):
        rpy.logdebug(rpy.get_caller_id() + "\n Path turns control theta threshold: %s mili-rad",  
                                            theta_threshold)
        pre_control_list = []
        theta_control_list = []
        for itr in range(len(full_path)-1):
            if abs(full_path[itr+1][3]-full_path[itr][3]) > (theta_threshold):
                pre_control_list.append(full_path[itr])
        ids_list = list(map(itmgtr(0), pre_control_list))
        for k, g in gpby(enumerate(ids_list), lambda x: x[1]-x[0]) :
            index = list(map(itmgtr(0), g))
            theta_control_list.append(pre_control_list[index[0]])
        rpy.logdebug(rpy.get_caller_id() + "\n Points More than threshold theta: %s",  
                                            theta_control_list)
        return theta_control_list

    def step_theta_control(self, full_path, step_distance):
        step_control_list = self.path_steps_control(full_path, step_distance)
        theta_control_list = self.path_turns_control(full_path)
        for theta_control_point in theta_control_list:
            if theta_control_point not in step_control_list:
                for step_control_point in step_control_list:
                    if theta_control_point[0] < step_control_point[0]:
                        index = step_control_list.index(step_control_point)
                        step_control_list.insert(index, theta_control_point)
                        break
        for iter, val in enumerate(step_control_list):
            val[0] = iter
        return step_control_list

    def path_segmenting(self, step_distance = 1, number_points_segment = 5, new_path = True, goal = [0, 0, 0]):
        if new_path:
            full_path = self.cannyone_planner_request(goal)
            self.step_theta_control_list = self.step_theta_control(full_path, step_distance)
            next_segment = self.step_theta_control_list[:number_points_segment]
        else:
            next_segment = self.step_theta_control_list[:number_points_segment]
        return (next_segment, len(self.step_theta_control_list)-number_points_segment)

    def start_cannyone_plan(self):
        segment, remaining = self.path_segmenting(stp_distance, no_points_segment, True, New_Goal)
        while (remaining > 0):
            # rpy.sleep(1.0)
            self.step_theta_control_list.pop(0)
            segment, remaining = self.path_segmenting(stp_distance, no_points_segment, False)
            # print(tabulate(segment, headers=["ID", "X", "Y", "Theta"]))
            # print("Remaining: ", remaining)
        print(tabulate(segment, headers=["ID", "X", "Y", "Theta"]))
        print("Remaining: ", remaining)

if __name__ == '__main__':
    """Initiate CannyOne Path Segmentation Node
    """
    rpy.init_node('cannyone_path_segmentation_node', log_level=rpy.DEBUG)
    rpy.loginfo(rpy.get_caller_id() +  '\n CannyOne Path Segmentation Node')

    obj = CannyOnePathSegmenter()
    obj.start_cannyone_plan()

    try:
        rpy.spin()
    except (rpy.ROSException, rpy.exceptions.ROSInterruptException, rpy.ServiceException) as err:
            rpy.logerr('Path Segmentation failed: %s \n %s', rpy.get_caller_id(), err)
    except KeyboardInterrupt:
        rpy.loginfo('Shutting down ROS %s', rpy.get_caller_id())
        print('Shutting down Path Segmentation module')