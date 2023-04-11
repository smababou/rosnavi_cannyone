#!/usr/bin/env python3

import rospy as rpy
import actionlib
from itertools import groupby as gpby
from operator import itemgetter as itmgtr
from geometry_msgs.msg import PoseStamped
from cannyone_navigation.srv import CannyOnePlan
from tf_conversions import transformations as angle_trans
from cannyone_communication import SerialService as CanOneSerSrv
from cannyone_navigation.msg import CannyOnePathSgmntAction, CannyOnePathSgmntFeedback, CannyOnePathSgmntResult


M_PI = 3.141592653589793

stp_distance = 10
no_points_segment = 5
agv_speed = 300         # Velocity [mm/s]
map_accuracy = 3        # [mm]
hz = int((agv_speed / map_accuracy) / stp_distance)

# Test Goal Data
# New_Goal = [3000, 0, 0]
# New_Goal = [3000, 3000, 0]
# New_Goal = [3000, 3000, 4712]

class CannyOnePathSegmenter():
    # create messages that are used to publish feedback/result
    _feedback = CannyOnePathSgmntFeedback()
    _result = CannyOnePathSgmntResult()

    def __init__(self):

        self.xStatusPi = ''
        self.rate = rpy.Rate(10) # 10hz
        _action_name = 'cannyone_segmenter_action'
        baudrate = int(rpy.get_param('~baud','115200'))
        port_name = rpy.get_param('~port','/dev/ttyAMA1')
        self.map_frame_id = rpy.get_param('~frame_id', 'map')
        gplanner_service_name = rpy.get_param('~planner', 'cannyone_planner')
        self.log_comm_type = rpy.get_param('~logcommType','eNaviLog_t')
        self.log_comm_topic = rpy.get_param('~logcommTopic','xNaviLog')
        self.status_comm_type = rpy.get_param('~statuscommType','eState_t')
        self.status_comm_topic = rpy.get_param('~statuscommTopic','xStatusPi')
        self.PathSgmt_comm_topic = rpy.get_param('~pathcommTopic','xaPathSegments')
        rpy.logdebug(rpy.get_caller_id() + "\n Waiting for [%s] service", gplanner_service_name)
        rpy.wait_for_service(rpy.get_namespace() + gplanner_service_name)
        try:
            self.global_planner_client = rpy.ServiceProxy(rpy.get_namespace() + gplanner_service_name, 
                                                                        CannyOnePlan)
        except rpy.ServiceException as err:
            rpy.logerr('Service call failed: %s \n %s', rpy.get_caller_id(), err)
        while not rpy.is_shutdown():
            if rpy.has_param('cannyone_communication_data'): 
                rpy.sleep(0.1)
                self.cannyone_communication_data = rpy.get_param('cannyone_communication_data')
                rpy.logdebug(rpy.get_caller_id() + "\n Communication Data Parameters received")
                break
            else:
                rpy.logerr_once(rpy.get_caller_id() + "\n Communication Data Parameter are not launched")
                rpy.logwarn_once(rpy.get_caller_id() + "\n Communication Data Parameter retry to launch")
            self.rate.sleep()
        self.cannyone_serial_sender = CanOneSerSrv(port_name, baudrate)
        self._path_sgmnt_action_server = actionlib.SimpleActionServer(_action_name, 
                                                                        CannyOnePathSgmntAction, 
                                                                        execute_cb=self.start_cannyone_plan, 
                                                                        auto_start = False)
        self._path_sgmnt_action_server.start()
        rpy.loginfo(rpy.get_caller_id() + "\n CannyOne Path Segmenter Action Server Is Running")
        rpy.logdebug(rpy.get_caller_id() + "\n Path Segment can be sent with max %s Hertz", hz)

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
            self._feedback.status = 'CannyOne Global Path Request Trial: ' + str(i+1)
            self._path_sgmnt_action_server.publish_feedback(self._feedback)
            plan_response = self.global_planner_client(goal_msg)
            if plan_response.status == "Plan Success":
                self._feedback = plan_response
                self._path_sgmnt_action_server.publish_feedback(self._feedback)
                # xNaviLog [eNaviLog_t]: NAVI_OK
                xNaviLog = self.cannyone_communication_data[self.log_comm_type]['NAVI_OK']
                self.cannyone_serial_sender.msg_sender(self.log_comm_topic, [xNaviLog])
                # xStatusPi [eState_t]: OK
                self.xStatusPi = self.cannyone_communication_data[self.status_comm_type]['OK']
                self.cannyone_serial_sender.msg_sender(self.status_comm_topic, [self.xStatusPi])
                break
            elif "Replan" in plan_response.status: 
                rpy.logwarn(rpy.get_caller_id() + "\n Recalculating new Path")
                self._feedback = plan_response
                self._path_sgmnt_action_server.publish_feedback(self._feedback)
                # xNaviLog [eNaviLog_t]: NAVI_CALC_PATH_BUSY
                xNaviLog = self.cannyone_communication_data[self.log_comm_type]['NAVI_CALC_PATH_BUSY']
                self.cannyone_serial_sender.msg_sender(self.log_comm_topic, [xNaviLog])
            elif "Failed" in plan_response.status:
                rpy.logerr(rpy.get_caller_id() + "\n " + plan_response.status)
                self._feedback = plan_response
                self._path_sgmnt_action_server.publish_feedback(self._feedback)
                # xNaviLog [eNaviLog_t]: NAVI_ERR_TARGET_UNREACHABLE
                xNaviLog = self.cannyone_communication_data[self.log_comm_type]['NAVI_ERR_TARGET_UNREACHABLE']
                self.cannyone_serial_sender.msg_sender(self.log_comm_topic, [xNaviLog])
                # xStatusPi [eState_t]: FAILED
                self.xStatusPi = self.cannyone_communication_data[self.status_comm_type]['FAILED']
                self.cannyone_serial_sender.msg_sender(self.status_comm_topic, [self.xStatusPi])
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
        if full_path and full_path[-1] != step_control_list[-1]: step_control_list.append(full_path[-1])
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
        # remove the PosIds
        step_control_list = list(map(lambda x: x[1:], step_control_list))
        return step_control_list

    def start_cannyone_plan(self, New_Goal):
        New_Goal = [New_Goal.x, New_Goal.y, New_Goal.theta]
        self._feedback.status = 'New Goal received: ' + ' '.join(str(i) for i in New_Goal)
        self._path_sgmnt_action_server.publish_feedback(self._feedback)
        # Calculate Full Path
        full_path = self.cannyone_planner_request(New_Goal)
        # StepDist and Theta Control
        xaPathSegments = self.step_theta_control(full_path, stp_distance)
        if self._path_sgmnt_action_server.is_preempt_requested():
            self._feedback = CannyOnePathSgmntFeedback()
            if self._path_sgmnt_action_server.is_new_goal_available():
                self._feedback.status = 'Segmenting is recalled using new coming goal'
                self._path_sgmnt_action_server.publish_feedback(self._feedback)
                self._result.result = self._feedback.status
                self._path_sgmnt_action_server.set_preempted(self._result)
                return
            else:
                self._feedback.status = 'Segmenting is aborted'
                self._path_sgmnt_action_server.publish_feedback(self._feedback)
                self._result.result = self._feedback.status
                self._path_sgmnt_action_server.set_preempted(self._result)
                return
        if self.xStatusPi == self.cannyone_communication_data[self.status_comm_type]['OK']:
            rpy.logdebug(rpy.get_caller_id() + "\n Path ready to serial send Length[%s]",  
                                                len(xaPathSegments))
            xaPathSegments.insert(0, len(xaPathSegments))
            rpy.logdebug(rpy.get_caller_id() + "\n Path ready to serial send Full Segment: %s",  
                                                xaPathSegments)
            self._feedback.plan.poses = [self._feedback.plan.poses[-1]]
            self._feedback.status = 'Sending Segment Length: ' + str(len(xaPathSegments))
            self._path_sgmnt_action_server.publish_feedback(self._feedback)
            # xaPathSegments [xPathSegments_t]
            self.cannyone_serial_sender.msg_sender(self.PathSgmt_comm_topic, xaPathSegments)
            rpy.logdebug(rpy.get_caller_id() + "\n Full Path is sent")
            # xStatusPi [eState_t]: FINISHED
            self.xStatusPi = self.cannyone_communication_data[self.status_comm_type]['FINISHED']
            self.cannyone_serial_sender.msg_sender(self.status_comm_topic, [self.xStatusPi])
            self._result.result = 'Full Path is sent'
            self._path_sgmnt_action_server.set_succeeded(self._result)
        else:
            self._result.result = 'Failed to get a plan.'
            self._path_sgmnt_action_server.set_aborted(self._result)