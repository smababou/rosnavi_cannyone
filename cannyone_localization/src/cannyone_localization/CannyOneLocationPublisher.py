#!/usr/bin/env python3

import rospy as rpy
from nav_msgs.msg import Odometry
from math import dist as eDistance
from aruco_mapping.msg import ArucoMarker
from cannyone_navigation.msg import CannyOnexNaviType
from cannyone_communication import SerialService as CanOneSerSrv
from .CannyOneTFLocalizationListener import CannyOneTFLocalizationListener as CanOneTfLocLsnr


class CannyOneLocationPublisher:

    def __init__(self):
        
        self.xNaviLog = None
        self.xStatusPi = None
        self.rate = rpy.Rate(10)
        self.any_visible_marker = False
        self.cannyone_loc_msg = Odometry()
        self.xNaviType = CannyOnexNaviType()
        self.visible_markers_eDistances_list = []
        baudrate = int(rpy.get_param('~baud','115200'))
        port_name = rpy.get_param('~port','/dev/ttyAMA1')
        self.log_comm_type = rpy.get_param('~logcommType','eNaviLog_t')
        self.log_comm_topic = rpy.get_param('~logcommTopic','xNaviLog')
        self.communication_topic = rpy.get_param('~commTopic','xActPos')
        self.status_comm_type = rpy.get_param('~statuscommType','eState_t')
        self.status_comm_topic = rpy.get_param('~statuscommTopic','xStatusPi')
        markers_sub_topic = rpy.get_param('~aruco_markers_topic','aruco_poses')
        loc_pub_topic = rpy.get_param('~cannyone_loc_topic','cannyone_position')
        navitype_sub_topic = rpy.get_param('~cannyone_navitypes_topic','eCMD_t')
        self.eDistance_thershold = float(rpy.get_param('~eDist_thershold','0.900'))
        non_aruco_loc_nvaitype = {'NAVI_first', 'NAVI_end', 'NAVI_CHARGE', 'NAVI_POINTS', 
                                    'NAVI_ORDER_IDLE', 'NAVI_STOP', 'NAVI_RESTART'}
        self.cannyone_loc_msg.header.frame_id = 'odom'
        self.cannyone_loc_msg.child_frame_id = 'base_footprint'
        self.cannyone_loc_msg.pose.covariance = [0.01, 0.0,  0.0,  0.0,  0.0,  0.0,
                                                 0.0,  0.01, 0.0,  0.0,  0.0,  0.0,
                                                 0.0,  0.0,  0.01, 0.0,  0.0,  0.0,
                                                 0.0,  0.0,  0.0,  0.1,  0.0,  0.0,
                                                 0.0,  0.0,  0.0,  0.0,  0.1,  0.0,
                                                 0.0,  0.0,  0.0,  0.0,  0.0,  0.1]
        self.cannyone_location_listed = CanOneTfLocLsnr()
        self.publisher = rpy.Publisher(rpy.get_namespace() + loc_pub_topic, 
                                        Odometry, 
                                        queue_size=10)
        rpy.loginfo(rpy.get_caller_id() + '\n ROS Topic will be published: /%s [Odometry Msg]', 
                                            loc_pub_topic)
        while not rpy.is_shutdown():
            if rpy.has_param('cannyone_communication_data'): 
                rpy.sleep(0.1)
                self.cannyone_communication_data = rpy.get_param('cannyone_communication_data')
                rpy.logdebug(rpy.get_caller_id() + "\n Communication Data Parameters received")
                self.aruco_loc_navitype = set(self.cannyone_communication_data[navitype_sub_topic].keys())
                self.aruco_loc_navitype -= non_aruco_loc_nvaitype
                rpy.logdebug(rpy.get_caller_id() + "\n Acuro Localization NaviTypes: %s", 
                                                    self.aruco_loc_navitype)
                break
            else:
                rpy.logerr_once(rpy.get_caller_id() + "\n Communication Data Parameter are not launched")
                rpy.logwarn_once(rpy.get_caller_id() + "\n Communication Data Parameter retry to launch")
        self.cannyone_serial_sender = CanOneSerSrv(port_name, baudrate)
        rpy.Subscriber(rpy.get_namespace() + navitype_sub_topic, 
                                        CannyOnexNaviType, 
                                        self.navitype_callback)
        rpy.logdebug(rpy.get_caller_id() + '\n Waiting for [%s] topic', markers_sub_topic)
        rpy.wait_for_message(rpy.get_namespace() + markers_sub_topic, ArucoMarker)
        rpy.Subscriber(rpy.get_namespace() + markers_sub_topic, 
                        ArucoMarker, 
                        self.markers_callback)
        rpy.loginfo(rpy.get_caller_id() + '\n CannyOne Localization Publisher/Sender is Running')
        while not rpy.is_shutdown():
            self.location_publisher()
            self.rate.sleep()


    def navitype_callback(self, data):
        self.xNaviType = data
        rpy.loginfo(rpy.get_caller_id() + '\n CannyOne New NaviType [%s]', data.xNaviType)

    def markers_callback(self, data):
        self.any_visible_marker = data.marker_visibile
        self.visible_markers_eDistances_list.clear()
        if data.marker_visibile:
            for marker in data.global_marker_poses:
                self.visible_markers_eDistances_list.append(
                    eDistance([data.global_camera_pose.position.x, data.global_camera_pose.position.y], 
                              [marker.position.x, marker.position.y]
                             )
                                                            )
            rpy.logdebug(rpy.get_caller_id() + '\n Visible Markers eDistances List: %s', 
                                                self.visible_markers_eDistances_list)
        else:
            self.visible_markers_eDistances_list.clear()


    def location_publisher(self):
        # activate if case in order to check the received navi-type firstly
        # if self.xNaviType.xNaviType in self.aruco_loc_navitype:
        # xActPosRaw = [7.005, 3.510, 0.000, 1.000]
        # xActPos = [7005, 3510, 0]
        xActPos = self.cannyone_location_listed.location_listener_filtered()
        if xActPos == 'No Transform' or not self.any_visible_marker:
            if self.xStatusPi != self.cannyone_communication_data[self.status_comm_type]['FAILED']:
                # xNaviLog [eNaviLog_t]: NAVI_ERR_LOCALIZATION
                self.xNaviLog = self.cannyone_communication_data[self.log_comm_type]['NAVI_ERR_LOCALIZATION']
                # self.cannyone_serial_sender.msg_sender(self.log_comm_topic, [self.xNaviLog])
                # xStatusPi [eState_t]: FAILED
                self.xStatusPi = self.cannyone_communication_data[self.status_comm_type]['FAILED']
                # self.cannyone_serial_sender.msg_sender(self.status_comm_topic, [self.xStatusPi])
        else:
            if self.xStatusPi != self.cannyone_communication_data[self.status_comm_type]['RUN']:
                # xNaviLog [eNaviLog_t]: NAVI_OK
                self.xNaviLog = self.cannyone_communication_data[self.log_comm_type]['NAVI_OK']
                # self.cannyone_serial_sender.msg_sender(self.log_comm_topic, [self.xNaviLog])
                # xStatusPi [eState_t]: RUN
                self.xStatusPi = self.cannyone_communication_data[self.status_comm_type]['RUN']
                # self.cannyone_serial_sender.msg_sender(self.status_comm_topic, [self.xStatusPi])
                # If localization is within the boundary of the markers
            if (isinstance(xActPos, list) and min(self.visible_markers_eDistances_list) <= self.eDistance_thershold):
                self.cannyone_serial_sender.msg_sender(self.communication_topic, xActPos)
            xActPosRaw = self.cannyone_location_listed.location_listener_raw()
            if isinstance(xActPosRaw, list):
                self.cannyone_loc_msg.header.stamp = rpy.Time.now()
                self.cannyone_loc_msg.pose.pose.position.x    = xActPosRaw[0]
                self.cannyone_loc_msg.pose.pose.position.y    = xActPosRaw[1]
                self.cannyone_loc_msg.pose.pose.orientation.z = xActPosRaw[2]
                self.cannyone_loc_msg.pose.pose.orientation.w = xActPosRaw[3]
                try:
                    self.publisher.publish(self.cannyone_loc_msg)
                except (rpy.ROSSerializationException) as err:
                    rpy.logerr('Publishing message failed: %s \n %s', rpy.get_caller_id(), err)