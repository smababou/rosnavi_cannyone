#!/usr/bin/env python3

from sys import exit
import rospy as rpy
import rosparam as rparam
import tf2_ros as tf_ros
from tf2_geometry_msgs import do_transform_pose as do_trans
from geometry_msgs.msg import PoseStamped, TransformStamped
from fiducial_msgs.msg import FiducialTransformArray


class OdomCorrector:
    """Odom Corrector

    After the Robot Odometry being calculated using the odom data
    fused with the IMU data.
    Using Marker vision localization, the Odometry Frame is being corrected 
    by calculating the fault deference detected by the static Markers and Camera Farme.

    @detected_markers: receives the detected markers
    @get_marker_global_loc: retrieve the marker location from txt file
    @object_error_filter: filter the Marker to be used in the Odom correction
    @odom_correction: correction matrices are calculated
    @tf_broadcaster: publish the corrected transformation between odom and map
    """

    def __init__(self):
        self.ref_frame_id = rpy.get_param('~ref_frame_id', 'odom')
        self.marker_id = rpy.get_param('~marker_id','fiducial_')
        m_local_file = rpy.get_param('~markers_location', 
                    '/home/ubuntu/catkin_ws/src/rosnavi_cannyone/cannyone_localization/config/markers_location.yaml')
        self.markers_location = rparam.load_file(m_local_file)[0][0]
        markers_topic = rpy.get_param('~markers_topic','fiducial_transforms')
        self.det_markers_list = FiducialTransformArray()
        self.rate = rpy.Rate(10) # 10hz
        self.tfBuffer = tf_ros.Buffer()
        listener = tf_ros.TransformListener(self.tfBuffer)
        self.broadcaster = tf_ros.TransformBroadcaster()
        rpy.wait_for_message(rpy.get_namespace() + markers_topic, FiducialTransformArray)
        rpy.Subscriber(rpy.get_namespace() + markers_topic, FiducialTransformArray, self.detected_markers)
        self.rate.sleep()
        rpy.loginfo('Robot Loco Ready')
        rpy.sleep(0.3)


    def detected_markers(self, data):
        """Detected Markers

        Callback function for the subscribed Markers detection ROS topic 

        Args:
            data (ROS msg): list of Markers [marker_id, position, orientation, detection error]
        """

        self.det_markers_list = data
        if len(data.transforms) == 0:
            rpy.logwarn('No Markers are detected')
        else:
            rpy.loginfo_once('Marker Detected')
            self.odom_correction()

    def get_marker_global_loc(self, filtered_id):
        """Marker Global localization

        Import from the pre-saved Markers locations the Global static
        localization of the filtered Marker with respect to the Map frame.

        Args:
            filtered_id (int): ID number of the best marker

        Returns:
            Dict: Containing Position and Orientation
        """

        det_markers_ids = [m.fiducial_id for m in self.det_markers_list.transforms]
        det_markers_loc = [self.markers_location['m' + str(id)] for id in det_markers_ids]
        try:
            det_marker_loc = self.markers_location['m' + str(filtered_id)]
            return det_marker_loc
        except KeyError as err:
            rpy.logerr('The global location of the detected Marker [ID:%s] is not listed : %s', filtered_id, err)

    def object_error_filter(self):
        """Detected Markers Filter

        The best Marker is filtered from the detected Markers list 
        according to the object error, image error and fiducial area 
        of each detected Marker.
        The least object error Marker will be used in @odom_correction.

        Returns:
            int: The ID number of the best detected Marker
        """

        # Filtering 1
        # for m in self.det_markers_list.transforms:
        #     if (m.fiducial_area > 2400 and m.object_error < 0.1 and m.image_error < 0.2):
        #         return m.fiducial_id

        # Filtering 2
        det_markers_err = [m.object_error for m in self.det_markers_list.transforms if (m.image_error < 0.2 and m.fiducial_area > 2400)]
        try:
            best_marker_id = self.det_markers_list.transforms[det_markers_err.index(min(det_markers_err))].fiducial_id
            return best_marker_id
        except Exception as err:
            rpy.logerr('Empty Markers list: %s',err)

    def odom_correction(self):
        """Odometry Correction

        Retrieve: 
        - Odometry Frame transformation with respect to the best detected Marker
        (ROS Master).
        - The Global location of the best detected Marker with respect to 
        the Map frame (yaml file).

        Calculate the Odometry Correction with respect to the Map frame

        Typical usage example:
        H           Transformation Matrix
        H(ac)       Map frame   -->     Real Marker frame
        H(ab)       Map frame   -->     Odom frame 
        H(bc)       Odom frame  -->     Detected Marker frame

        H(ac) = H(ab) x H(bc)
        H(ab) = H(bc)^-1 x H(ac)

        """

        best_marker_id = self.object_error_filter()

        if best_marker_id:
            best_marker_loc = self.get_marker_global_loc(best_marker_id)
            try:
                trans = self.tfBuffer.lookup_transform(self.marker_id + str(best_marker_id), self.ref_frame_id, 
                                                        rpy.Time(), rpy.Duration(1.0))
                # Transformation between Real Marker Position with respect to Map frame
                marker_loc = TransformStamped()
                # Best Marker Real location to be used
                marker_loc.transform.translation.x= best_marker_loc['Translation'][0]
                marker_loc.transform.translation.y= best_marker_loc['Translation'][1]
                marker_loc.transform.translation.z= best_marker_loc['Translation'][2]
                marker_loc.transform.rotation.x= best_marker_loc['Rotation'][0]
                marker_loc.transform.rotation.y= best_marker_loc['Rotation'][1]
                marker_loc.transform.rotation.z= best_marker_loc['Rotation'][2]
                marker_loc.transform.rotation.w= best_marker_loc['Rotation'][3]
                # Odom Position with respect to the detected the detected Marker
                trans_pose = PoseStamped()
                trans_pose.pose.position.x = round(trans.transform.translation.x, 3)
                trans_pose.pose.position.y = round(trans.transform.translation.y, 3)
                trans_pose.pose.position.z = round(trans.transform.translation.z, 3)
                trans_pose.pose.orientation.x = round(trans.transform.rotation.x, 10)
                trans_pose.pose.orientation.y = round(trans.transform.rotation.y, 10)
                trans_pose.pose.orientation.z = round(trans.transform.rotation.z, 10)
                trans_pose.pose.orientation.w = round(trans.transform.rotation.w, 10)
                correction = do_trans(trans_pose, marker_loc)
                self.tf_broadcaster(correction)
            except (tf_ros.LookupException, tf_ros.ConnectivityException, tf_ros.ExtrapolationException) as err:
                rpy.logerr('lookup_transform: %s',err)
                self.rate.sleep()

    def tf_broadcaster(self, correction):
        """Transformation Frame Broadcaster

        Send the TF correction between Odometry and Map frame to the ROS Master.

        Args:
            correction (ROS msg): Odometry corrected values
        """

        br = TransformStamped()
        br.header.stamp = rpy.Time.now()
        br.header.frame_id = 'map'
        br.child_frame_id = self.ref_frame_id
        br.transform.translation = correction.pose.position
        br.transform.translation.z = 0.0
        br.transform.rotation = correction.pose.orientation
        br.transform.rotation.y = 0.0
        self.broadcaster.sendTransform(br)
        rpy.loginfo_once('TF published:\n%s', br)



if __name__ == '__main__':

    rpy.init_node('cannyone_localization_node', log_level=rpy.DEBUG)
    rpy.loginfo(rpy.get_caller_id() +  '\n CannyOne Localization Node')

    try:
        obj = OdomCorrector()
        rpy.spin()
    except rpy.ROSInterruptException as err:
        rpy.logerr('ROS Interruption: %s',err)
    except rpy.exceptions.ROSException as inf:
        rpy.loginfo(rpy.get_caller_id() +  '\n CannyOne Localization Node is Shutting Down: %s', inf)
        exit()