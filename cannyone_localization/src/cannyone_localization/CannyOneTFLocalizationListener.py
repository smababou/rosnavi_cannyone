#!/usr/bin/env python3

import rospy as rpy
import tf2_ros as tf_ros
from tf_conversions import transformations as angle_trans

M_PI = 3.141592653589793

class CannyOneTFLocalizationListener:

    def __init__(self):
        self.ref_frame_id = rpy.get_param('~ref_frame_id', 'map')
        self.child_frame_id = rpy.get_param('~child_frame_id','base_footprint')
        self.rate = rpy.Rate(10) # 10hz
        self.tfBuffer = tf_ros.Buffer()
        listener = tf_ros.TransformListener(self.tfBuffer)
        self.broadcaster = tf_ros.TransformBroadcaster()
        self.rate.sleep()
        rpy.loginfo('Robot Loco Ready')

    def location_listener_raw(self):

        try:
            trans = self.tfBuffer.lookup_transform(self.ref_frame_id, self.child_frame_id, 
                                                    rpy.Time(), rpy.Duration(1.0))
            return ([trans.transform.translation.x, 
                     trans.transform.translation.y, 
                     trans.transform.rotation.z, 
                     trans.transform.rotation.w])
        except (tf_ros.LookupException, tf_ros.ConnectivityException, tf_ros.ExtrapolationException) as err:
            rpy.logerr(rpy.get_caller_id() + '\n Lookup Transform: %s',err)
            self.rate.sleep()

    def location_listener_filtered(self):
        trans = self.location_listener_raw()
        if hasattr(trans, '__len__'):
            x = trans[0] * 1000
            y = trans[1] * 1000
            theta = angle_trans.euler_from_quaternion([0.0, 
                                                       0.0, 
                                                       trans[2], 
                                                       trans[3]
                                                    ])[2]
            theta %= 2 * M_PI
            return ([int(x), 
                     int(y), 
                     int(theta * 1000)
                    ])
        else:
            rpy.logerr(rpy.get_caller_id() + '\n Can not find Transform') 
            return ('No Transform')