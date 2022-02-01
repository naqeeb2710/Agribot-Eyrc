#! /usr/bin/env python3

import cv2 as cv
import numpy as np
import roslib
import sys
import rospy
import message_filters
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import tf2_ros
import geometry_msgs.msg
import tf_conversions
from tracker import *
import roslaunch
import message_filters
import tf
from find_object_2d.msg import ObjectsStamped, DetectionInfo
from object_msgs.msg import ObjectPose

Lower_Limit = np.array([0, 3, 0])
Upper_Limit = np.array([1, 180, 255])
tracker = EuclideanDistTracker()

class ObjectPerception:
    '''A class used to subscribe to get object related data from find_object_2d and tf and return it'''

    def __init__(self):
        # subscriber initialization to subscribe to /objectStamped topic
        self.ob_sub = message_filters.Subscriber('/info', DetectionInfo)
        self.image_sub = message_filters.Subscriber('camera/color/image_raw2', Image)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.ob_sub, self.image_sub], 10, 0.1, allow_headerless=False)
        self.bridge = CvBridge()
        self.ts.registerCallback(self.perception_cb)

        # listener initialization used to lookup required transforms 
        self.tf_listener = tf.TransformListener()
    
    def perception_cb(self, detection_info, ros_img):
        '''callback function for ob_subscriber'''

        ids = detection_info.ids
        widths = detection_info.widths
        heights = detection_info.heights
        filePaths = detection_info.filePaths
        inliers = detection_info.inliers
        outliers = detection_info.outliers
        homographies = detection_info.homographies

        ob_data = zip(ids, widths, heights, filePaths, inliers, outliers, homographies)

        self.ob_data = Utils.dict_from_DetectionInfo(ob_data)

        self.ros_img = ros_img

        # unregister subscriber after recieving required data
        self.ob_sub.unregister()
        self.image_sub.unregister()    

    def get_ob_data(self):
        '''function used to return detected object name and transforms w.r.t base_link
            parameters: No parameters
            returns: list of dictionary with detected object name and transform'''

        if not self.ob_data:
            return None 

        else:
            for ob in self.ob_data:

               # get detected object name using ob_id
               ob['name'] = Utils.get_ob_name(ob['filePath'])

               # wait for required to transform to become available
               self.tf_listener.waitForTransform('base_link', 'object_' + str(ob['id']), rospy.Time(0), rospy.Duration(3))
               # lookup the required transform between object and base_link
               (ob['trans'], ob['rot']) = self.tf_listener.lookupTransform('base_link', 'object_' + str(ob['id']), rospy.Time(0))

            return self.ob_data 
