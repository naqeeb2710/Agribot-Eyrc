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

Lower_Limit = np.array([0, 3, 0])
Upper_Limit = np.array([1, 180, 255])
tracker = EuclideanDistTracker()

class follow_person:
        def __init__(self):
            self.bridge = CvBridge()

            image_sub = message_filters.Subscriber("camera/color/image_raw2", Image)
            depth_sub = message_filters.Subscriber("camera/depth/image_raw2", Image)

            self.ts = message_filters.ApproximateTimeSynchronizer([image_sub, depth_sub], 10, 0.5)
            self.ts.registerCallback(self.callback)

        def callback(self, rgb_data, depth_data):
            try:
                image = self.bridge.imgmsg_to_cv2(rgb_data, "bgr8")
                depth_image = self.bridge.imgmsg_to_cv2(depth_data, "32FC1")
            except CvBridgeError as e:
                print (e)

        depth_array = np.array(depth_image, dtype=np.float32)
        cv.normalize(depth_array, depth_array, 0, 1, cv.NORM_MINMAX)

        gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        upper_bodys = self.body_cascade.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=10,
            minSize=(100,100),
            flags=cv.cv.CV_HAAR_SCALE_IMAGE
        )

        for (x, y, w, h) in upper_bodys:
            cv.rectangle(depth_array, (x, y), (x+w, y+h), (0,255,0), 2)
            cv.circle(depth_array, (x+(w/2), h), 3, (171,110,0), 2)

        cv.imshow('Body Recognition', depth_array)
        cv.waitKey(3)

def main(args):
    fp = follow_person()
    rospy.init_node('follow_person', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    cv.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)