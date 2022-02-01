#! /usr/bin/env python3

from __future__ import print_function
from tracker import *
import cv2 as cv
import argparse
import numpy as np
from math import sqrt
import roslib
import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import tf2_ros
import geometry_msgs.msg
import tf_conversions

Font = cv.FONT_HERSHEY_SIMPLEX



# Distance Approximation
def Distance_Measurement(Actual_Width, Focal_Length, Frame_Width):
    Distance = (Actual_Width * Focal_Length) / Frame_Width
    return int(Distance)

def callback(rgb_data):
    #Create tracker object
    tracker = EuclideanDistTracker()

    # HSV Limits
    Lower_Limit = np.array([0,3, 0])
    Upper_Limit = np.array([1, 180, 255])

    #defined frame center
    center_x = 320.5
    center_y = 240.5

    # Camera focal length
    Focal_Length = 554.387

    # Tomato Width
    Actual_Width = 10

    window_capture_name = 'Video Capture'
    window_detection_name = 'Object Detection'

    cv.namedWindow(window_capture_name)
    cv.namedWindow(window_detection_name)

    try:
        bridge = CvBridge()

        frame = bridge.imgmsg_to_cv2(rgb_data, "rgb8")
        frame = cv.cvtColor(frame, cv.COLOR_RGB2BGR)

        frame2 = bridge.imgmsg_to_cv2(rgb_data, "bgr8")
        frame_HSV = cv.cvtColor(frame2, cv.COLOR_BGR2HSV)
        mask = cv.inRange(frame_HSV, Lower_Limit, Upper_Limit)
        _, mask = cv.threshold(mask, 254, 255, cv.THRESH_BINARY)

        # Contours
        contours, _ = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        detections = []

        for cnt in contours:
            area = cv.contourArea(cnt)
            if area > 400:
                x, y, w, h = cv.boundingRect(cnt)
                detections.append([x, y, w, h])

        # Co-Ordinates
        Co_Ordinates = []

        co_ordinates = tracker.update(detections)
        for coordinate in co_ordinates:
            x, y, w, h, id = coordinate
            cx = (x + x + w) // 2
            cy = (y + y + h) // 2

            Frame_Width = sqrt(w**2 + h**2)
            Distance = Distance_Measurement(Actual_Width, Focal_Length, Frame_Width)

            Co_Ordinates.append([cx, cy, Distance])
            print(Co_Ordinates)

            # TF CODE using Co_Ordinates
              # transforming pixel coordinates to world coordinates
            world_x = (cx - center_x)/Focal_Length*Distance
            world_y = (cy - center_y)/Focal_Length*Distance
            world_z = Distance
            rospy.loginfo(world_x," ",world_y," ",world_z)

            # broadcasting TF for each aruco marker
            br = tf2_ros.TransformBroadcaster()
            t = geometry_msgs.msg.TransformStamped()
            t.header.stamp = rospy.Time.now()
            # t.header.frame_id = "sjcam_link"
            # t.child_frame_id = "aruco"+str(markerID)

                # putting world coordinates coordinates as viewed for sjcam frame
            # t.transform.translation.x = world_z
            # t.transform.translation.y = -world_x
            # t.transform.translation.z = world_y
            #     # not extracting any orientation thus orientation is (0, 0, 0)
            # q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
            # t.transform.rotation.x = q[0]
            # t.transform.rotation.y = q[1]
            # t.transform.rotation.z = q[2]
            # t.transform.rotation.w = q[3]

            # br.sendTransform(t)
        
        # rospy.loginfo(t)



        # Object Tracking
        boxes_ids = tracker.update(detections)
        for box_id in boxes_ids:
            x, y, w, h, id = box_id
            cv.putText(frame, str(id), (x, y - 10), cv.FONT_HERSHEY_PLAIN, 1, (255, 0, 0), 2)
            cv.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)

        # All Frames
        cv.imshow(window_capture_name, frame)
        cv.imshow(window_detection_name, mask)
        cv.waitKey(1)

    except CvBridgeError as e:
        print(e)

def main(args):
    rospy.init_node('aruco_tf', anonymous=True)

    # Subscribing to RGB Camera
    image_sub = rospy.Subscriber("/camera/color/image_raw2", Image, callback)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)