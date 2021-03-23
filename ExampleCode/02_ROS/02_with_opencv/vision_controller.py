#!/usr/bin/env python3

import rospy
import time

from sensor_msgs.msg import Image
from threading import Lock

import cv2
from cv_bridge import CvBridge, CvBridgeError

import numpy as np

GUI_UPDATE_PERIOD = 0.10  # Seconds


class SternformostVision:
    
    def __init__(self):
        self.running = True
        self.subVideo   = rospy.Subscriber('/camera/rgb/image_raw', Image, self.callback_image_raw)

        self.bridge = CvBridge()

        self.image = None
        self.imageLock = Lock()

        self.bound_low = np.array([0, 0, 0])
        self.bound_up  = np.array([0, 0, 0])

        self.statusMessage = ''

        self.connected = False

        self.redrawTimer = rospy.Timer(rospy.Duration(GUI_UPDATE_PERIOD), self.callback_redraw)

        self.mask_window = "Mask"
        cv2.namedWindow(self.mask_window, cv2.WINDOW_NORMAL)
        cv2.createTrackbar("Hue lower bound:", self.mask_window, 0, 179, self.callback_trackbars)
        cv2.createTrackbar("Hue upper bound:", self.mask_window, 0, 179, self.callback_trackbars)
        cv2.createTrackbar("Saturation lower bound:", self.mask_window, 0, 255, self.callback_trackbars)
        cv2.createTrackbar("Saturation upper bound:", self.mask_window, 0, 255, self.callback_trackbars)
        cv2.createTrackbar("Value lower bound:", self.mask_window, 0, 255, self.callback_trackbars)
        cv2.createTrackbar("Value upper bound:", self.mask_window, 0, 255, self.callback_trackbars)

    def is_running(self):
        return self.running

    def convert_ros_to_opencv(self, ros_image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
            return cv_image
        except CvBridgeError as error:
            raise Exception("Failed to convert to OpenCV image")

    def callback_redraw(self, event):
        if self.running == True and self.image is not None:
            self.imageLock.acquire()
            try:
                # Convert the captured frame from ROS to OpenCV.
                image_cv = self.convert_ros_to_opencv(self.image)
            finally:
                self.imageLock.release()

            cv2.namedWindow("Image", cv2.WINDOW_NORMAL)
            img = cv2.resize(image_cv,(360,480))
            cv2.imshow("Image", img)


            image_hsv = cv2.cvtColor(image_cv, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(image_hsv, self.bound_low, self.bound_up)
            mask =cv2.resize(mask, (360,480))
            cv2.imshow('Mask', mask)

            key = cv2.waitKey(5)
            if key == 27: # Esc key top stop
                cv2.destroyAllWindows()
                self.running = False

    def callback_trackbars(self, value):
        h_low = cv2.getTrackbarPos('Hue lower bound:', self.mask_window)
        h_up = cv2.getTrackbarPos('Hue upper bound:', self.mask_window)
        s_low = cv2.getTrackbarPos('Saturation lower bound:', self.mask_window)
        s_up = cv2.getTrackbarPos('Saturation upper bound:', self.mask_window)
        v_low = cv2.getTrackbarPos('Value lower bound:', self.mask_window)
        v_up = cv2.getTrackbarPos('Value upper bound:', self.mask_window)

        self.bound_low = np.array([h_low, s_low, v_low], np.uint8)
        self.bound_up = np.array([h_up, s_up, v_up], np.uint8)

    def callback_image_raw(self, data):
        self.imageLock.acquire()
        try:
            self.image = data
        finally:
            self.imageLock.release()


if __name__=='__main__':
    rospy.init_node('sternformost_example_vision')

    display = SternformostVision()

    while display.is_running():
        time.sleep(5)
