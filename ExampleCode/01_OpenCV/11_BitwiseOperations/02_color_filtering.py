#!/usr/bin/env python3

import cv2
import numpy


def main():
    image = cv2.imread("../Images/Similar-geometric-shapes.png")
    cv2.imshow("Similar geometric shapes", image)
    cv2.waitKey()

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    cv2.imshow("HSV", hsv)
    cv2.waitKey(0)

    cv2.namedWindow("Filtered", cv2.WINDOW_AUTOSIZE)

    # Orange  0-22
    # Yellow 22- 38
    # Green 38-75
    # Blue 75-130
    # Violet 130-160
    # Red 160-179
    
    bound_low = numpy.array([0, 0, 0])
    bound_up  = numpy.array([0, 0, 0])
    
    cv2.createTrackbar("Hue lower bound:", "Filtered", 0, 179, callback_trackbars)
    cv2.createTrackbar("Hue upper bound:", "Filtered", 0, 179, callback_trackbars)
    cv2.createTrackbar("Saturation lower bound:", "Filtered", 0, 255, callback_trackbars)
    cv2.createTrackbar("Saturation upper bound:", "Filtered", 0, 255, callback_trackbars)
    cv2.createTrackbar("Value lower bound:", "Filtered", 0, 255, callback_trackbars)
    cv2.createTrackbar("Value upper bound:", "Filtered", 0, 255, callback_trackbars)

    while True:
        h_low = cv2.getTrackbarPos('Hue lower bound:', 'Filtered')
        h_up = cv2.getTrackbarPos('Hue upper bound:', 'Filtered')
        s_low = cv2.getTrackbarPos('Saturation lower bound:', 'Filtered')
        s_up = cv2.getTrackbarPos('Saturation upper bound:', 'Filtered')
        v_low = cv2.getTrackbarPos('Value lower bound:', 'Filtered')
        v_up = cv2.getTrackbarPos('Value upper bound:', 'Filtered')

        bound_low = numpy.array([h_low, s_low, v_low], numpy.uint8)
        bound_up = numpy.array([h_up, s_up, v_up], numpy.uint8)

        mask = cv2.inRange(hsv, bound_low, bound_up)
        filtered = cv2.bitwise_and(image, image, mask=mask)

        cv2.imshow("Filtered", filtered)
        k = cv2.waitKey(1000) & 0xFF # large wait time to remove freezing
        if k == 113 or k == 27:
            break
        

def callback_trackbars(argument):
    pass
    
if __name__ == "__main__":
    main()
