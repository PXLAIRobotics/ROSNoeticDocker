#!/usr/bin/env python3

import cv2
import numpy


def main():
    # Create a 400x400 image with a depth of 3.
    image = numpy.zeros( (400,400,3), dtype="uint8")
    # Why a dept of 3?
    #    We do want color, now do we?
    #    + OpenCV uses BGR by default.

    """ FUN FACT:
    Why did they choose BGR color space in OpenCV ?

    The reason the early developers at OpenCV chose BGR color format is that
    back then BGR color format was popular among camera manufacturers and
    software providers. E.g. in Windows, when specifying color value using
    COLORREF they use the BGR format 0x00bbggrr.
    
    source: https://learnopencv.com/why-does-opencv-use-bgr-color-format/
    """

    cv2.imshow("image", image)
    cv2.waitKey(0) 

    # Change all pixels to blue
    image[:] = 255, 0, 0  #
    cv2.imshow("image", image)
    cv2.waitKey(0) 

    

    # Change all pixels to grey
    image[:] = 120, 120, 120
    cv2.imshow("image", image)
    cv2.waitKey(0) 

    # Turn it all to black again.
    image[:] = 0, 0, 0
    cv2.imshow("image", image)
    cv2.waitKey(0) 

    # Draw a rectangle on the image
    x1 = 100
    y1 = 100
    x2 = 300
    y2 = 300
    point_one = (x1,y1)
    point_two = (x2,y2)
    color = (123,244,12)
    cv2.rectangle(image, point_one, point_two, color, thickness=3)
    cv2.imshow("image", image)
    cv2.waitKey(0)

    # Filled rectangle
    cv2.rectangle(image, point_one, point_two, color, thickness=cv2.FILLED)
    # Ugly fact: cv2.FILLED is the value -1. But please do not use -1. It's ugly.
    cv2.imshow("image", image)
    cv2.waitKey(0)

    # Draw a cirle
    diameter = 50
    color = (70,70,70)
    cv2.circle(image, (image.shape[1]//2, image.shape[0]//2), diameter, color, thickness=7)
    cv2.imshow("image", image)
    cv2.waitKey(0)


    # Draw a line
    x1 += 160
    y1 += 160
    x2 -= 160
    y2 -= 160
    point_one = (x1,y1)
    point_two = (x2,y2)
    cv2.line(image, point_one, point_two, color, thickness=7)
    cv2.imshow("image", image)
    cv2.waitKey(0)

    # And another line
    point_one = (x1,y2)
    point_two = (x2,y1)
    cv2.line(image, point_one, point_two, color, thickness=7)
    cv2.imshow("image", image)
    cv2.waitKey(0)

    # Write text
    text = "1983 called, it wants its graphics back!"
    text_x = 3
    text_y = y1 + 80
    origin = (text_x, text_y)
    font_face = cv2.FONT_HERSHEY_DUPLEX
    font_scale = 0.60
    color = (123,244,12)
    cv2.putText(image, text, origin, font_face, font_scale, color, thickness=0)
    cv2.imshow("image", image)
    cv2.waitKey(0)


if __name__ == "__main__":
    main()
