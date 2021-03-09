#!/usr/bin/env python3


import cv2
import numpy

# This works with images and already existing videos.
def rescale_image(image, scale=0.4):
    width  = int(image.shape[1] * scale)
    height = int(image.shape[0] * scale)

    dimensions = (width, height)

    return cv2.resize(image, dimensions, interpolation=cv2.INTER_AREA)
        

def main():
    image = cv2.imread("../Images/1280px-Color-wallpapers-30653-4019293.jpg")
    scaled_image = rescale_image(image, 0.7)
    cv2.imshow("Color wallpapers", scaled_image)
    cv2.waitKey(0)

    blue, green, red = cv2.split(scaled_image)
    cv2.imshow("Blue", blue)
    cv2.waitKey(0)

    cv2.imshow("Green", green)
    cv2.waitKey(0)

    cv2.imshow("Red", red)
    cv2.waitKey(0)

    # OW, they are all gray! Let's fix this!

    black = numpy.zeros(scaled_image.shape[:2], dtype="uint8")
    cv2.imshow("Black", black)
    cv2.waitKey(0)

    blue_correct_channel = cv2.merge([blue, black, black])
    cv2.imshow("Blue in its correct channel", blue_correct_channel)
    cv2.waitKey(0)
    
    green_correct_channel = cv2.merge([black, green, black])
    cv2.imshow("Green in its correct channel", green_correct_channel)
    cv2.waitKey(0)
    
    red_correct_channel = cv2.merge([black, black, red])
    cv2.imshow("Red in its correct channel", red_correct_channel)
    cv2.waitKey(0)
    

if __name__ == "__main__":
    main()
