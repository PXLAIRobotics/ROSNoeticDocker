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
    image = cv2.imread("../Images/pillars_of_creation.jpg")
    scaled_image = rescale_image(image, 0.7)
    cv2.imshow("[NASA] Pillars of creation", scaled_image)
    cv2.waitKey(0)

    # Averaging blur
    kernel   = (7, 7)    
    average = cv2.blur(scaled_image, kernel)
    cv2.imshow("Average", average)
    cv2.waitKey()

    # Gaussian blur
    sigmax = 0 # std deviation
    gaussian = cv2.GaussianBlur(scaled_image, kernel, sigmax)
    cv2.imshow("Gaussian", gaussian)
    cv2.waitKey()    

    # Median blur
    kernel_size = 7
    median = cv2.medianBlur(scaled_image, kernel_size)
    cv2.imshow("Median", gaussian)
    cv2.waitKey()

    # Bilateral blur
    # Most effective, it retains the edges!
    diameter = 7
    sigmaColor = 30
    sigmaSpace = 30
    bilateral = cv2.bilateralFilter(scaled_image, diameter, sigmaColor, sigmaSpace)
    cv2.imshow("Bilateral", bilateral)
    cv2.waitKey()

    
if __name__ == "__main__":
    main()
