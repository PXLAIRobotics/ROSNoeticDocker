#!/usr/bin/env python3

import cv2
import numpy

def main():
    image = cv2.imread("../Images/Similar-geometric-shapes.png")
    cv2.imshow("Similar geometric shapes", image)
    cv2.waitKey()

    # Create grayscle
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    cv2.imshow("Gray scale", gray)
    cv2.waitKey(0)

    threshold1 = 127
    threshold2 = 255
    canny = cv2.Canny(gray, threshold1, threshold2)
    cv2.imshow("Canny (100,200)", canny)
    cv2.waitKey()

    # cv2.RETR_LIST -> All contours
    # cv2.RETR_TREE -> All hierarchical contours
    # cv2.RETR_EXTERNAL -> Only external
    # Good to know: findContours used to also return an image! But not anymore!
    contours, hierarchies = cv2.findContours(canny, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    print("Number of contours found: " + str(len(contours)))

    # Draw all contours 
    # -1 signifies drawing all contours
    black_image = numpy.zeros( (image.shape[0], image.shape[1], 3), dtype="uint8")
    cv2.drawContours(black_image, contours, -1, (0, 255, 0), 3)
    cv2.imshow("Contours", black_image)
    cv2.waitKey()

    # Blur
    kernel = (5, 5)
    sigmax = 0
    blur = cv2.GaussianBlur(image, kernel, sigmax)
    cv2.imshow("Blur (5,5)", blur)
    cv2.waitKey()

    # Create grayscle
    gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
    cv2.imshow("Blur (5,5) Gray scale", gray)
    cv2.waitKey(0)

    threshold1 = 127
    threshold2 = 255
    canny = cv2.Canny(gray, threshold1, threshold2)
    cv2.imshow("Blur (5,5) Canny (100,200)", canny)
    cv2.waitKey()

    # cv2.RETR_LIST -> All contours
    # cv2.RETR_TREE -> All hierarchical contours
    # cv2.RETR_EXTERNAL -> Only external
    # Good to know: findContours used to also return an image! But not anymore!
    contours, hierarchies = cv2.findContours(canny, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    print("Blur (5,5) Number of contours found: " + str(len(contours)))

    # Draw all contours 
    # -1 signifies drawing all contours
    black_image = numpy.zeros( (image.shape[0], image.shape[1], 3), dtype="uint8")
    cv2.drawContours(black_image, contours, -1, (0,255,0), 3)
    cv2.imshow("Blur (5,5) Contours", black_image)
    cv2.waitKey()


if __name__ == "__main__":
    main()
