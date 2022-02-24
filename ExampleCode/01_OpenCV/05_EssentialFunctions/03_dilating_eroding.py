#!/usr/bin/env python3


import cv2


# This works with images and already existing videos.
def rescale_image(image, scale=0.4):
    width  = int(image.shape[1] * scale)
    height = int(image.shape[0] * scale)

    dimensions = (width, height)

    return cv2.resize(image, dimensions, interpolation=cv2.INTER_AREA)
        

def main():
    scaled_image = rescale_image(cv2.imread("../Images/Bletchley_Park.jpg"), 0.7)
    
    # Canny edge detection on a blurred image
    kernel = (9, 9)
    sigmax = 0
    blurred = cv2.GaussianBlur(scaled_image, kernel, sigmax)
    
    threshold1 = 100
    threshold2 = 200
    canny_blurred = cv2.Canny(blurred, threshold1, threshold2)
    cv2.imshow("Canny (100,200) on blurred (9,9)", canny_blurred)
    cv2.waitKey()


    # dilating
    kernel = (3, 3)
    dilated = cv2.dilate(canny_blurred, kernel, iterations=3)
    cv2.imshow("Dilated", dilated)
    cv2.waitKey()

    # erode
    kernel = (3, 3)
    eroded = cv2.erode(dilated, kernel, iterations=3)
    cv2.imshow("Eroded", eroded)
    cv2.waitKey()
    

if __name__ == "__main__":
    main()
