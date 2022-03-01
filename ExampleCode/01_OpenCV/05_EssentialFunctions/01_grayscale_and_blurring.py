#!/usr/bin/env python3

import cv2


# This works with images and already existing videos.
def rescale_image(image, scale=0.4):
    width  = int(image.shape[1] * scale)
    height = int(image.shape[0] * scale)

    dimensions = (width, height)

    return cv2.resize(image, dimensions, interpolation=cv2.INTER_AREA)
        

def main():
    image = cv2.imread("../Images/pillars_of_creation.jpg")

    window_name = "[NASA] Pillars of creation (scaled)"

    scaled_image = rescale_image(image, 0.7)
    
    cv2.imshow(window_name, scaled_image)
    cv2.waitKey(0)

    # Create grayscle
    gray = cv2.cvtColor(scaled_image, cv2.COLOR_BGR2GRAY)
    cv2.imshow("Gray scale", gray)
    cv2.waitKey(0)

    
    # Removing noise by using bluring (a.k.a. smoothing.) 
    kernel = (5,5) # This is de kernal size, which needs to be an odd number
                   # It's the window size which OpenCV will use to calculate the blur.
    sigmax = 0 # sigmax - standard deviation in X direction; if 0, calculated from kernel size
    blur = cv2.GaussianBlur(scaled_image, kernel, sigmax)
    cv2.imshow("Blur (5,5)", blur)
    cv2.waitKey()
    
    # Blur some more
    kernel = (13,13)
    blur2 = cv2.GaussianBlur(scaled_image, kernel, sigmax)
    cv2.imshow("Blur (13,13)", blur2)
    cv2.waitKey()

    # Blur even more
    kernel = (21,21)
    blur3 = cv2.GaussianBlur(scaled_image, kernel, sigmax)
    cv2.imshow("Blur (21,21)", blur3)
    cv2.waitKey()


if __name__ == "__main__":
    main()
