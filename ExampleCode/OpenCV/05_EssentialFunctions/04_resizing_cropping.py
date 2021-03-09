#!/usr/bin/env python3


import cv2

# This works with images and already existing videos.
def rescale_image(image, scale=0.4, interpolation=cv2.INTER_AREA):
    width  = int(image.shape[1] * scale)
    height = int(image.shape[0] * scale)

    dimensions = (width, height)

    return cv2.resize(image, dimensions, interpolation)
        

def main():
    image = cv2.imread("../Images/640px-Curiosity_-_Robot_Geologist_and_Chemist_in_One!.jpg")
    cv2.imshow("Curiosity [640x360]", image)
    cv2.waitKey(0)
    
    scaled_image = rescale_image(image, 0.7)
    cv2.imshow("Curiosity @ 0.7", scaled_image)
    cv2.waitKey(0)

    # Enhance!
    scaled_image2 = rescale_image(image, 2, interpolation=cv2.INTER_LINEAR)
    cv2.imshow("Curiosity @ 2 (Linear)", scaled_image2)
    cv2.waitKey(0)

    # Enhance!
    scaled_image3 = rescale_image(image, 2, interpolation=cv2.INTER_CUBIC)
    cv2.imshow("Curiosity @ 2 (Cubic)", scaled_image3)
    cv2.waitKey(0)


    # Cropping
    # Remember images are just arrays.
    x1=155
    y1=65
    x2=530
    y2=340
    cropped = image[y1:y2, x1:x2]
    cv2.imshow("Curiosity Cropped", cropped)
    cv2.waitKey(0)
    
    
    
if __name__ == "__main__":
    main()
