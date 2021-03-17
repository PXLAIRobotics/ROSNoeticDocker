#!/usr/bin/env python3

import cv2
import numpy

def main():
    image = cv2.imread("../Images/640px-Curiosity_-_Robot_Geologist_and_Chemist_in_One!.jpg")
    cv2.imshow("Curiosity [640x360]", image)
    cv2.waitKey()

    # flipCode < 0: flip vertically and horizontally
    # flipCode = 0: flip vertically
    # 0 < flipCode: flip horizontally
    flipped = cv2.flip(image, flipCode=-2)
    cv2.imshow("Flipped -2 (vertically and horizontally)", flipped)
    cv2.waitKey()

    flipped = cv2.flip(image, flipCode=-1)
    cv2.imshow("Flipped -1 (vertically and horizontally)", flipped)
    cv2.waitKey()

    flipped = cv2.flip(image, flipCode=0)
    cv2.imshow("Flipped 0 (vertically)", flipped)
    cv2.waitKey()

    flipped = cv2.flip(image, flipCode=1)
    cv2.imshow("Flipped 1 (horizontally)", flipped)
    cv2.waitKey()

    flipped = cv2.flip(image, flipCode=2)
    cv2.imshow("Flipped 2 (horizontally)", flipped)
    cv2.waitKey()
    
if __name__ == "__main__":
    main()
