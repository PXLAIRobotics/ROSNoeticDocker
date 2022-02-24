#!/usr/bin/env python3

import cv2
import numpy

# x < 0  -> move image left
# y < 0  -> move image up
# 0 < x  -> move image right
# 0 < y  -> move image down
def translate(image, x, y):
    translation_matrix = numpy.float32([[1, 0, x], [0, 1, y]])
    dimensions = (image.shape[1], image.shape[0])

    # From https://docs.opencv.org/3.4/d4/d61/tutorial_warp_affine.html:
    # What is an Affine Transformation?
    # A transformation that can be expressed in the form of a matrix
    # multiplication (linear transformation) followed by a vector
    # addition (translation).
    #
    # From the above, we can use an Affine Transformation to express:
    #  - Rotations (linear transformation)
    #  - Translations (vector addition)
    #  - Scale operations (linear transformation)
    return cv2.warpAffine(image, translation_matrix, dimensions)


def rotate(image, angle, rotation_point=None):
    (height, width) = image.shape[:2]

    if rotation_point == None:
        rotation_point = (width//2, height//2)

    scale = 1.0
    rotation_matrix = cv2.getRotationMatrix2D(rotation_point, angle, scale)

    dimensions = (width, height)

    return cv2.warpAffine(image, rotation_matrix, dimensions)


def main():
    image = cv2.imread("../Images/640px-Curiosity_-_Robot_Geologist_and_Chemist_in_One!.jpg")
    cv2.imshow("Curiosity [640x360]", image)
    cv2.waitKey()

    translated = translate(image, 50, 50)
    cv2.imshow("Translated (50,50)", translated)
    cv2.waitKey()
    
    translated = translate(image, -50, -50)
    cv2.imshow("Translated (-50,-50)", translated)
    cv2.waitKey()

    rotated = rotate(image, 36)
    cv2.imshow("Rotated (36)", rotated)
    cv2.waitKey()

    rotated = rotate(rotated, 36)
    cv2.imshow("Rotated (36, after rotate 36)", rotated)
    cv2.waitKey()

    rotated = rotate(image, (36 + 36))
    cv2.imshow("Rotated (36 + 36)", rotated)
    cv2.waitKey()


    rotated = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
    cv2.imshow("cv2 rotate ROTATE_90_CLOCKWISE", rotated)
    cv2.waitKey()

    rotated = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
    cv2.imshow("cv2 rotate ROTATE_90_COUNTERCLOCKWISE", rotated)
    cv2.waitKey()

    rotated = cv2.rotate(image, cv2.ROTATE_180)
    cv2.imshow("cv2 rotate cv2.ROTATE_180", rotated)
    cv2.waitKey()

    
if __name__ == "__main__":
    main()
