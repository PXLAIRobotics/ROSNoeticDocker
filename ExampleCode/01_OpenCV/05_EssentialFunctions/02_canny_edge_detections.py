#!/usr/bin/env python3

import cv2


# This works with images and already existing videos.
def rescale_image(image, scale=0.4):
    width  = int(image.shape[1] * scale)
    height = int(image.shape[0] * scale)

    dimensions = (width, height)

    return cv2.resize(image, dimensions, interpolation=cv2.INTER_AREA)


def main():
    image = cv2.imread("../Images/Bletchley_Park.jpg")

    window_name = "Bletchley Park"

    scaled_image = rescale_image(image, 0.7)

    cv2.imshow(window_name, scaled_image)
    cv2.waitKey(0)

    # Canny edge detection
    """
    Fun Fact

    Canny Edge Detection is a popular edge detection algorithm. It was developed
    by John F. Canny in 1986. It is a multi-stage algorithm.
    It involves a lot of blurring, ...

    See the source on more info about the parameters. 

    source: https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_canny/py_canny.html
    """
    threshold1 = 100
    threshold2 = 200
    canny = cv2.Canny(scaled_image, threshold1, threshold2)
    
    # Nice to know: Canny uses a Gaussian filter to reduce noise. (5x5 kernel)
    cv2.imshow("Canny (100,200)", canny)
    cv2.waitKey()

    # Canny edge detection on a blurred images
    kernel = (5, 5)
    # Sigmax: Standard deviation of the Gaussian filter. Good values to start
    # with are between 0.6 and 2.4. Smaller filters cause less blurring, and
    # allow detection of small, sharp lines. A larger filter increases
    # processing time and causes more blurring.
    sigmax = 1.5
    blurred = cv2.GaussianBlur(scaled_image, kernel, sigmax)
    cv2.imshow("Blurred (5,5)", blurred)
    cv2.waitKey()
    threshold1 = 100
    threshold2 = 200
    canny_blurred = cv2.Canny(blurred, threshold1, threshold2)
    cv2.imshow("Canny (100,200) on blurred (5,5)", canny_blurred)
    cv2.waitKey()
    
    kernel = (7, 7)
    sigmax = 1.5
    blurred = cv2.GaussianBlur(scaled_image, kernel, sigmax)
    cv2.imshow("Blurred (7,7)", blurred)
    cv2.waitKey()
    threshold1 = 100
    threshold2 = 200
    canny_blurred = cv2.Canny(blurred, threshold1, threshold2)
    cv2.imshow("Canny (100,200) on blurred (7,7)", canny_blurred)
    cv2.waitKey()


if __name__ == "__main__":
    main()
