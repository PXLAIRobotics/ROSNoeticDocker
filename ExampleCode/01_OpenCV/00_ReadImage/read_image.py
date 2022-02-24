#!/usr/bin/env python3

import cv2


def main():
    image = cv2.imread("../Images/pillars_of_creation.jpg")

    window_name = "[NASA] Pillars of creation"

    cv2.imshow(window_name, image)
    cv2.waitKey(0) # This will wait indefinitely for a key press.
                   # Non zero: amount of ms that OpenCV will wait.

if __name__ == "__main__":
    main()
