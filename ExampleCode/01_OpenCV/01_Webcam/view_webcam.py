#!/usr/bin/env python3

import cv2


def show_webcam(mirror=False):
    cam = cv2.VideoCapture(0) # The standard camera should be 0.
                              # If there are multiple cameras,
                              # change this parameter to the correct value.

    while True:
        # Read returns retval, image
        # retval: false if no frames has been grabbed.
        _, img = cam.read()

        if mirror:
            img = cv2.flip(img, 1)

        cv2.imshow('my webcam', img)

        if cv2.waitKey(1) == 27:
            break  # esc to quit

    cv2.destroyAllWindows()


def main():
    show_webcam(mirror=True)


if __name__ == '__main__':
    main()
