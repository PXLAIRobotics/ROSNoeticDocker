#!/usr/bin/env python3

import cv2


def show_webcam(mirror=False):
    webcam = cv2.VideoCapture(0)

    # Resizing
    width  = 320
    height = 400
    webcam.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    webcam.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    # Other interesting values:
    # | Value                   | To set     | Information                     |
    # |-------------------------+------------+---------------------------------|
    # | cv2.CAP_PROP_FPS        | Frame rate |                                 |
    # | cv2.CAP_PROP_BRIGHTNESS | Brightness | Only works with a camera/webcam |
    # | cv2.CAP_PROP_CONTRAST   | Contrast   | Only works with a camera/webcam |
    # | cv2.CAP_PROP_SATURATION | Saturation | Only works with a camera/webcam |

    while True:
        _, image = webcam.read()

        if mirror:
            image = cv2.flip(image, 1)

        cv2.imshow('webcam', image)

        if cv2.waitKey(1) == 27:
            break  # esc to quit

    cv2.destroyAllWindows()


def main():
    show_webcam(mirror=True)


if __name__ == '__main__':
    main()
