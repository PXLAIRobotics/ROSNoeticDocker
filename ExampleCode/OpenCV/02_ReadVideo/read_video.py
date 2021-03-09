#!/usr/bin/env python3

import cv2


def main():
    window_name = "OpenCV video example"
    capture = cv2.VideoCapture("../Videos/Meditation_HDV_1080p25_Mp4___TanuriX_Stock_Footage.mp4")

    while capture.isOpened():
        return_value, frame = capture.read()

        if return_value == True:
            cv2.imshow(window_name, frame)
        else:
            break
        
        if cv2.waitKey(int(1))  & 0xFF == ord('q'):
            break


    capture.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
