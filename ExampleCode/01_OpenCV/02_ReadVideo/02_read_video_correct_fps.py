#!/usr/bin/env python3

from threading import Thread
import cv2
import time


class ThreadedVideo(object):
    def __init__(self, source=0, fps="25", name=""):
        
        self.capture = cv2.VideoCapture(source)
        # Let the internal buffer store a max of 2 frames.
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 2)
    
        self.fps = fps
        self.fps_ms = int( (1 / fps) * 1000 )
        self.name = name

        self.thread = Thread(target=self.update, args=())
        self.thread.daemon = True
        self.thread.start()
        

    def update(self):
        while True:
            if self.capture.isOpened():
                (self.status, self.frame) = self.capture.read()
            time.sleep( 1 / self.fps )
        

    def show_frame(self):
        if  self.status:
            cv2.imshow(self.name, self.frame)
            cv2.waitKey(self.fps_ms)
            return True
        
        return False
        
def main():
    source = "../Videos/Meditation_HDV_1080p25_Mp4___TanuriX_Stock_Footage.mp4"
    fps = 25
    name = "OpenCV video example"

    threaded_video = ThreadedVideo(source, fps, name)

    while True:
        try:
            if not threaded_video.show_frame():
                break
        except AttributeError:
            pass

if __name__ == "__main__":
    main()
