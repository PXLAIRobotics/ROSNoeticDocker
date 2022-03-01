#!/usr/bin/env python3

import dearpygui.dearpygui as dpg
import array
import numpy as np

from threading import Lock
from threading import Thread

import cv2


class DisplayImageAsTextureThreaded:
    def __init__(self):
        # These are standard DeadPyGui statements to create an env. incl. a window.
        dpg.create_context()
        dpg.create_viewport(title="OpenCV in DearPygui with threading", width=800, height=600)
        dpg.setup_dearpygui()

        self.cam = cv2.VideoCapture(0) # The standard camera should be 0.
                              # If there are multiple cameras,
                              # change this parameter to the correct value.

        # We need the dimensions and fps.
        cam_frame_width  = self.cam.get(cv2.CAP_PROP_FRAME_WIDTH)
        cam_frame_height = self.cam.get(cv2.CAP_PROP_FRAME_HEIGHT)
        cam_fps          = self.cam.get(cv2.CAP_PROP_FPS)

        # Let's read one frame and make a texture to display on start up.
        status, frame = self.cam.read()

        # TODO: Clean up: don't do an exit in a constructor!
        if not status:
            print("Couldn't read an image from the webcam, stopping here!")
            exit(-1)

        # The needed data manipulation statements
        data = np.flip(frame, 2)
        data = data.ravel()
        data = np.asfarray(data, dtype="f")
        self.texture_data = np.true_divide(data, 255.0) 

        with dpg.texture_registry(show=True): 
            dpg.add_raw_texture(cam_frame_width, cam_frame_height, self.texture_data, format=dpg.mvFormat_Float_rgb, tag="texture_tag", label="Webcam")

        with dpg.window(label="OpenCV image"):
            dpg.add_image("texture_tag")

        dpg.show_viewport()
        dpg.show_metrics() # We want to see some metrics!


    def start_reading_and_displaying(self):
        # Let's start a separate thread to get frames from the Webcam.

        # We need a lock to share the texture_data among the threads.
        self.texture_data_lock = Lock()

        # We also need a lock to share the webcam among the threads.
        # Yes, i's possible to implement this example with one lock,
        # but it could/will negatively influence the performance!
        self.cam_lock = Lock()

        webcam_thread = Thread(target=self.read_image_from_cam)
        webcam_thread.start()
        
        # Let's replace the render loop to add a fresh frame at each redraw
        while dpg.is_dearpygui_running():
            # This will run every frame!
            # So, keep this as fast as possible!
            
            self.texture_data_lock.acquire()
            try:
                dpg.set_value("texture_tag", self.texture_data)
            finally:
                self.texture_data_lock.release()
                          
            dpg.render_dearpygui_frame()
            
            
    def read_image_from_cam(self):
        while True:

            frame = None
            self.cam_lock.acquire()
            try:
                ok, frame = self.cam.read() 

                if not ok:
                    break
            finally:
                self.cam_lock.release()

            if frame is not None:
                # Our data manipulations, as described above.
                data = np.flip(frame, 2)
                data = data.ravel() 
                data = np.asfarray(data, dtype="f")
            
                texture_data = np.true_divide(data, 255.0)

                self.texture_data_lock.acquire()
                try:
                    self.texture_data = texture_data
                finally:
                    self.texture_data_lock.release()
                
            
    def cleanup(self):
        self.cam_lock.acquire()
        try:
            if self.cam.isOpened():
                self.cam.release()
        finally:
            self.cam_lock.release()

        dpg.destroy_context()

        
if __name__ == '__main__':
    app = DisplayImageAsTextureThreaded()
    app.start_reading_and_displaying()
    app.cleanup()
