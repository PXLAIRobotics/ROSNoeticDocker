#!/usr/bin/env python3

import dearpygui.dearpygui as dpg
import array
import numpy as np

import cv2

if __name__ == '__main__':
    # These are standard DeadPyGui statements to create an env. incl. a window.
    dpg.create_context()
    dpg.create_viewport(title="OpenCV in DearPygui", width=800, height=600)
    dpg.setup_dearpygui()

    
    # We need to use textures in order to display an (OpenCV) image.
    #
    # DearPyGui had 3 types of textures:
    # - Static
    #      For a single "static" image
    # - Dynamic
    #      if the image gets a lot of updates (render, video, ...)
    #      This type also Does type chacking (incl. converting ints to float)
    # - Raw
    #      Dynamic without the type checking, straight to the GPU,  quiker
    #      than Dynamic.
    #      -> This example will use Raw. Let's push the padel to the metal!
    #         (Or not, thanks to a blocking OpenCV function.)
    

    # Let's create a texture form webcam image
    cam = cv2.VideoCapture(0) # The standard camera should be 0.
                              # If there are multiple cameras,
                              # change this parameter to the correct value.

    # We need the dimensions and fps.
    cam_frame_width  = cam.get(cv2.CAP_PROP_FRAME_WIDTH)
    cam_frame_height = cam.get(cv2.CAP_PROP_FRAME_HEIGHT)
    cam_fps          = cam.get(cv2.CAP_PROP_FPS)

    # Let's read one frame and make a texture to display on start up.
    status, frame = cam.read()

    if not status:
        print("Couldn't read an image from the webcam, stopping here!")
        exit(-1)

    # Numpy can also change the colour space from BGR to RGB.
    # It's also possible to set (cam.set) the colour space of certain cams!
    data = np.flip(frame, 2)

    # A texture is a single row of data, we need to ravel (a.k.a. flatten) the
    # camera data to a 1d structure.
    #
    # Definition of ravel
    #     intransitive verb. 1 : to become unwoven, untwisted, or unwound 
    data = data.ravel()

    # Textures on a GPU are 32bit floats, we need to convert the data.
    data = np.asfarray(data, dtype="f")

    # Also, textures are normalized.
    # 
    # Fact:
    #     Pixel values range from 0 to 256.
    # Fact:
    #     Normalized values used in probability density functions are between
    #     0 and 1.
    # Therefore:
    #     we need to divide all values by 255
    texture_data = np.true_divide(data, 255.0) 

    # Texture need to be add to a registry
    with dpg.texture_registry(show=True): 
        dpg.add_raw_texture(cam_frame_width, cam_frame_height, texture_data, format=dpg.mvFormat_Float_rgb, tag="texture_tag", label="Webcam")

    with dpg.window(label="OpenCV image"):
        dpg.add_image("texture_tag")

    dpg.show_viewport()
    
    # We want to see some metrics!
    dpg.show_metrics() 

    # Let's replace the render loop to add a fresh frame at each redraw
    while dpg.is_dearpygui_running():
        # This will run every frame!

        # Move this in an other thread with locking
        status, frame = cam.read() # This is a blocking function, @ 30 fps!

        # Our data manipulations, as described above.
        data = np.flip(frame, 2)
        data = data.ravel() 
        data = np.asfarray(data, dtype="f") 
        texture_data = np.true_divide(data, 255.0)

        dpg.set_value("texture_tag", texture_data)
                      
        dpg.render_dearpygui_frame()

    # We need to clean up when the program stops.
    cam.release()
    dpg.destroy_context()
