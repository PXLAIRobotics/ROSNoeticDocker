#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


import os
print(os.getcwd())   

from yolo_example.msg import DetectedObject
import numpy as np
from ultralytics import YOLO
from pathlib import Path
from time import sleep
import cv2

# Load the YOLO model
model = YOLO('yolov8n-seg.pt')

# Initialize a bridge between ROS Image messages and OpenCV image format
bridge = CvBridge()

def image_callback(data):
    """
    Callback function for the image topic subscriber. 

    Parameters:
    - data (Image): The image message received from the topic.

    This function attempts to convert the ROS Image message into an OpenCV image format (BGR). 
    If successful, further image processing can be added after the conversion. 
    If there's an error in conversion, it logs an error message.
    """
    try:
        # Convert the ROS Image message to an OpenCV image (BGR format)
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        # If an error occurs during the conversion, log an error message
        rospy.logerr(e)
        return
    
    print("Predicting...")

    # Predict with the model
    results = model.predict(cv_image)

    for r in results:
        # Convert the tensors in the results from GPU to CPU, and then to a numpy array.
        boxes = r.boxes.xyxy.cpu().numpy()
        confidences = r.boxes.conf.cpu().numpy()
        class_ids = r.boxes.cls.cpu().numpy()
        
        # Generate a list of labels by mapping the class IDs to their corresponding names.
        labels = [r.names[int(class_id)] for class_id in class_ids]

        for box, label, confidence in zip(boxes, labels, confidences):
            # Create a ROS message with the bounding box, label and confidences
            detected_obj_msg = DetectedObject()
            detected_obj_msg.label = label
            detected_obj_msg.confidence = confidence
            detected_obj_msg.box.x1 = box[0]
            detected_obj_msg.box.y1 = box[1]
            detected_obj_msg.box.x2 = box[2]
            detected_obj_msg.box.y2 = box[3]
            # print(detected_obj_msg)

            # Publish the detected object's information over ROS topic
            detection_pub.publish(detected_obj_msg)

            # Draw the bounding box on the image.
            cv2.rectangle(cv_image, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (0, 255, 0), 2)
            
            # Display the label and confidence score on the image, just above the bounding box.
            cv2.putText(cv_image, f"{label} {confidence:.2f}", (int(box[0]), int(box[1]-10)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)


    # Show the image with bounding boxes and labels
    cv2.imshow('YOLO Detections', cv_image)
    cv2.waitKey(1)  # waits for a key event for a very short period
    
    print("-------------")
    print()

def main():
    global detection_pub
    # Initialize a ROS node named 'yolo_object_detector'
    rospy.init_node('yolo_object_detector', anonymous=True)

    # Subscribe to the "/camera/color/image_raw" topic. When a message of type Image 
    # is received on this topic, the 'image_callback' function is triggered
    rospy.Subscriber("/camera/color/image_raw", Image, image_callback)

    # Define a ROS publisher to publish messages of type DetectedObject on the 
    # "/detected_objects" topic. A queue size of 10 is set, meaning it can buffer 
    # up to 10 messages before starting to drop old ones
    detection_pub = rospy.Publisher("/detected_objects", DetectedObject, queue_size=10)

    # Keep the node running and listening for incoming messages until manually terminated
    rospy.spin()

if __name__ == '__main__':
    main()
