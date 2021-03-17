#!/usr/bin/env python3


import cv2
import numpy

# This works with images and already existing videos.
def rescale_image(image, scale=0.4):
    width  = int(image.shape[1] * scale)
    height = int(image.shape[0] * scale)

    dimensions = (width, height)

    return cv2.resize(image, dimensions, interpolation=cv2.INTER_AREA)
        

def main():
    black = numpy.zeros((500,500), dtype="uint8")
    cv2.imshow("Black", black)
    cv2.waitKey(0)

    color = 255
    rectangle = cv2.rectangle(black.copy(), (100,100), (400,400), color, thickness=cv2.FILLED)
    cv2.imshow("Rectangle", rectangle)
    cv2.waitKey(0)

    center = (250, 250)
    radius = 180
    circle = cv2.circle(black.copy(), center, radius, color, thickness=cv2.FILLED)
    cv2.imshow("Circle", circle)
    cv2.waitKey(0)


    # AND
    bitwise_and = cv2.bitwise_and(rectangle, circle)
    cv2.imshow("AND", bitwise_and)
    cv2.waitKey(0)

    # OR
    bitwise_or = cv2.bitwise_or(rectangle, circle)
    cv2.imshow("OR", bitwise_or)
    cv2.waitKey(0)

    # XOR
    bitwise_xor = cv2.bitwise_xor(rectangle, circle)
    cv2.imshow("XOR", bitwise_xor)
    cv2.waitKey(0)

    # Not
    bitwise_not_rectangle = cv2.bitwise_not(rectangle)
    cv2.imshow("NOT (Rectangle)", bitwise_not_rectangle)
    cv2.waitKey(0)

    bitwise_not_circle = cv2.bitwise_not(circle)
    cv2.imshow("NOT (Circle)", bitwise_not_circle)
    cv2.waitKey(0)
    
    
    
if __name__ == "__main__":
    main()
