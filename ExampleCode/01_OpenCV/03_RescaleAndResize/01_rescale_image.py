#!/usr/bin/env python3

# Q: Why resize/rescale?
# A: To prevent computational strain.

import cv2

# This works with images and already existing videos.
def rescale_image(image, scale=0.4):
    width  = int(image.shape[1] * scale)
    height = int(image.shape[0] * scale)

    dimensions = (width, height)

    return cv2.resize(image, dimensions, interpolation=cv2.INTER_AREA)


def main():
    image = cv2.imread("../Images/pillars_of_creation.jpg")

    window_name = "[NASA] Pillars of creation"

    scaled_image = rescale_image(image)

    cv2.imshow(window_name, scaled_image)
    cv2.waitKey(0) # This will wait indefinitely for a key press.

if __name__ == "__main__":
    main()
