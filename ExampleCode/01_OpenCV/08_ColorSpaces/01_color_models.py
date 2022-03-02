#!/usr/bin/env python3


import cv2

# This works with images and already existing videos.
def rescale_image(image, scale=0.4):
    width  = int(image.shape[1] * scale)
    height = int(image.shape[0] * scale)

    dimensions = (width, height)

    return cv2.resize(image, dimensions, interpolation=cv2.INTER_AREA)
        

def main():
    image = cv2.imread("../Images/1280px-Color-wallpapers-30653-4019293.jpg")
    scaled_image = rescale_image(image, 0.6)
    cv2.imshow("Color wallpapers (BGR)", scaled_image)
    cv2.waitKey(0)

    # gray scale
    gray = cv2.cvtColor(scaled_image, cv2.COLOR_BGR2GRAY)
    cv2.imshow("Gray", gray)
    cv2.waitKey(0)

    # BGR to HSV
    # HSL (hue, saturation, lightness) and HSV (hue, saturation, value, also
    # known as HSB or hue, saturation, brightness) are alternative
    # representations of the RGB color model, designed in the 1970s by computer
    # graphics researchers to more closely align with the way human vision
    # perceives color-making attributes. In these models, colors of each hue are
    # arranged in a radial slice, around a central axis of neutral colors which
    # ranges from black at the bottom to white at the top.
    #
    # source: https://en.wikipedia.org/wiki/HSL_and_HSV
    hsv = cv2.cvtColor(scaled_image, cv2.COLOR_BGR2HSV)
    cv2.imshow("HSV", hsv)
    cv2.waitKey(0)
    
    # L*a*b
    # The CIELAB color space also referred to as L*a*b* is a color space defined
    # by the International Commission on Illumination (abbreviated CIE) in
    # 1976. (Referring to CIELAB as "Lab" without asterisks should be avoided to
    # prevent confusion with Hunter Lab.) It expresses color as three values: L*
    # for perceptual lightness, and a* and b* for the four unique colors of
    # human vision: red, green, blue, and yellow. CIELAB was intended as a
    # perceptually uniform space, where a given numerical change corresponds to
    # similar perceived change in color. While the LAB space is not truly
    # perceptually uniform, it nevertheless is useful in industry for detecting
    # small differences in color.
    # 
    # source: https://en.wikipedia.org/wiki/CIELAB_color_space
    lab = cv2.cvtColor(scaled_image, cv2.COLOR_BGR2LAB)
    cv2.imshow("L*a*b", lab)
    cv2.waitKey(0)
    

    # So, OpenCV uses BGR and a lot of other software components use RBG.
    # If an BGR isn't converted to RBG before being displayed by RGB software,
    # it might/will look like the following...
    # Well, it's the same as displaying a RGB in a BGR world...
    rgb = cv2.cvtColor(scaled_image, cv2.COLOR_BGR2RGB)
    cv2.imshow("RGB", rgb)
    cv2.waitKey()


    # It's possible to transform BGR to different color spaces.
    # It's possible to transform different color spaces to BGR.
    # Since BGR is the default, all conversions need to pass this format. :-)
    

if __name__ == "__main__":
    main()
