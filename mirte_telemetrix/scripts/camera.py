#!/usr/bin/python

import rospy
import cv2 as cv
import numpy as np
from pyzbar.pyzbar import decode

from mirte_msgs.srv import *
from mirte_msgs.msg import color

cap = cv.VideoCapture(0)

# smaller for higher framerate
# cap.set(3,160)
# cap.set(4,120)

ret, img = cap.read()

# get initial width and height
w = img.shape[1]
h = img.shape[0]

rospy.init_node('listener', anonymous=True)

# not only gets qr codes, but basically anything you throw at it
def handle_get_barcode(req):
    ret, img = cap.read()
    gray_img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    barcodes = decode(gray_img)

    # only return one barcode at a time
    if(len(barcodes) > 0):
        return barcodes[0].data.decode("utf-8")
    
    # return empty string when nothing is found
    return ""


def handle_get_virtual_color(req):
    global w
    global h
    
    ret, img = cap.read()

    # mask
    mask_left = np.zeros((h,w), np.uint8)
    mask_right = np.zeros((h,w), np.uint8)

    # draw masks
    cv.circle(mask_left, (5,h-5), 5, (255,255,255), -1)
    cv.circle(mask_right, (w-5,h-5), 5, (255,255,255), -1)

    # get mean colors
    mean_left = cv.mean(img, mask=mask_left)[:-1]
    mean_right = cv.mean(img, mask=mask_right)[:-1]

    # opencv works with (b,g,r), but we want (r,g,b)
    if(req.direction == "left"):
        return color(int(mean_left[2]), int(mean_left[1]), int(mean_left[0]))
    elif(req.direction == "right"):
        return color(int(mean_right[2]), int(mean_right[1]), int(mean_right[0]))
    else:
        return color(0, 0, 0)

def listener():
    rospy.Service('get_virtual_color', get_virtual_color, handle_get_virtual_color)
    rospy.Service('get_barcode', get_barcode, handle_get_barcode)
    rospy.spin()

if __name__ == '__main__':
    listener()
