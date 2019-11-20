#!/usr/bin/python

import rospy
import cv2 as cv
import numpy as np

from zoef_msgs.srv import *
from geometry_msgs.msg import Vector3

cap = cv.VideoCapture(0)
cap.set(3,160)
cap.set(4,120)

ret, img = cap.read()

w = img.shape[1]
h = img.shape[0]

rospy.init_node('listener', anonymous=True)

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

    if(req.direction == "left"):
        v = Vector3(mean_left[0], mean_left[1], mean_left[2])
        return v
    elif(req.direction == "right"):
        v = Vector3(mean_right[0], mean_right[1], mean_right[2])
        return v
    else:
        v = Vector3(0.0, 0.0, 0.0)
        return v

def listener():
    rospy.Service('get_virtual_color', get_virtual_color, handle_get_virtual_color)
    rospy.spin()

if __name__ == '__main__':
    listener()
