import cv2
import cv2.aruco as aruco
import numpy as np
import os
from Motor import *
from picamera import PiCamera
from picamera.array import PiRGBArray
from time import sleep

PWM = Motor()
def findArucoMarkers(img, markerSize = 6, totalMarkers=250, draw=True):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #key = getattr(aruco, f'DICT_{markerSize}X{markerSize}_{totalMarkers}')
    key = 10
    arucoDict = aruco.Dictionary_get(key)
    arucoParam = aruco.DetectorParameters_create()
    bboxs, ids, rejected = aruco.detectMarkers(gray, arucoDict, parameters = arucoParam)
    # print(ids)
    if draw:
        aruco.drawDetectedMarkers(img, bboxs) 
    return [bboxs, ids]
cap = cv2.VideoCapture(0)   
middlex = 256
thresh = 60
while True:
    success, img = cap.read()
    arucofound = findArucoMarkers(img)
     # loop through all the markers and augment each one
    if  len(arucofound[0])!=0:
        for bbox, id in zip(arucofound[0], arucofound[1]):
            #print(bbox)
            centerx = (bbox[0][0][0] + bbox[0][1][0])/2
            centery = (bbox[0][0][0] + bbox[0][2][0])/2
            print('center: ',centerx, centery)
            if (centerx<middlex-thresh):
                PWM.setMotorModel(-1600, 600, 1600, 600)
            elif (centerx>middlex+thresh):
                PWM.setMotorModel(1600, 600, -1600, 600)
            else:
                print('centered!')
                PWM.setMotorModel(600, 600, 600, 600)
    else:
        PWM.setMotorModel(0, 0, 0, 0)



    #cv2.imshow('img',img)
    k = cv2.waitKey(30) & 0xff
    if k == 27:
        break
cap.release()
cv2.destroyAllWindows()
