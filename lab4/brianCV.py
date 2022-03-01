import numpy as np
import cv2 as cv
from Motor import *
from picamera import PiCamera
from picamera.array import PiRGBArray
from time import sleep

print('starting...')
PWM = Motor()
cam = PiCamera()
resx = 1024
resy = 768
originx = resx/2
originy = resy/2
cam.resolution = (resx, resy)
cam.framerate = 32
rawCapture = PiRGBArray(cam)
print('warming up...')
time.sleep(1)

# It converts the BGR color space of image to HSV color space
#hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
     
 

print('processing...')
for frame in cam.capture_continuous(rawCapture, format='bgr', use_video_port=True):
    frame = frame.array

    # Threshold of blue in BGR space
    lower_blue = np.array([100, 0, 0])
    upper_blue = np.array([255, 150, 100])
    
    # preparing the mask to overlay
    mask = cv.inRange(frame, lower_blue, upper_blue)
     
    # The black region in the mask has the value of 0,
    # so when multiplied with original image removes all non-blue regions
    result = cv.bitwise_and(frame, frame, mask = mask)

    # Our operations on the frame come here
    gray = cv.cvtColor(result, cv.COLOR_BGR2GRAY)
    gray = cv.medianBlur(gray, 5)
    rows = gray.shape[0]

    circles = cv.HoughCircles(gray, cv.HOUGH_GRADIENT, 1, rows / 8,
                               param1=100, param2=25,
                               minRadius=1, maxRadius=200)

    if circles is not None:
        circles = np.uint16(np.around(circles))
        centerx = None
        centery = None
        # Assume the largest radius is the radius most likely to be the ball
        likelyradius = None
        for i in circles[0, :]:
            center = (i[0], i[1])
            radius = i[2]

            # Save the info of largest radius circle
            if centerx is not None and radius > likelyradius:
                centerx = i[0]
                centery = i[1]
                likely_radius = radius
            # circle center
            cv.circle(frame, center, 1, (0, 100, 100), 3)
            # circle outline
            cv.circle(frame, center, radius, (255, 0, 255), 3)

        # If most likely ball left, rotate CCW
        if centerx<originx-100:
            print('ball left of me!')
            if likelyradius < 140:
                print('chasing!')
                # depending on ball size we determine distance away
                # go left and forward
                PWM.setMotorModel(750,750,-900,-900)
            else:
                PWM.setMotorModel(750,750,-750,-750)
        elif centerx>originx+100:
            print('ball right of me!')
            if likelyradius < 140:
                print('chasing!')
                # depending on ball size we determine distance away
                # go left and forward
                PWM.setMotorModel(-750,-750,900,900)
            else:
                PWM.setMotorModel(-750,-750,750,-750)
        else:
            print("ball at center")
            if likelyradius < 140:
                print('chasing!')
                PWM.setMotorModel(750,750,750,750)
            else:
                PWM.setMotorModel(0,0,0,0)

    else:
        print("no ball :(")
        PWM.setMotorModel(0,0,0,0)
        

    # Display the resulting frame
    #cv.imshow('Video Circles', frame)
    #PWM.setMotorModel(1000,-1000,1000,-1000)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

    # Clear stream
    rawCapture.truncate(0)
    rawCapture.seek(0)


# When everything done, release the capture
cap.release()
cv.destroyAllWindows()
PWM.setMotorModel(0,0,0,0)
