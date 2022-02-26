import numpy as np
import cv2 as cv

# Video source - can be camera index number given by 'ls /dev/video*
# or can be a video file, e.g. '~/Video.avi'
cap = cv.VideoCapture(0)


while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    # It converts the BGR color space of image to HSV color space
    #hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
     
    # Threshold of blue in HSV space
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
                               param1=200, param2=20,
                               minRadius=1, maxRadius=200)

    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            center = (i[0], i[1])
            # circle center
            cv.circle(result, center, 1, (0, 100, 100), 3)
            # circle outline
            radius = i[2]
            cv.circle(result, center, radius, (255, 0, 255), 3)

    # Display the resulting frame
    cv.imshow('Video Circles', result)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv.destroyAllWindows()
