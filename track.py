# https://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/
import numpy as np
import cv2
import imutils
from collections import deque
orange_lower = (10, .4*255, .6*255)
orange_upper = (30, .8* 255, 1 *255)
pts = deque(maxlen=64)

camera = cv2.VideoCapture(0)
while True:

    (grabber, frame) = camera.read()

    frame = imutils.resize(frame, width=600)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, orange_lower, orange_upper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2. dilate(mask, None, iterations=2)

    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None

    if len(cnts) > 0:
	# find the largest contour in the mask, then use
	# it to compute the minimum enclosing circle and
	# centroid
	c = max(cnts, key=cv2.contourArea)
	((x, y), radius) = cv2.minEnclosingCircle(c)
	M = cv2.moments(c)
	center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        if radius > 10:
            cv2.circle(frame, (int(x), int(y)), int(radius),(0, 255, 255), 2)
	    cv2.circle(frame, center, 5, (0, 0, 255), -1)
    pts.appendleft(center)

    # loop over the set of tracked points
    for i in xrange(1, len(pts)):
	# if either of the tracked points are None, ignore
	# them
	if pts[i - 1] is None or pts[i] is None:
	    continue
 
        # otherwise, compute the thickness of the line and
        # draw the connecting lines
	thickness = int(np.sqrt(64 / float(i + 1)) * 2.5)
	cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)
 
	# show the frame to our screen
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

camera.release()
cv2.destroyAllWindows()
