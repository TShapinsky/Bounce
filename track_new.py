import numpy as np
import cv2
import math
from collections import deque


def filter_spherical_candidates(cnts, spherical_threshold=.5, radius_threshold=5):
    filtered_cnts = []
    for cnt in cnts:
        ((x, y), radius) = cv2.minEnclosingCircle(cnt)
        area = cv2.contourArea(cnt)
        idea_area = math.pi * np.power(radius,2)
        if idea_area*spherical_threshold < area and radius > radius_threshold:
            filtered_cnts.append(cnt)
    return filtered_cnts

def create_mask(hsv, lower=(10,.45*255,.6*255), upper=(40,1*255,255), erode=2, dilate=2):
    mask = cv2.inRange(hsv, lower, upper)
    mask = cv2.erode(mask, None, iterations=erode)
    mask = cv2.erode(mask, None, iterations=dilate)
    return mask

def find_contours(mask):
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    return cnts

def plot_contours(frame, cnts):
    cv2.drawContours(frame, cnts, -1, (0,255,0), 3)

def plot_points(frame, pts, radius=5, color=(255,0,0)):
    for pt in pts:
        if pt[0] == pt[0]:
            cv2.circle(frame, (int(pt[0]),int(pt[1])), radius, color, -1)

def generate_model(pts):
    xs = []
    ys = []
    for pt in pts:
        if pt[0] == pt[0]:
            xs.append(pt[0])
            ys.append(pt[1])

    if len(xs) > 1:
        return np.polyfit(xs,ys,2)
    else:
        return [float('nan')]

def find_roots(model, y0):
    a = model[0]
    b = model[1]
    c = model[2] - y0
    d = np.sqrt(b*b - 4*a*c)
    roots = [(-b + d)/(2*a),(-b - d)/(2*a)]
    return roots
    
camera = cv2.VideoCapture(0)

pts = deque(maxlen=5)
height = int(camera.get(4))
while True:

    (_, frame) = camera.read()


    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask = create_mask(hsv)

    cnts = find_contours(mask)

    cnts = filter_spherical_candidates(cnts)

    if len(cnts) > 0:
        ball_cnt = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(ball_cnt)

        pts.appendleft((x, y))

        
    plot_points(frame, pts)
    plot_contours(frame, cnts)

    model = generate_model(pts)
    if model[0] == model[0]:
        roots = find_roots(model, height)
        if roots[0] == roots[0]:
            cv2.circle(frame, (int(roots[0]),height), 10, (0,0,255),-1)
            cv2.circle(frame, (int(roots[1]),height), 10, (0,0,255),-1)

    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
