
import numpy as np
import cv2
import math
from collections import deque
import time

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
    ts = []
    for pt in pts:
        if pt[0] == pt[0]:
            xs.append(pt[0])
            ys.append(pt[1])
            ts.append(pt[2])

    if len(xs) > 1:
        return [np.polyfit(xs,ys,2), np.polyfit(ts,xs,2), np.polyfit(ts,ys,2)]
    else:
        return [float('nan')]

def evaluate_model(model, x):
    return model[0]*x*x + model[1]*x + model[2]
    
def find_roots(model, y0):
    a = model[0]
    b = model[1]
    c = model[2] - y0
    d = np.sqrt(b*b - 4*a*c)
    roots = [(-b + d)/(2*a),(-b - d)/(2*a)]
    return roots

def find_derivative(models, x):
    a = models[0]
    b = models[1]
    return 2*a*x + b

def find_velocity(model, t):
    xdt = find_derivative(model[1],t)
    ydt = find_derivative(model[2],t)
    return np.array([xdt,ydt])
    
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

        pts.appendleft((x, y, time.clock()))

        
    plot_points(frame, pts)
    plot_contours(frame, cnts)

    model = generate_model(pts)
    if len(model) > 1:
        time_roots = find_roots(model[2], height)
        vel = find_velocity(model, np.max(time_roots))
        print(vel)
        if time_roots[0] == time_roots[0]:
            cv2.circle(frame, (int(evaluate_model(model[1],np.max(time_roots))),height), 10, (0,0,255),-1)

    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
