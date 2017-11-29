import numpy as np
import cv2
import math
from collections import deque
import time
import serial
import sys
import bitstring
from findAngles import findAngles
from findNormal import findNormal
from ray import Ray
from math import *

def setup_serial(port="/dev/ttyUSB0",baud=115200):
    return serial.Serial(port,baud)

def get_capture_devices():
    caps = [None]*2
    with open('cameras.dat','r') as f:
        data = f.read().split('\n')
        for cam in data:
            if "6E364060" in cam:
                caps[0] = cv2.VideoCapture(int(cam.split("video")[1]))
            if "1C8A4060" in cam:
                caps[1] = cv2.VideoCapture(int(cam.split("video")[1]))
    return caps

def filter_spherical_candidates(cnts, spherical_threshold=.5, radius_threshold=3):
    filtered_cnts = []
    for cnt in cnts:
        ((x, y), radius) = cv2.minEnclosingCircle(cnt)
        area = cv2.contourArea(cnt)
        idea_area = math.pi * np.power(radius,2)
        if idea_area*spherical_threshold < area and radius > radius_threshold:
            filtered_cnts.append(cnt)
    return filtered_cnts

def create_mask(hsv, lower=(10,.5*255,.2*255), upper=(35,1*255,.8*255), erode=2, dilate=2):
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
    zs = []
    ts = []
    for pt in pts:
        if pt[0] == pt[0]:
            xs.append(pt[0])
            ys.append(pt[1])
            zs.append(pt[2])
            ts.append(pt[3])

    if len(xs) > 1:
        return [np.polyfit(ts,xs,2,full=True), np.polyfit(ts,ys,2,full=True), np.polyfit(ts,zs,2,full=True)]
    else:
        return [float('nan')]

def model_metrics(models):
    errors = []
    for model in models:
        residuals = model[1][0]
        errors.append(np.mean(np.square(residuals)))
    return errors
        
def evaluate_model(model, t):
    res = 0
    for i in range(len(model)):
        res += model[i]*math.pow(t,len(model)-i-1)
    return res

def evaluate_models(models, t):
    res = []
    for model in models:
        res.append(evaluate_model(model[0],t))
    return np.array(res)

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
    xdt = find_derivative(model[0][0],t)
    ydt = find_derivative(model[1][0],t)
    zdt = find_derivative(model[2][0],t)
    return np.array([xdt,ydt,zdt])


def predict_target(points, error_threshold = 0.1):
    if len(points) > 5:
        models = generate_model(points)
        errors = model_metrics(models)
        if np.max(errors) > error_threshold:
            return points[0],[0,0,-10], time.clock() 
        time_root = np.max(find_roots(models[2][0],0))
        root = evaluate_models(models, time_root)
        vel = find_velocity(models, time_root)
        return root, vel, time_root
    else:
        return points[0],[0,0,-10], time.clock()

def to_spherical_coordinates(x, y):
    focal_length = 4.0
    fov = 60.0/180.0*math.pi
    res = 640.0
    px_mm = 2.0*focal_length*math.tan(fov/2)/res
    y_mm = y*px_mm
    x_mm = x*px_mm
    y_rad = math.atan(y_mm/focal_length)
    x_rad = math.atan(x_mm/focal_length)
    return [x_rad,y_rad]

def to_3d_point(p1, p2):
    p1 = to_spherical_coordinates(p1[0],p1[1])
    p2 = to_spherical_coordinates(p2[0],p2[1])

    p1[0] += math.pi/6
    p2[0] += math.pi/6
    ray1 = Ray([20.5,0,.75],[-1,math.tan(p1[1]),math.tan(p1[0])])
    ray2 = Ray([0,20.5,.75],[math.tan(p2[1]),-1,math.tan(p2[0])])
    return ray1.find_closest_point(ray2)

def write_csv(fp, data):
    for i in range(len(data)):
        fp.write(str(data[i]))
        if i < len(data) - 1:
            fp.write(',')
        else:
            fp.write('\n')

use_serial = False
use_prediction = False
use_csv = False
headless = False
if len(sys.argv) > 0:
    for arg in sys.argv:
        if arg == "--serial":
            use_serial = True
        if arg == "--predict":
            use_prediction = True
        if arg == "--csv":
            use_csv = True
        if arg == "--headless":
            headless = True
cameras = get_capture_devices()

for cam in cameras:
    cam.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH,640);
    cam.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT,360);
if use_serial:
    ser = setup_serial()
    ser2 = setup_serial(port = "/dev/ttyACM0")
points = deque(maxlen=5)
heights = [ int(cam.get(4)) for cam in cameras]
widths  = [ int(cam.get(3)) for cam in cameras]
components = [ (0,0) for cam in cameras ]
if use_csv:
    fp = open('points.csv','w')

next_ball = time.clock()
    
while True:

    if time.clock() < next_ball:
        continue
    
    grab_time = time.clock()
    for cam in cameras:
        cam.grab()
    
    frames = [cam.retrieve()[1] for cam in cameras]

    for i in range(len(frames)):
        frame = frames[i]
        height = heights[i]
        width = widths[i]

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask = create_mask(hsv)

        cnts = find_contours(mask)

        cnts = filter_spherical_candidates(cnts)

        point = [width/2,height/2,0]
        if len(cnts) > 0:
            ball_cnt = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(ball_cnt)
            point = [x,y,grab_time]

        point[0] -= width/2
        point[1] -= height/2
        components[i] = point

        if not headless:
            plot_contours(frame, cnts)
            cv2.imshow("Frame" + str(i), frame)

    if components[0][2] != 0 and components[1][2] != 0 and abs(components[0][2] - components[1][2]) < 0.1:
        point = to_3d_point(components[0],components[1])
        if use_csv:
            write_csv(fp, [point[0],point[1],point[2],(components[0][2]+components[1][2])/2.0])
        points.appendleft([point[0],point[1],point[2],(components[0][2]+components[1][2])/2.0])
        target = []
        vel = []
        u = []
        if use_prediction:
            if points[0][3] - points[-1][3] < .5:
                target,vel,contact_time = predict_target(points)
                next_ball = contact_time
                try:
                    theta = -atan2(-target[1],target[0])
                    phi = radians(.5*linalg.norm(vel)*sqrt(target[1]**2 + target[2]**2))
                    u = [cos(theta)*sin(phi),sin(theta)*sin(phi),cos(phi)]
                    #u = findNormal(vel[0], -vel[1], vel[2], target[0], -target[1])
                except:
                    u = [0, 0, 1]
                    print("could not compute target")
        else:
            target = point
            vel = [0,0,-10]
            theta = -atan2(-target[1],target[0])
            phi = radians(.1*sqrt(target[1]**2 + target[2]**2))
            u = [cos(theta)*sin(phi),sin(theta)*sin(phi),cos(phi)]
        try:

            angles = findAngles(target[0],-target[1],u[0],u[1],u[2])
            #angles = [angle*10000 for angle in angles]
            if(use_serial):
                #cmd_msg = "{%.2f,%.2f,%.2f,%.2f,%.2f,%.2f}" % (angles[0],angles[1],angles[2],angles[3],angles[4],angles[5])
                cmd = bitstring.pack('HHHHHH',angles[0],angles[1],angles[2],angles[3],angles[4],angles[5])
                cmd_time = "{%d}" % int(1000000/np.linalg.norm(vel))
                ser.write(cmd.tobytes())
                ser2.write(cmd_time)
        except Exception as e:
            print("could not compute angles")
            print(e)
            pass
    
    if not headless:
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
