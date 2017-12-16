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

kp = 1.0#.82
kv = 0.9#.79
fdg_x = -1.45 #-.15
fdg_y = .2
posis = []
uz = 100

magic_exp = 1#/2
current_pos = np.array([0, 0]);
threshhold = 0.1
nPoints = 5

class NoTargetException(Exception):
    def __init__(self,msg="No target"):
        Exception.__init__(self,msg)

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

def create_mask(hsv, lower=(10,.5*255,.2*255), upper=(40,1*255,1*255), erode=2, dilate=2):
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

def get_velocities_simple(pts):
    t0 = pts[-1][3]
    xs = []
    ys = []
    zs = []
    ts = []
    g = -386.22
    for pt in pts:
        if pt[0] == pt[0]:
            t = pt[3] - t0
            xs.append(pt[0])
            ys.append(pt[1])
            zs.append(pt[2] - .5*g*t**2)
            ts.append(t)

    xMod = np.polyfit(ts,xs,1,full=True)
    yMod = np.polyfit(ts,ys,1,full=True)
    zMod = np.polyfit(ts,zs,1,full=True)

    return xMod[0][0], yMod[0][0], zMod[0][0]+g*ts[0]



def predict_target_simple(pts):
    if(len(pts) < nPoints):
        raise NoTargetException("No target")
    x = pts[0][0]
    y = pts[0][1]
    z = pts[0][2]
    t = pts[0][3]
    overallDt = t - pts[-1][3]
    if(overallDt > 10.0/30):
        raise NoTargetException("No target")
    g = -386.22
    vx,vy,vz = get_velocities_simple(pts)
    deltaT = (-vz - sqrt(vz**2-4*g*z))/g
    if(deltaT>1 or deltaT<.1):
        print(deltaT)
        raise NoTargetException("No target")
    root = np.array([x + vx*deltaT, y + vy*deltaT, 0])
    vel = np.array([vx,vy,vz+g*deltaT])
    time_root = t+deltaT
    return root, vel, time_root




def generate_model(pts):
    t0 = pts[-1][3]
    xs = []
    ys = []
    zs = []
    ts = []
    for pt in pts:
        if pt[0] == pt[0]:
            xs.append(pt[0])
            ys.append(pt[1])
            zs.append(pt[2])
            ts.append(pt[3]-t0)

    if np.max(ts) - np.min(ts) > 1:
        print("Points to far appart")
        print(np.max(ts) - np.min(ts))
        print(2.0*nPoints/30)
        raise NoTargetException()

    if len(xs) > 2:
        return [np.polyfit(ts,xs,1,full=True), np.polyfit(ts,ys,1,full=True), np.polyfit(ts,zs,2,full=True)]
    else:
        print("Not enough points")
        raise NoTargetException()

def model_metrics(models):
    errors = []
    for model in models:
        stats = model[1]
        try:
            residuals = stats[0]
        except IndexError:
            errors.append(0)
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
    g = abs(2*a/39.3701)
    if abs(g-9.81)>4:
        print("g = ", g)
        raise NoTargetException()
    if(b*b - 4*a*c < 0):
        print("gravity is backwards")
        raise NoTargetException()
    d = np.sqrt(b*b - 4*a*c)
    roots = [(-b + d)/(2*a),(-b - d)/(2*a)]
    return roots

def find_derivative(models, x):
    if(len(models) == 3):
        a = models[0]
        b = models[1]
        return 2*a*x + b
    elif(len(models) == 2):
        return models[0]

def find_velocity(model, t):
    xdt = find_derivative(model[0][0],t)
    ydt = find_derivative(model[1][0],t)
    zdt = find_derivative(model[2][0],t)
    return np.array([xdt,ydt,zdt])


def predict_target(points, error_threshold = 10000):
    if len(points) >= nPoints:
        t0 = points[-1][3]
        models = generate_model(points)
        errors = model_metrics(models)
        if np.max(errors) > error_threshold:
            print("error is ", np.max(errors))
            raise NoTargetException()
        time_root = np.max(find_roots(models[2][0],0))
        root = evaluate_models(models, time_root)
        vel = find_velocity(models, time_root)
        return root, vel, time_root+t0
    else:
        print("way too few points")
        raise NoTargetException()

c1toGlobal = np.array([[-0.546532, -0.00232285, 0.837435, -20.8501],
              [0.0233164, -0.999651, 0.0124441, -0.128546],
              [0.837113, 0.0263271, 0.546396, 0.838027],
              [0., 0., 0., 1.]])
c2toGlobal = np.array([[0.00988099, 0.999441, -0.0319341, -0.253576],
              [-0.523506, 0.0323798, 0.851406, -20.7753],
              [0.851965, 0.00830498, 0.523534, 0.751526],
              [0., 0., 0., 1.]])
fx = 725.902
fy = 706.294
cx = 341.759
cy = 166.322

c1Pos = c1toGlobal.dot([0,0,0,1])[0:3]
c2Pos = c2toGlobal.dot([0,0,0,1])[0:3]

mtx = np.array([[ 736.82026776  ,  0.        ,  320.38304561],
                [   0.        ,  735.59843396,  171.57594123],
                [   0.        ,    0.        ,    1.        ]])
dist = np.array([[ 0.0456937   ,0.85564883, -0.00454031  ,0.01840192 ,-2.39214515]])

def to_spherical_coordinates(x, y):
    y_rad = math.atan((x-cx)/fx)
    x_rad = math.atan((y-cy)/fy)
    return [x_rad,y_rad]

def to_3d_point(p1, p2):
    x1 = p1[0]
    y1 = p1[1]
    x2 = p2[0]
    y2 = p2[1]
    u1 = c1toGlobal.dot([(x1-cx)/fx, (y1-cy)/fy, 1, 0])[0:3]
    u2 = c2toGlobal.dot([(x2-cx)/fx, (y2-cy)/fy, 1, 0])[0:3]
    ray1 = Ray(c1Pos,u1)
    ray2 = Ray(c2Pos,u2)
    return ray1.find_closest_point(ray2)

def write_csv(fp, data):
    for i in range(len(data)):
        fp.write(str(data[i]))
        if i < len(data) - 1:
            fp.write(',')
        else:
            fp.write('\n')

h, w = (360,640)
newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))

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
    cam.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH,640)
    cam.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT,360)
if use_serial:
    ser = setup_serial()

points = []
heights = [ int(cam.get(4)) for cam in cameras]
widths  = [ int(cam.get(3)) for cam in cameras]

testPts=np.mgrid[-3:4,-3:4].T.reshape(-1,2).astype(float)
nCols = 7
for i in range(0,nCols**2,nCols*2):
    testPts[i:i+nCols]=testPts[i:i+nCols][::-1]


index = 0

calibX = 0
calibY = 0
calibZ = 18.0+1.0/16.0;

components = [ (0,0) for cam in cameras ]
if use_csv:
    fp = open('points.csv','w')

next_ball = time.clock()
zeroed = False
last_point = [0,0,0,0]
while True:

    grab_time = time.clock()
    for cam in cameras:
        cam.grab()

    frames = [cam.retrieve()[1] for cam in cameras]

    for i in range(len(frames)):
        frame = cv2.undistort(frames[i], mtx, dist, None, newcameramtx)
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

        components[i] = point

        if not headless:
            plot_contours(frame, cnts)
            cv2.imshow("Frame" + str(i), frame)
            #cv2.imshow("Mask" + str(i), mask)

    zero = False
    if time.clock() - next_ball > 1 and not zeroed:
        zero = True
        zeroed = True
    if (components[0][2] != 0 and components[1][2] != 0) or zero:
        point = to_3d_point(components[0],components[1])
        if use_csv:
            write_csv(fp, [point[0],point[1],point[2],(components[0][2]+components[1][2])/2.0])

        point = [point[0],point[1],point[2],(components[0][2]+components[1][2])/2.0]
        point[0] += 0.15
        point[1] += 0.15
        target = []
        vel = []
        u = []
        if zero:
            target = [0,0,0]
            u = [0,0,1]
            vel = [0,0,0]
            posis = []
        elif use_prediction:
            try:
                pass
                target,vel,_=predict_target(points)
                print(target)
                x = target[0]
                y = target[1]
                vx = vel[0]
                vy = -vel[1]
                u = np.array([-x*kp-vx*kv, -y*kp-vy*kv, uz])
            except NoTargetException as e:
                target = point
                vel = [0,0,-130]
                x = target[0]
                y = target[1]
                #theta = atan2(y,x)np.sub(current_pos,)
                #phi = -magic_k*(x**2+y**2)**magic_exp
                u = np.array([-x*kp, -y*kp, uz])
        else:
            #if(point[2] < 3):
            #    continue
            target = [point[0],point[1]]
            if(len(points) > 0):
                last_point=points[-1]
                #print(point[3]-last_point[3])
                if(point[3]-last_point[3] > .075):
                    points = [point]
                    #print("bounce")
                    continue
                points.append(point)
                dt = point[3]-points[0][3]
                dx = point[0]-points[0][0]
                dy = point[1]-points[0][1]
                dz = point[2]-points[0][2]
                vx = dx/dt
                vy = dy/dt
                vz = dz/dt
            else:
                points = [point]
                continue
            x = target[0]
            y = target[1]

            r = sqrt(x**2+y**2)

            v = np.array([vx, vy])

            rhat = np.array([x,y])
            rhat = rhat/r
            thetahat = np.array([-y,x])
            thetahat = thetahat/r

            vr = rhat.dot(v)
            vtheta = thetahat.dot(v)
            if r > 0.25:
                ur = -vr*0.5 - 0.25
                utheta = -vtheta*0.5
            else:
                ur = -vr*0.5 - r
                utheta = -vtheta*0.5

            uxy = ur*rhat + utheta*thetahat
            u = np.array([uxy[0]+fdg_x, uxy[1]+fdg_y, uz])
            #u = np.array([-x*kp - vx*kv + fdg_x, -y*kp - vy*kv + fdg_y, uz])
            zeroed = False

        try:
            angles = findAngles(target[0],target[1],u[0],u[1],u[2])
        except (ValueError, RuntimeError) as e:
            continue

        distToTarget = sqrt((current_pos[0]-target[0])**2 + (current_pos[1]-target[1])**2)

        if(use_serial):
            #cmd_msg = "{%.2f,%.2f,%.2f,%.2f,%.2f,%.2f}" % (angles[0],angles[1],angles[2],angles[3],angles[4],angles[5])
            if time.clock() > next_ball:
                #print("Sending")
                next_ball = time.clock()+.05;
                cmd = bitstring.pack('>hhhhhhh',-1,angles[0],angles[1],angles[2],angles[3],angles[4],angles[5])
                ser.write(cmd.tobytes())
                current_pos = np.array([target[0],target[1]])
    if not headless:
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        if key == ord('f'):
            use_prediction = not use_prediction
            print('predicting: ' + str(use_prediction))
        if(key == ord(' ')):
            print(point[0],point[1],point[2])
