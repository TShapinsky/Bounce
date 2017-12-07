import cv2
import numpy as np
import glob

mtx = np.array([[ 736.82026776  ,  0.        ,  320.38304561],
                [   0.        ,  735.59843396,  171.57594123],
                [   0.        ,    0.        ,    1.        ]])
dist = np.array([[ 0.0456937   ,0.85564883, -0.00454031  ,0.01840192 ,-2.39214515]])

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

cam = get_capture_devices()[0]
cam.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH,640);
cam.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT,360);

def draw(img, corners, imgpts):
    corner = tuple(corners[0].ravel())
    img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 5)
    img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5)
    img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 5)
    return img

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
objp = np.zeros((6*5,3), np.float32)
objp[:,:2] = np.mgrid[0:6,0:5].T.reshape(-1,2)
axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)


while True:
    img = cam.retrieve()[1]
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, (6,5),None)
    if ret == True:
        # Find the rotation and translation vectors.
        ret,rvecs, tvecs, inliers = cv2.solvePnP(objp, corners, mtx, dist)
        # project 3D points to image plane
        imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)
        img = draw(img,corners,imgpts)
        cv2.imshow('img',img)
        k = cv2.waitKey(0) & 0xFF
        if k == ord('s'):
            cv2.imwrite(fname[:6]+'.png', img)
cv2.destroyAllWindows()
