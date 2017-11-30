from math import *
import numpy as np
from scipy.optimize import broyden1
from findAngles import findAngles
from time import time

p =.9# 0.05
s =1# 157.0
g = 386.2
def getFinalT(vx, vy, vz, phi, theta):
  costh = cos(theta)
  sinth = sin(theta)
  return (2*s + 2*p*s - 2*p*vz + phi**2*vz + p*phi**2*vz - 2*phi*vx*costh - 2*p*phi*vx*costh - 2*phi*vy*sinth - 2*p*phi*vy*sinth)/g

def getX(vx, vy, vz, phi, theta, t, rx):
  costh = cos(theta)
  sinth = sin(theta)
  return rx + (t*(vx - phi*vz*costh - phi**2*costh*sinth*(vy - phi*vz*sinth + phi**2*costh*(vy*costh - vx*sinth)) + phi*costh*(1 + phi**2*costh**2)*(s + p*(s - vz - phi*(vx*costh + vy*sinth)))))/(1 + phi**2*costh**2)

def getY(vx, vy, vz, phi, theta, t, ry):
  costh = cos(theta)
  sinth = sin(theta)
  return ry + t*(vy - (1 + p)*phi*vz*sinth + phi*(phi*vy*costh**2 + (1 + p)*(s - phi*vx*costh)*sinth - p*phi*vy*sinth**2))

def getFinalPos(x, vx, vy, vz, rx, ry):
  phi   = x[0]
  theta = x[1]
  t = getFinalT(vx, vy, vz, phi, theta)
  return np.array([getX(vx,vy,vz,phi,theta,t,rx), getY(vx,vy,vz,phi,theta,t,ry)])

def findTilt(vx, vy, vz, rx, ry):
  return broyden1(lambda x:getFinalPos(x,vx,vy,vz,rx,ry), [0.2,-atan2(ry,rx)], f_tol = 1e-14)

def findNormal(vx, vy, vz, rx, ry):
  x = findTilt(vx,vy,vz,rx,ry)
  phi   = x[0]
  theta = x[1]

  if(theta > 0):
    theta = theta%(2*pi)

  #print("phi =",degrees(phi))
  #print("theta=",degrees(theta))
  ux, uy, uz = cos(theta)*sin(phi), sin(theta)*sin(phi), cos(phi)
  return [ux, uy, uz]


if(__name__ == '__main__'):
  x0 = 1
  y0 = 1
  v0 = -10
  vx0 = 2;
  vy0 = 1;
  tBefore = time()
  u = findNormal(vx0,vy0,v0,x0,y0)
  print("pos =",x0,y0)
  print("u =",u)
  angles = findAngles(x0,y0,u[0],u[1],u[2])
  tTotal = time() - tBefore
  #print(angles)
  #print(tTotal*1000)
