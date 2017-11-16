from math import *
import numpy as np
from scipy.optimize import broyden1

p = 0.7
s = 75
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
  
def findAngles(vx, vy, vz, rx, ry):
  return broyden1(lambda x:getFinalPos(x,vx,vy,vz,rx,ry), [0.1,-atan2(ry,rx)])
  
