import pygame
from pygame import locals
from findAngles import findAngles
import serial
from time import time
from math import *


pygame.init()


pygame.joystick.init() # main joystick device system

ser = serial.Serial("/dev/ttyUSB0",115200)

try:
  j = pygame.joystick.Joystick(0) # create a joystick instance
  j.init() # init instance
  print('Enabled joystick: ' + j.get_name())
except pygame.error:
  print('no joystick found.')

lastTime = 0;
epsilon = .1;
lastX = 0;
lastY = 0;

while 1:
  for e in pygame.event.get(): # iterate over event stack
    if e.type == pygame.locals.JOYAXISMOTION: # 7
      x , y, z, w = j.get_axis(0), j.get_axis(1), j.get_axis(2), j.get_axis(3)
      x = -x;
      x, y = x*2, y*2
      if sqrt((lastX - x)**2 + (lastY - y)**2) > epsilon:
          theta = -atan2(y,-x)
          r = sqrt(x**2 + y**2)
          phi = r*.1;
          lastX, lastY = x, y
          try:
              angles = findAngles(x,y,sin(phi)*cos(theta),sin(phi)*sin(theta),cos(phi))
              angles = [ang * 10000 for ang in angles]
              cmd = ("{"+("%.2f,"*6)[:-1]+"}") % tuple(angles)
              #print(cmd)
              ser.write(cmd.encode())
          except:
              pass
