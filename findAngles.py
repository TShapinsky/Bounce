from scipy.optimize import brenth
from math import *

def getLen0(x0,x,y,ux,uy,uz):
  sqrt((-3.1934686764551174 + (3.405049132851517*uz)/sqrt(ux**2 + uz**2) + (0.6141989928900345*ux*uy)/(sqrt(ux**2 + uz**2)*sqrt(ux**2 + uy**2 + uz**2)) + x + 0.5*cos(x0))**2 + (1.84375 + (-0.6141989928900345*ux**2 - 0.6141989928900345*uz**2)/(sqrt(ux**2 + uz**2)*sqrt(ux**2 + uy**2 + uz**2)) + y + 0.8660254037844386*cos(x0))**2 + (-12.43 + (3.405049132851517*ux)/sqrt(ux**2 + uz**2) - (0.6141989928900345*uy*uz)/(sqrt(ux**2 + uz**2)*sqrt(ux**2 + uy**2 + uz**2)) + sin(x0))**2)

def getLen1(x1,x,y,ux,uy,uz):
  sqrt((-3.1934686764551174 + (3.405049132851517*uz)/sqrt(ux**2 + uz**2) - (0.6141989928900345*ux*uy)/(sqrt(ux**2 + uz**2)*sqrt(ux**2 + uy**2 + uz**2)) + x + 0.5*cos(x1))**2 + (1.84375 + (-0.6141989928900345*ux**2 - 0.6141989928900345*uz**2)/(sqrt(ux**2 + uz**2)*sqrt(ux**2 + uy**2 + uz**2)) - 1.*y + 0.8660254037844386*cos(x1))**2 + (-12.43 + (3.405049132851517*ux)/sqrt(ux**2 + uz**2) + (0.6141989928900345*uy*uz)/(sqrt(ux**2 + uz**2)*sqrt(ux**2 + uy**2 + uz**2)) + sin(x1))**2)

def getLen2(x2,x,y,ux,uy,uz):
  sqrt((-3.6875 + (3.255958546628605*ux**2 + 3.255958546628605*uz**2)/(sqrt(ux**2 + uz**2)*sqrt(ux**2 + uy**2 + uz**2)) + y)**2 + ((1.17061263560417*uz)/sqrt(ux**2 + uz**2) + (3.255958546628605*ux*uy)/(sqrt(ux**2 + uz**2)*sqrt(ux**2 + uy**2 + uz**2)) - 1.*x + cos(x2))**2 + (-12.43 - (1.17061263560417*ux)/sqrt(ux**2 + uz**2) + (3.255958546628605*uy*uz)/(sqrt(ux**2 + uz**2)*sqrt(ux**2 + uy**2 + uz**2)) + sin(x2))**2)
  
def getLen3(x3,x,y,ux,uy,uz):
  sqrt((3.1934686764551174 - (2.2344364972473456*uz)/sqrt(ux**2 + uz**2) - (2.6417595537385705*ux*uy)/(sqrt(ux**2 + uz**2)*sqrt(ux**2 + uy**2 + uz**2)) + x + 0.5*cos(x3))**2 + (-1.84375 + (2.6417595537385705*ux**2 + 2.6417595537385705*uz**2)/(sqrt(ux**2 + uz**2)*sqrt(ux**2 + uy**2 + uz**2)) + y + 0.8660254037844386*cos(x3))**2 + (-12.43 - (2.2344364972473456*ux)/sqrt(ux**2 + uz**2) + (2.6417595537385705*uy*uz)/(sqrt(ux**2 + uz**2)*sqrt(ux**2 + uy**2 + uz**2)) + sin(x3))**2)

def getLen4(x4,x,y,ux,uy,uz):
  sqrt((1.84375 + (-2.641759553738569*ux**2 - 2.641759553738569*uz**2)/(sqrt(ux**2 + uz**2)*sqrt(ux**2 + uy**2 + uz**2)) + y - 0.8660254037844386*cos(x4))**2 + (3.1934686764551174 - (2.2344364972473474*uz)/sqrt(ux**2 + uz**2) + (2.641759553738569*ux*uy)/(sqrt(ux**2 + uz**2)*sqrt(ux**2 + uy**2 + uz**2)) + x + 0.5*cos(x4))**2 + (-12.43 - (2.2344364972473474*ux)/sqrt(ux**2 + uz**2) - (2.641759553738569*uy*uz)/(sqrt(ux**2 + uz**2)*sqrt(ux**2 + uy**2 + uz**2)) + sin(x4))**2)

def getLen5(x5,x,y,ux,uy,uz):
  sqrt((3.6875 + (-3.2559585466286043*ux**2 - 3.2559585466286043*uz**2)/(sqrt(ux**2 + uz**2)*sqrt(ux**2 + uy**2 + uz**2)) + y)**2 + ((1.1706126356041722*uz)/sqrt(ux**2 + uz**2) - (3.2559585466286043*ux*uy)/(sqrt(ux**2 + uz**2)*sqrt(ux**2 + uy**2 + uz**2)) - 1.*x + cos(x5))**2 + (-12.43 - (1.1706126356041722*ux)/sqrt(ux**2 + uz**2) - (3.2559585466286043*uy*uz)/(sqrt(ux**2 + uz**2)*sqrt(ux**2 + uy**2 + uz**2)) + sin(x5))**2)

funcs = [getLen0, getLen1, getLen2, getLen3, getLen4, getLen5]  

def findAngles(x,y,ux,uy,uz):
  thetas = [0,0,0,0,0,0]
  for i in range(6):
    thetas[i] = brenth(funcs[i], -pi/2, pi/2, disp = True, args =(x,y,ux,uy,uz))
    
  return thetas