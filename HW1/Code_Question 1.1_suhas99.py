# Python program to plot the trajectory of point O on the front wheel of a bicycle

import matplotlib.pyplot as plt
import math

t = 0                # time
dt = 0.001           # time interval - 1 millisecond
x = 0                # x coordinate - initial value
y = 0                # y coordinate - initial value
theta = math.pi/2    # angle between x axis and cycle frame (pose) - initial value
w = 10               # angular velocity of rear wheel in rad/sec
r = 0.25             # radius of the wheel in m
l = 1.5              # distance between front and rear wheel
T = 10               # total time considered
lx = []     
ly = []              # list initialization
la = []

while t <= T :       # loop for getting plot points
    
    alpha = (0.5) * math.sin(math.pi * t)     #steering angle
    v = ( w * r ) / math.cos(alpha)
    theta = theta + ((( v * math.sin(alpha))/l) * dt)
    x = x + ((( v * math.cos(theta - alpha))) * dt)
    y = y + ((( v * math.sin(theta - alpha))) * dt)
    
    lx.append(x)
    ly.append(y)
    
    t = t + dt
    
plt.plot(lx,ly)
plt.xlabel('X Co-ordinate')
plt.ylabel('Y Co-ordinate')
plt.title('2D trajectory of point O on the front wheel of the bicycle',fontsize=10)
plt.grid(True)
plt.show()