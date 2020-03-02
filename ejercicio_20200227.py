import sys
import sim
import math as m
import numpy as np
import time


# Establishing communication
sim.simxFinish(-1) # just in case, close all opened connections
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5) # start a connection
if clientID!=-1:
	print ('Connected to remote API server')
else:
	print('Not connected to remote API server')
	sys.exit("No connection")

sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot)

# Getting handles
err, motorL = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_blocking)
err, motorR = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_blocking)
err, robot = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx', sim.simx_opmode_blocking)

#Control functions and constants
r = 0.195/2
L = 0.331
Kv = 0.8
Kh = 0.8

def v2u(v, omega, r, L):
    ur = 0.5*v + L*omega/(2*r)
    ul = 0.5*v - L*omega/(2*r)
    return ur, ul

def ctrlPID(r, L, Kv, Kh, xd, yd, x, y, theta, d):
    e = m.sqrt((xd-x)**2+(yd-y)**2) - d
    v = Kv*e
    thd = m.atan2(yd-y, xd-x)
    omega = Kh*(thd-theta)
    ur, ul  = v2u(v, omega, r, L)
    return ur, ul

# Start the variables
t = time.time()
err, pos = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_streaming)
err, ang = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_streaming)

while(time.time() - t < 20):
    err, pos = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_buffer)
    err, ang = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_buffer)

sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot)
sim.simxFinish(clientID)
