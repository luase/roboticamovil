import numpy as np
import time
import math as m
import sys
import sim as vrep # access all the VREP elements

def v2u(v, omega, r, L):
    ur = v/r + L*omega/(2*r)
    ul = v/r - L*omega/(2*r)
    return ur, ul
  
def ctrl(r, L, Kv, Kh, xd, yd, x, y, d, theta):
    v = Kv*m.sqrt((xd-x)**2+(yd-y)**2)
    thd = m.atan2(yd-y, xd-x)
    omega = Kh*(thd-theta)
    ur, ul = v2u(v, omega, r, L)
    return ur, ul

vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # start a connection
if clientID!=-1:
	print ('Connected to remote API server')
else:
	print('Not connected to remote API server')
	sys.exit("No connection")

# Getting handles for the motors
err, motorL = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', vrep.simx_opmode_blocking)
err, motorR = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', vrep.simx_opmode_blocking)
err, robot =  vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx', vrep.simx_opmode_blocking)
err, goal =  vrep.simxGetObjectHandle(clientID, 'goal', vrep.simx_opmode_blocking)

r = 0.5*0.195
L = 0.311
t = time.time()
tf = 10

err, pos = vrep.simxGetObjectPosition(clientID, robot, -1, vrep.simx_opmode_streaming)
err, angle = vrep.simxGetObjectOrientation(clientID, robot, -1, vrep.simx_opmode_streaming)
err, gpos = vrep.simxGetObjectPosition(clientID, goal, -1, vrep.simx_opmode_streaming)

Kh = 1.0
Kv = 0.7

while (time.time()-t) < tf:
    ur, ul = ctrl(r, L, Kv, Kh, (time.time()-t)*0.3, 0, pos[0], pos[1], 0, angle[2])
    err = vrep.simxSetJointTargetVelocity(clientID, motorL, ul, vrep.simx_opmode_streaming)
    err = vrep.simxSetJointTargetVelocity(clientID, motorR, ur, vrep.simx_opmode_streaming)
    err, pos = vrep.simxGetObjectPosition(clientID, robot, -1, vrep.simx_opmode_buffer)
    err, gpos = vrep.simxGetObjectPosition(clientID, goal, -1, vrep.simx_opmode_buffer)
    err, angle = vrep.simxGetObjectOrientation(clientID, robot, -1, vrep.simx_opmode_buffer)
    print('Err x {}, Err y{}'.format(1.5-pos[0], 1.5-pos[1]))
    # time.sleep(0.1)

vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)