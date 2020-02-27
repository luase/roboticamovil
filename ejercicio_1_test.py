import numpy as np
import time
import math as m
import sys
import sim

# Establishing communication
sim.simxFinish(-1) # just in case, close all opened connections
clientID = sim.simxStart('127.0.0.1',19999,True,True,5000,5) # start a connection
if clientID!=-1:
	print ('Connected to remote API server')
else:
	print('Not connected to remote API server')
	sys.exit("No connection")

returnCode = sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot)


# Getting handles
err, motorL = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_blocking)
err, motorR = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_blocking)
err, robot = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx', sim.simx_opmode_blocking)
err, cubo = sim.simxGetObjectHandle(clientID, 'Cuboid', sim.simx_opmode_blocking)

#Control functions and constants
r = 0.195/2
L = 0.331
Kv = 0.8
Kh = 0.8

def v2u(v, omega, r, L):
    ur = 0.5*v + L*omega/(2*r)
    ul = 0.5*v - L*omega/(2*r)
    return ur, ul

def ctrl(r, L, Kv, Kh, xd, yd, x, y, theta):
    v = Kv*m.sqrt((xd-x)**2+(yd-y)**2)
    thd = m.atan2(yd-y, xd-x)
    omega = Kh*(thd-theta)
    ur, ul  = v2u(v, omega, r, L)
    return ur, ul

t = time.time()

# Start the variables
err, pos = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_streaming)
err, ang = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_streaming)
err, pos_cubo = sim.simxGetObjectPosition(clientID, cubo, -1, sim.simx_opmode_streaming)

# We're giving the robot 30 seconds to move
while True:
    err, pos = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_buffer)
    err, pos_cubo = sim.simxGetObjectPosition(clientID, cubo, -1, sim.simx_opmode_buffer)
    err, ang = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_buffer)

    ur, ul = ctrl(r, L, Kv, Kh, pos_cubo[0], pos_cubo[1], pos[0], pos[1], ang[2])
    err = sim.simxSetJointTargetVelocity(clientID, motorR, ur, sim.simx_opmode_streaming)
    err = sim.simxSetJointTargetVelocity(clientID, motorL, ul, sim.simx_opmode_streaming)

    print("Err x {}, Err y {}".format((pos_cubo[0]-pos[0]), (pos_cubo[1]-pos[1])))

    if pos[0] >= 4.5 or pos[0] <= -4.5 and pos[1] >= 4.5 or pos[1] <= -4.5:
        while (time.time()-t) < 2:
            err, pos = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_buffer)
            err, ang = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_buffer)

            ur, ul = ctrl(r,L, Kv, Kh, 0, 0, pos[0], pos[1], ang[2])
            err = sim.simxSetJointTargetVelocity(clientID, motorR, ur, sim.simx_opmode_streaming)
            err = sim.simxSetJointTargetVelocity(clientID, motorL, ul, sim.simx_opmode_streaming)

    time.sleep(0.1)


sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot)
sim.simxFinish(clientID)
