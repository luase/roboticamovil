import astarmod
import math as m
import matplotlib.pyplot as plt
import numpy as np
import os
from scipy.ndimage.morphology import binary_dilation
from skimage.morphology import selem
import time
import random
import sim as vrep


#Función para transformar coordenadas de la matriz de  ocupación al mundo

def worldgrid(x, y):
    xw = 0.1*(50 - x)
    yw = - 0.1*(50 - y)
    return xw, yw


#Get existing map 
    
if os.path.exists('zmap.txt'):
    print('Loading map...')
    occgrid = np.loadtxt('zmap.txt')
    tocc = 1.0*(occgrid > 0.5)
    occgrid[occgrid> 0.5] = 0
    print("Map loaded")
else:
    print('Map not found')
    exit()

plt.figure(1)
plt.imshow(tocc+occgrid)
plt.show()

# Object Dilation

dilation = 4.5
zmap = np.uint8(tocc > 0.5)
disk = selem.disk(dilation)
zmapd = binary_dilation(input=zmap, structure=disk)
plt.figure(2)
plt.imshow(zmapd)
plt.show()

# Get the rute

origen = (8,8)
num1 = random.randint(30, 90)
num2 = random.randint(30, 90)
meta = (num1, num2)

#if zmapd[meta[0]][meta[1]] != 0:
#    continue

ruta_tuplas = astarmod.astar(zmapd, origen, meta, allow_diagonal_movement=True)
rr, cc = astarmod.path2cells(ruta_tuplas)
mapr = np.zeros((100, 100))
for i, punto in enumerate(zip(rr, cc)):
    mapr[rr[i]][cc[i]] = 1.5
plt.figure(3)
plt.imshow(mapr+zmapd)
plt.show()

#Ensanchamiento de la ruta

dilation = 3.0
disk = selem.disk(dilation)
rutad = binary_dilation(input=mapr, structure=disk)
plt.figure(4)
plt.imshow(rutad+zmapd)
plt.show


clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # start a connection
if clientID!=-1:
	print ('Connected to remote API server')
else:
	print('Not connected to remote API server')
	exit("No connection")


# Getting handles for the motors and robot
    
err, robot = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx#', vrep.simx_opmode_blocking)
err, motor_izquierdo = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor#', vrep.simx_opmode_blocking)
err, motor_derecho = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor#', vrep.simx_opmode_blocking)
                                            
# GetRobot to origen

#sim.getObjectsInTree('Pioneer_p3dx#')
# punto_de_origen_mundo = grid2world(punto_de_origen[0], punto_de_origen[1])
# vrep.simxSetObjectPosition(clientID, robot, -1,[punto_de_origen_mundo[0], punto_de_origen_mundo[1], 0.1388], vrep.simx_opmode_oneshot)
#sim.resetDynamicObject
#sim.setObjectPosition('Pioneer_p3dx#',-1,carpos)

vrep.simxFinish(-1) # just in case, close all opened connections

# Robot movement
                                              
Kv = 0.25
Kh = 2.0
hd = 0
r = 0.1
L = 0.2
errp = 10

for punto in zip(rr, cc):
    yd, xd = worldgrid(punto[0], punto[1])
    ret, carpos = vrep.simxGetObjectPosition(clientID, robot, -1, vrep.simx_opmode_blocking)
    ret, carrot = vrep.simxGetObjectOrientation(clientID, robot, -1, vrep.simx_opmode_blocking)
    errp = m.sqrt((xd-carpos[0])**2 + (yd-carpos[1])**2)
    angd = m.atan2(yd-carpos[1], xd-carpos[0])
    errh = angd-carrot[2]
    v = Kv*errp
    omega = Kh*errh
    ul = v/r - L*omega/(2*r)
    ur = v/r + L*omega/(2*r)
    errf = vrep.simxSetJointTargetVelocity(clientID, motor_izquierdo, ul, vrep.simx_opmode_streaming)
    errf = vrep.simxSetJointTargetVelocity(clientID, motor_derecho, ur, vrep.simx_opmode_streaming)
    time.sleep(0.5)

vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)