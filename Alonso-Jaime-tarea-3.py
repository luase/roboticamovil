import astarmod
import math as m
import matplotlib.pyplot as plt
import numpy as np
import os
from scipy.ndimage.morphology import binary_dilation
from skimage.morphology import selem
import time
import vrep

# Funciones
def grid2world(x, y):
    '''
    Función para transformar coordenadas de la matriz de 
    ocupación al mundo
    '''
    xw = - 0.1*(x - 50)
    yw = - 0.1*(50 - y)
    return xw, yw


# Variables globales
ensanchamiento = 4
punto_de_origen = (9,9)
punto_de_llegada = (90, 90)


# Cargamos el mapa existente, o en su caso terminamos el programa
if os.path.exists('tarea-3/map.txt'):
    print('Cargando mapa...')
    matriz_de_ocupacion = np.loadtxt('tarea-3/map.txt')
    tocc = 1.0*(matriz_de_ocupacion > 0.5)
    matriz_de_ocupacion[matriz_de_ocupacion > 0.5] = 0
    print("Mapa cargado de manera exitosa.")
else:
    print('Mapa no encontrado. Terminando programa.')
    exit()


# Mostrar los resultados usando plt
plt.figure(1)
plt.imshow(tocc+matriz_de_ocupacion)
plt.show()


# Ensanchamiento de los obstaculos
mapa = np.uint8(tocc > 0.5)
disco = selem.disk(ensanchamiento)
mapa_ensanchado = binary_dilation(input=mapa, structure=disco)


# Mostrar los resultados usando plt
plt.figure(2)
plt.imshow(mapa_ensanchado)
plt.show()


# Generamos la ruta
ruta_tuplas = astarmod.astar(mapa_ensanchado, punto_de_origen, punto_de_llegada, allow_diagonal_movement=False)
renglones, columnas = astarmod.path2cells(ruta_tuplas)


# Mostrar los resultados de la ruta usando plt
mapa_ruta = np.zeros((100, 100))
for i, punto in enumerate(zip(renglones, columnas)):
    mapa_ruta[renglones[i]][columnas[i]] = 0.5
    print(punto)
plt.figure(3)
plt.imshow(mapa_ruta+mapa_ensanchado)
plt.show()


# Generar el movimiento del robot
# Establecer conexión con el api
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # start a connection
if clientID!=-1:
	print ('Conectado con el api')
else:
	print('Imposible conectar con el api')
	exit("Sin conexión")
# Getting handles for the motors and robot
err, robot = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx#', vrep.simx_opmode_blocking)
err, sphere = vrep.simxGetObjectHandle(clientID, 'Sphere#', vrep.simx_opmode_blocking)
# Ubicamos el robot en la posición inicial
# punto_de_origen_mundo = grid2world(punto_de_origen[0], punto_de_origen[1])
# vrep.simxSetObjectPosition(clientID, robot, -1,[punto_de_origen_mundo[0], punto_de_origen_mundo[1], 0.1388], vrep.simx_opmode_oneshot)
err, motor_izquierdo = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor#', vrep.simx_opmode_blocking)
err, motor_derecho = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor#', vrep.simx_opmode_blocking)

# Variables para el movimiento del robot
Kv = 0.5
Kh = 1.5
hd = 0
r = 0.1
L = 0.2
errp = 10
# Movimiento del robot
for punto in zip(renglones, columnas):
    yd, xd = grid2world(punto[0], punto[1])
    ret = vrep.simxSetObjectPosition(clientID, sphere, -1, [xd, yd, 0.05], vrep.simx_opmode_oneshot)
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
