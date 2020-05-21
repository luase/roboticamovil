import astarmod
import math as m
import matplotlib.pyplot as plt
import numpy as np
import os
from scipy.ndimage.morphology import binary_dilation
from skimage.morphology import selem
import time
import vrep
import random

# Funciones
def grid2world(x, y):
    '''
    Función para transformar coordenadas de la matriz de
    ocupación al mundo
    '''
    xw = - 0.1*(x - 50)
    yw = - 0.1*(50 - y)
    return xw, yw

def generarPuntos(mapa, distancia, puntoDeOrigen):
    while True:
        x = random.randint(-distancia, distancia)
        y = random.randint(-distancia, distancia)
        px = puntoDeOrigen[0] + x
        py = puntoDeOrigen[1] + y
        x_en_limite = px <= 100 and px >= 0
        y_en_limite = py <= 100 and py >= 0
        if x_en_limite and y_en_limite and mapa[px][py] == 0 :
            break
    return px, py

def generarRuta(mapa_ensanchado, punto_de_origen, punto_de_llegada, allow_diagonal_movement):
    ruta_tuplas = astarmod.astar(mapa_ensanchado, puntos_de_origen, puntos_de_llegada, allow_diagonal_movement)
    renglones, columnas = astarmod.path2cells(ruta_tuplas)
    return renglones, columnas

# Variables globales
n = 4 #numero de robots
ensanchamiento = 0
# punto_de_origen = (9,9)
puntos_de_origen =[(9,9),(9,90),(90,90),(90,9)]

#punto_de_llegada = (90, 90)

ruta_del_mapa = 'tarea-3/map.txt'
# Cargamos el mapa existente, o en su caso terminamos el programa
if os.path.exists(ruta_del_mapa):
    print('Cargando mapa...')
    matriz_de_ocupacion = np.loadtxt(ruta_del_mapa)
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

#Generamos puntos random de llegada
puntos_de_llegada = []
for i in range(n):
    puntos_de_llegada.append(generarPuntos(mapa,35,puntos_de_origen[i]))

print(puntos_de_origen)
print(puntos_de_llegada)

# Mostrar los resultados usando plt
plt.figure(2)
plt.imshow(mapa_ensanchado)
plt.show()


# Generamos la ruta
#ruta_tuplas = astarmod.astar(mapa_ensanchado, punto_de_origen, punto_de_llegada, allow_diagonal_movement=False)
# rutas_tuplas = []
#
# for i in range(n):
#     rutas_tuplas.append(astarmod.astar(mapa_ensanchado, puntos_de_origen[i], puntos_de_llegada[i], allow_diagonal_movement=True))
#
# print(rutas_tuplas)
# #renglones, columnas = astarmod.path2cells(ruta_tuplas)
# columnas = []
# renglones = []
# for i in range(n):
#    renglon, columna = astarmod.path2cells(rutas_tuplas[i])
#    columnas.append(columna)
#    renglones.append(renglon)
columnas = []
renglones = []
for i in range(n):
    renglon, columna = generarRuta(mapa_ensanchado, puntos_de_origen[i], puntos_de_llegada[i], False)
    columnas.append(columna)
    renglones.append(renglon)

# Mostrar los resultados de la ruta usando plt
mapa_ruta = np.zeros((100, 100))
for j in range(n):
    for i, punto in enumerate(zip(renglones[j], columnas[j])):
        mapa_ruta[renglones[j][i]][columnas[j][i]] = 0.5
        print(punto)
plt.figure(3)
plt.imshow(mapa_ruta+mapa_ensanchado)
plt.show()

#Generar el movimiento del robot
#Establecer conexión con el api
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # start a connection
if clientID!=-1:
	print ('Conectado con el api')
else:
	print('Imposible conectar con el api')
	exit("Sin conexión")
# Getting handles for the motors and robot
robots = []
for i in range(n):
    _ , tem = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx#'+ str(i), vrep.simx_opmode_blocking)
    robots.append(tem)
    # err, sphere = vrep.simxGetObjectHandle(clientID, 'Sphere#', vrep.simx_opmode_blocking)
# Ubicamos el robot en la posición inicial
# punto_de_origen_mundo = grid2world(punto_de_origen[0], punto_de_origen[1])
# vrep.simxSetObjectPosition(clientID, robot, -1,[punto_de_origen_mundo[0], punto_de_origen_mundo[1], 0.1388], vrep.simx_opmode_oneshot)

motores_izquierdo = []
motores_derecho = []
for i in range(n):
    _, tem = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor#' + str(i), vrep.simx_opmode_blocking)
    motores_izquierdo.append(tem)
    _, tem = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor#' + str(i), vrep.simx_opmode_blocking)
    motores_derecho.append(tem)

# Variables para el movimiento del robot
Kv = 0.5
Kh = 1.5
hd = 0
r = 0.1
L = 0.2
errp = 10

indice_robots = []
for i in range(n):
    indice_robots.append(0)

# Movimiento del robot
while True:
    for i in range(n):
        punto = (renglones[i][indice_robots[i]],columnas[i][indice_robots[i]])
        yd, xd = grid2world(punto[0], punto[1])
        ret = vrep.simxSetObjectPosition(clientID, _, -1, [xd, yd, 0.05], vrep.simx_opmode_oneshot)
        ret, carpos = vrep.simxGetObjectPosition(clientID, robots[i], -1, vrep.simx_opmode_blocking)
        ret, carrot = vrep.simxGetObjectOrientation(clientID, robots[i], -1, vrep.simx_opmode_blocking)
        errp = m.sqrt((xd-carpos[0])**2 + (yd-carpos[1])**2)
        angd = m.atan2(yd-carpos[1], xd-carpos[0])
        errh = angd-carrot[2]
        v = Kv*errp
        omega = Kh*errh
        ul = v/r - L*omega/(2*r)
        ur = v/r + L*omega/(2*r)
        _ = vrep.simxSetJointTargetVelocity(clientID, motores_izquierdo[i], ul, vrep.simx_opmode_streaming)
        _ = vrep.simxSetJointTargetVelocity(clientID, motores_derecho[i], ur, vrep.simx_opmode_streaming)
        indice_robots[i] += 1
        if indice_robots[i] == len(renglones[i]):
            punto_de_origen = tuple(zip(renglones[i][-1],columnas[i][-1]))
            punto_de_llegada = generarPuntos(mapa_ensanchado, 35, punto_de_origen)
            renglones[i], columnas[i] = generarRuta(mapa_ensanchado, punto_de_origen, punto_de_llegada, False)
            indice_robots[i] = 0
        time.sleep(0.5)

vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
