import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Line3DCollection
import numpy as np

# Definir los ángulos de rotación en grados
ang_x = 30  # Rotación alrededor del eje X

# Convertir los ángulos a radianes
theta_x = np.radians(ang_x)

# Definir las matrices de rotación
rot_x_h = np.array([[1,          0,             0       ,   0],
                    [0,  np.cos(theta_x), -np.sin(theta_x), 0],
                    [0,  np.sin(theta_x),  np.cos(theta_x), 0],
                    [0,           0      ,          0     , 1]])


# Definir el vector de traslación en 3D
sx, sy, sz = 5, 10, 0
tras_m = np.array([
    [1, 0, 0, sx],
    [0, 1, 0, sy],
    [0, 0, 1, sz],
    [0, 0, 0,  1]
])  # Matriz de traslación en 3D

# Calcular el punto transformado
Trans_H = tras_m @ rot_x_h

# Imprimir la matriz de transformación homogénea
print("Matriz de Transformación Homogénea:")
print(Trans_H)





"""
# Definir el vector de traslación en 3D
sx, sy, sz = 5, 10, 0
Trans_H = np.array([
    [1,     0,                  0,         sx],
    [0, np.cos(theta_x), -np.sin(theta_x), sy],
    [0, np.sin(theta_x),  np.cos(theta_x), sz],
    [0,     0,                  0,          1]
])  # Matriz de traslación en 3D"""