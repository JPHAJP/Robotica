import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Line3DCollection
import numpy as np

# Definir el punto inicial AP en 3D
x1, y1, z1 = 1, 0, 0
P1 = np.array([x1, y1, z1])  # Punto inicial en coordenadas homogéneas

# Definir los ángulos de rotación en grados
ang_x = 60  # Rotación alrededor del eje X
ang_y = 45  # Rotación alrededor del eje Y

# Convertir los ángulos a radianes
theta_x = np.radians(ang_x)
theta_y = np.radians(ang_y)

# Definir las matrices de rotación
rot_x = np.array([[1,          0,             0         ],
                  [0,  np.cos(theta_x), -np.sin(theta_x)],
                  [0,  np.sin(theta_x),  np.cos(theta_x)]])

rot_y = np.array([[np.cos(theta_y),  0, np.sin(theta_y)],
                  [0,                1,         0      ],
                  [-np.sin(theta_y), 0, np.cos(theta_y)]])

# Aplicar las rotaciones en el orden correcto: X -> Y
rot_mat = rot_x @ rot_y

print("Matriz de Rotacion: ")
print(rot_mat)

# Rotar Punto
rotated_P1 = rot_mat @ P1

print("P1 Rotado: ")
print(rotated_P1)
