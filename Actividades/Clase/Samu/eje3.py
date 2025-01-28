import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np


#definir puntos de traslación
sx, sy, sz = 3, 0, 2

# Definir los ángulos de rotación en grados
ang_z = 30  # Rotación alrededor del eje Z
ang_x = 0  # Rotación alrededor del eje X
ang_y = 270   # Rotación alrededor del eje Y

# Convertir los ángulos a radianes
theta_z = np.radians(ang_z)
theta_x = np.radians(ang_x)
theta_y = np.radians(ang_y)

# Definir las matrices de rotación
rot_z = np.array([[np.cos(theta_z), -np.sin(theta_z), 0],
                  [np.sin(theta_z),  np.cos(theta_z), 0],
                  [0,               0,               1]])

rot_x = np.array([[1,  0,               0],
                  [0,  np.cos(theta_x), -np.sin(theta_x)],
                  [0,  np.sin(theta_x),  np.cos(theta_x)]])

rot_y = np.array([[np.cos(theta_y),  0, np.sin(theta_y)],
                  [0,                1, 0],
                  [-np.sin(theta_y), 0, np.cos(theta_y)]])

# Aplicar las rotaciones en el orden correcto: Y -> X -> Z
rot_mat = rot_z @ rot_x @ rot_y

# Redondear la matriz de transformación a 4 decimales
rot_mat = np.round(rot_mat, 5)
print(rot_mat)

transform_m = np.array([
[rot_mat [0,0], rot_mat [0,1], rot_mat [0,2], sx],
[rot_mat [1,0], rot_mat [1,1], rot_mat [1,2], sy],
[rot_mat [2,0], rot_mat [2,1], rot_mat [2,2], sz],
[0,               0,            0,             1]
])


transform_m = np.round(transform_m, 5)
print (transform_m)
