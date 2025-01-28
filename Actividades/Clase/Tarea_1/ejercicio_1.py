import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

def rotacion_y(angulo_grados):
    angulo_rad = np.radians(angulo_grados)
    R_y = np.array([
        [np.cos(angulo_rad), 0, np.sin(angulo_rad)],
        [0, 1, 0],
        [-np.sin(angulo_rad), 0, np.cos(angulo_rad)]
    ])
    return R_y

def rotacion_x(angulo_grados):
    angulo_rad = np.radians(angulo_grados)
    R_x = np.array([
        [1, 0, 0],
        [0, np.cos(angulo_rad), -np.sin(angulo_rad)],
        [0, np.sin(angulo_rad), np.cos(angulo_rad)]
    ])
    return R_x

# Ángulos de rotación
angulo_y = 45  # grados
angulo_x = 60  # grados

# Matrices de rotación individuales
R_y = rotacion_y(angulo_y)
R_x = rotacion_x(angulo_x)

# Matriz de rotación compuesta (primero R_y, luego R_x)
R_total = R_x @ R_y

# Imprimir resultados
print("Matriz de rotación alrededor de Y:")
print(R_y)
print("\nMatriz de rotación alrededor de X:")
print(R_x)
print("\nMatriz de rotación total (primero Y, luego X):")
print(R_total)