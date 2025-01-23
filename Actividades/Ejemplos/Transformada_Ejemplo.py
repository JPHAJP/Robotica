import numpy as np
import matplotlib.pyplot as plt

# Configurar las listas de puntos
x_points = np.array([2, 2, 0.5, -1, -1, 2])
y_points = np.array([-1, 2, 3, 2, -1, -1])

# Crear una matriz (2 x n) de puntos
points = np.vstack((x_points, y_points))

# Matriz de identidad
iden_mat = np.array([
    [1, 0],
    [0, 1]
])

# Matriz de escala
a = 1.5
d = 0.5
scale_mat = np.array([
    [a, 0],
    [0, d]
])

# Aplicar las transformaciones
id_pts = iden_mat  @ points
scale_pts = scale_mat @ points

fig, axs = plt.subplots(3, 1, figsize=(4, 12))
# --- Puntos Originales ---
axs[0].plot(0, 0, '+k', label='Origen')
axs[0].plot(points[0, :], points[1, :], 'x-k', label='Original')
axs[0].set_title("Original")
axs[0].axis('equal')
axs[0].grid(True)
axs[0].legend()

# --- Transformación Identidad ---
axs[1].plot(0, 0, '+k', label='Origen')
axs[1].plot(id_pts[0, :], id_pts[1, :], 'x-b', label='Identidad')
axs[1].set_title("Identidad")
axs[1].axis('equal')
axs[1].grid(True)
axs[1].legend()

# --- Transformación de Escala ---
axs[2].plot(0, 0, '+k', label='Origen')
axs[2].plot(scale_pts[0, :], scale_pts[1, :], 'x-g', label='Escala')
axs[2].set_title("Escala")
axs[2].axis('equal')
axs[2].grid(True)
axs[2].legend()

# Ajustar el espacio entre subplots
plt.tight_layout()
plt.show()
