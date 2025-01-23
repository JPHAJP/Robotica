import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Line3DCollection
import numpy as np

# Definir los vértices del cubo (como vectores fila)
vertices = np.array([[-0.5, -0.5, -0.5],
                     [ 0.5, -0.5, -0.5],
                     [ 0.5,  0.5, -0.5],
                     [-0.5,  0.5, -0.5],
                     [-0.5, -0.5,  0.5],
                     [ 0.5, -0.5,  0.5],
                     [ 0.5,  0.5,  0.5],
                     [-0.5,  0.5,  0.5]])

# Transponer los vértices para que sean vectores columna (matriz 3 x n)
vertices = vertices.T

aristas = [[0, 1], [1, 2], [2, 3], [3, 0], 
           [4, 5], [5, 6], [6, 7], [7, 4],  
           [0, 4], [1, 5], [2, 6], [3, 7]]

# Definir los ángulos de rotación en grados
ang_z = 0  # Rotación alrededor del eje Z
ang_x = 0  # Rotación alrededor del eje X
ang_y = 15  # Rotación alrededor del eje Y

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

# Aplicar las rotaciones en el orden correcto: Z -> Y -> X
rot_mat = rot_z @ rot_y @ rot_x

# Rotar los vértices
rotated_vertices = rot_mat @ vertices

# Transponer el resultado de vuelta a vectores fila
rotated_vertices = rotated_vertices.T

# Extraer los segmentos de línea para el cubo original
og_lin = [[vertices.T[init], vertices.T[fin]] for init, fin in aristas]
og_lin = np.array(og_lin)

# Extraer los segmentos de línea para el cubo rotado
rot_lin1 = [[rotated_vertices[init], rotated_vertices[fin]] for init, fin in aristas]
rot_lin1 = np.array(rot_lin1)

# Crear un gráfico 3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Graficar las aristas del cubo original
lc_original = Line3DCollection(og_lin, colors='blue', linewidths=1, label="Original")
ax.add_collection3d(lc_original)

# Graficar las aristas del cubo rotado
lc_rotated1 = Line3DCollection(rot_lin1, colors='red', linewidths=1, label="Rotado")
ax.add_collection3d(lc_rotated1)

# Añadir los ejes para demostrar la regla de la mano derecha
ax.quiver(0, 0, 0, 1, 0, 0, color="black", arrow_length_ratio=0.1, linewidth=2)
ax.text(1.05, 0, 0, 'X', color='black', fontsize=15)
ax.quiver(0, 0, 0, 0, 1, 0, color="black", arrow_length_ratio=0.1, linewidth=2)
ax.text(0, 1.05, 0, 'Y', color='black', fontsize=15)
ax.quiver(0, 0, 0, 0, 0, 1, color="black", arrow_length_ratio=0.1, linewidth=2)
ax.text(0, 0, 1.05, 'Z', color='black', fontsize=15)

# Configurar los límites de los ejes para ajustar ambos cubos
plot_limit = 1  # El tamaño del cubo centrado en el origen
ax.set_xlim(-plot_limit, plot_limit)
ax.set_ylim(-plot_limit, plot_limit)
ax.set_zlim(-plot_limit, plot_limit)

# Etiquetar los ejes
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Añadir una leyenda
ax.legend(loc='upper right')

# Mostrar el gráfico
plt.show()
