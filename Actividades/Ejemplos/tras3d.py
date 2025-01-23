import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

# Definir el punto inicial en 3D
x1, y1, z1 = 5, 4, 2
P1 = np.array([x1, y1, z1, 1])  # Punto inicial en coordenadas homogéneas

# Definir el vector de traslación en 3D
sx, sy, sz = -4, -1, 2
tras_m = np.array([
    [1, 0, 0, sx],
    [0, 1, 0, sy],
    [0, 0, 1, sz],
    [0, 0, 0, 1]
])  # Matriz de traslación en 3D

# Calcular el punto transformado
P2 = tras_m @ P1
x2, y2, z2 = P2[0], P2[1], P2[2]  # Extraer coordenadas transformadas

# Crear la figura y los ejes 3D
fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, projection='3d')

# Graficar el origen
ax.scatter(0, 0, 0, color='black')  # Origen

# Graficar los vectores
ax.quiver(0, 0, 0, x1, y1, z1, color='green', linewidth=2)  # Vector inicial
ax.quiver(x1, y1, z1, sx, sy, sz, color='blue', linewidth=2)  # Vector de traslación
ax.quiver(0, 0, 0, x2, y2, z2, color='red', linewidth=2)  # Vector transformado

# Marcar los puntos
ax.scatter(x1, y1, z1, color='green')  # Punto inicial
ax.scatter(x2, y2, z2, color='red')  # Punto transformado

# Etiquetas de puntos y vectores
ax.text(x1, y1, z1, r'$(x_1, y_1, z_1)$', fontsize=10, color='green')
ax.text(x2, y2, z2, r'$(x_2, y_2, z_2)$', fontsize=10, color='red')
ax.text(x1 + sx / 2, y1 + sy / 2, z1 + sz / 2, r'$(s_x, s_y, s_z)$', fontsize=10, color='blue')

# Graficar las flechas de los ejes
ax.quiver(0, 0, 0, 5, 0, 0, color='black', linestyle='-', linewidth=1, arrow_length_ratio=0.1)  # Eje X
ax.quiver(0, 0, 0, 0, 5, 0, color='black', linestyle='-', linewidth=1, arrow_length_ratio=0.1)  # Eje Y
ax.quiver(0, 0, 0, 0, 0, 5, color='black', linestyle='-', linewidth=1, arrow_length_ratio=0.1)  # Eje Z

# Etiquetas de los ejes
ax.text(5.2, 0, 0, 'X', fontsize=12, color='black')
ax.text(0, 5.2, 0, 'Y', fontsize=12, color='black')
ax.text(0, 0, 5.2, 'Z', fontsize=12, color='black')

# Configurar los ejes
ax.set_xlim(-1, 6)
ax.set_ylim(-1, 6)
ax.set_zlim(-1, 6)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Agregar cuadrícula, leyenda y mostrar
ax.grid(True, linestyle='--', alpha=0.5)
ax.legend()
plt.show()
