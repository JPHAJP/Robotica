import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Line3DCollection
import numpy as np

# T: A -> B
print('T: A -> B')
# Definir el punto inicial AP en 3D
x1, y1, z1 = 0, 0, 0
PA1 = np.array([x1, y1, z1, 1])  # Punto inicial en coordenadas homogéneas

#1° Se rota en el eje Z 180°

# Definir los ángulos de rotación en grados
ang_z = 180  # Rotación alrededor del eje Z

# Convertir los ángulos a radianes
theta_z = np.radians(ang_z)

# Definir las matrices de rotación
rot_z = np.array([[np.cos(theta_z), -np.sin(theta_z), 0, 0],
                  [np.sin(theta_z),  np.cos(theta_z), 0, 0],
                  [0,               0,                1, 0],
                  [0,               0,                0, 1]])

# Definir los vectores originales
x_axis = np.array([1, 0, 0, 1])  # Vector en X
y_axis = np.array([0, 1, 0, 1])  # Vector en Y
z_axis = np.array([0, 0, 1, 1])  # Vector en Z
# Aplicar la rotación a los vectores X e Y
rot_x2_axis = rot_z @ x_axis
rot_y2_axis = rot_z @ y_axis
rot_z2_axis = rot_z @ z_axis

# Rotar el punto
rot_P1_Z = rot_z @ PA1
print("1°: Rotacion 180Z")
print(rot_P1_Z)

#2° Se mueve 3 unidades en X a la derecha

# Definir el vector de traslación en 3D
sx, sy, sz = 3, 0, 0
tras_m = np.array([
    [1, 0, 0, sx],
    [0, 1, 0, sy],
    [0, 0, 1, sz],
    [0, 0, 0,  1]
])  # Matriz de traslación en 3D

# Calcular el punto transformado
P2 = tras_m @ rot_P1_Z
x2, y2, z2 = P2[0], P2[1], P2[2]  # Extraer coordenadas transformadas
print("2°: Traslacion 3X")
print(x2, y2, z2)
'''-----------------------------------------------------------------------'''
# T: A -s vértices> C
print('')
print('T: A -> C')
#1° Rotar 90° eje Y

# Definir los ángulos de rotación en grados
ang_y = 90  # Rotación alrededor del eje Y

# Convertir los ángulos a radianes
theta_y = np.radians(ang_y)

rot_y = np.array([[np.cos(theta_y),  0, np.sin(theta_y), 0],
                  [        0,        1,         0,       0],
                  [-np.sin(theta_y), 0, np.cos(theta_y), 0],
                  [        0,        0,         0,       1]])

# Rotar el punto
rot_P1_Y = rot_y @ PA1
print("1°: Rotacion 90Y")
print(rot_P1_Y)

# Definir los vectores originales
x_axis = np.array([1, 0, 0, 1])  # Vector en X
y_axis = np.array([0, 1, 0, 1])  # Vector en Y
z_axis = np.array([0, 0, 1, 1])  # Vector en Z
# Aplicar la rotación a los vectores 
rot_1x3_axis = rot_y @ x_axis
rot_1y3_axis = rot_y @ y_axis
rot_1z3_axis = rot_y @ z_axis

#2° Rotar 30° eje X (eje X original, el Z en este caso)

# Definir los ángulos de rotación en grados
ang_z = 30  # Rotación alrededor del eje X

# Convertir los ángulos a radianes
theta_z = np.radians(ang_z)

# Definir las matrices de rotación
rot_z = np.array([[np.cos(theta_z), -np.sin(theta_z), 0, 0],
                  [np.sin(theta_z),  np.cos(theta_z), 0, 0],
                  [       0,               0,         1, 0],
                  [       0,               0,         0, 1]])

# Rotar el punto
rot_P3 = rot_z @ rot_P1_Y
print("2°: Rotacion 30X")
print(rot_P3)

# Aplicar la rotación en X a los vectores
rot_x3_axis = rot_z @ rot_1x3_axis
rot_y3_axis = rot_z @ rot_1y3_axis
rot_z3_axis = rot_z @ rot_1z3_axis

#3° Trasladar 3 unidades a la derecha

# Definir el vector de traslación en 3D
sx, sy, sz = 3, 0, 0
tras_mX = np.array([
    [1, 0, 0, sx],
    [0, 1, 0, sy],
    [0, 0, 1, sz],
    [0, 0, 0,  1]
])  # Matriz de traslación en 3D

# Calcular el punto transformado
P3 = tras_mX @ rot_P3
x3_1, y3_1, z3_1= P3[0], P3[1], P3[2]  # Extraer coordenadas transformadas
print("3°: Traslacion 3X")
print(x3_1, y3_1, z3_1)

#4° Trasladar 2 unidades arriba

# Definir el vector de traslación en 3D
sx, sy, sz = 0, 2, 0
tras_mY = np.array([
    [1, 0, 0, sx],
    [0, 1, 0, sy],
    [0, 0, 1, sz],
    [0, 0, 0,  1]
])  # Matriz de traslación en 3D

# Calcular el punto transformado
P3_1 = tras_mY @ P3
x3, y3, z3 = P3_1[0], P3_1[1], P3_1[2]  # Extraer coordenadas transformadas
print("4°: Traslacion 2Y")
print(x3, y3, z3)

#Dibujos
# Crear la figura y los ejes 3D
fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, projection='3d')

# Graficar el origen
ax.scatter(0, 0, 0, color='black')  # Origen

# Marcar los puntos
ax.scatter(x1, y1, z1, color='green')  # Punto inicial
# Graficar las flechas de los ejes
ax.quiver(x1, y1, z1, 1, 0, 0, color='red', linestyle='-', linewidth=1, arrow_length_ratio=0.1)  # Eje X
ax.quiver(x1, y1, z1, 0, 1, 0, color='green', linestyle='-', linewidth=1, arrow_length_ratio=0.1)  # Eje Y
ax.quiver(x1, y1, z1, 0, 0, 1, color='blue', linestyle='-', linewidth=1, arrow_length_ratio=0.1)  # Eje Z

ax.scatter(x2, y2, z2, color='red')  # Punto transformado AB
# Graficar las flechas de los ejes
ax.quiver(x2, y2, z2, *rot_x2_axis, color='red', linestyle='-', linewidth=1, arrow_length_ratio=0.1)  # Eje X
ax.quiver(x2, y2, z2, *rot_y2_axis, color='green', linestyle='-', linewidth=1, arrow_length_ratio=0.1)  # Eje Y
ax.quiver(x2, y2, z2, *rot_z2_axis, color='blue', linestyle='-', linewidth=1, arrow_length_ratio=0.1)  # Eje Z

ax.scatter(x3, y3, z3, color='blue')  # Punto transformado AC
# Graficar las flechas de los ejes
ax.quiver(x3, y3, z3, *rot_x3_axis, color='red', linestyle='-', linewidth=1, arrow_length_ratio=0.1)  # Eje X
ax.quiver(x3, y3, z3, *rot_y3_axis, color='green', linestyle='-', linewidth=1, arrow_length_ratio=0.1)  # Eje Y
ax.quiver(x3, y3, z3, *rot_z3_axis, color='blue', linestyle='-', linewidth=1, arrow_length_ratio=0.1)  # Eje Z

# Etiquetas de puntos y vectores
ax.text(x1, y1, z1, r'$(x_1, y_1, z_1)$', fontsize=10, color='green')
ax.text(x2, y2, z2, r'$(x_2, y_2, z_2)$', fontsize=10, color='red')
ax.text(x3, y3, z3, r'$(x_3, y_3, z_3)$', fontsize=10, color='blue')


# Configurar los ejes
ax.set_xlim(-1, 6)
ax.set_ylim(-1, 6)
ax.set_zlim(-1, 6)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Agregar cuadrícula, leyenda y mostrar
ax.grid(True, linestyle='--', alpha=0.5)
#plt.show()