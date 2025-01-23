import matplotlib.pyplot as plt
import numpy as np

# Definir los puntos iniciales
x1, y1 = 4, 2
P1 = np.array([x1, y1, 1]) 

# Definir el vector de traslación
sx = -1
sy = -1
# Matriz de traslación ver diapositiva 33 "Coordenadas homogéneas"
tras_m = np.array([[1, 0, sx], [0, 1, sy], [0, 0, 1]])  

# Calcular el punto transformado
P2 = tras_m @ P1
x2, y2 = P2[0], P2[1]  # Extraer coordenadas transformadas

# Crear la figura y los ejes
fig, ax = plt.subplots(figsize=(6, 6))

# Graficar el origen
ax.plot(0, 0, 'ko', label='O')  

# Graficar los vectores
ax.quiver(0, 0, x1, y1, angles='xy', scale_units='xy', scale=1, color='green', label=r'$(x_1, y_1)$', linewidth=2)  
ax.quiver(x1, y1, sx, sy, angles='xy', scale_units='xy', scale=1, color='blue', label=r'$(s_x, s_y)$', linewidth=2)  
ax.quiver(0, 0, x2, y2, angles='xy', scale_units='xy', scale=1, color='red', label=r'$(x_2, y_2)$', linewidth=2)  
ax.plot(x1, y1, 'go') 
ax.plot(x2, y2, 'ro')  

# Etiquetas de puntos y vectores
ax.text(x1+.2, y1, r'$(x_1, y_1)$', fontsize=12, color='green')
ax.text(x2 + 0.2, y2, r'$(x_2, y_2)$', fontsize=12, color='red')
ax.text(x1+sx/2+.2, y1 + sy / 2-.1, r'$(s_x, s_y)$', fontsize=12, color='blue')

# Dibujar ejes
ax.arrow(0, 0, 6, 0, head_width=0.2, head_length=0.3, fc='black', ec='black', linewidth=1)
ax.arrow(0, 0, 0, 6, head_width=0.2, head_length=0.3, fc='black', ec='black', linewidth=1)
ax.text(6.2, 0, 'X', fontsize=12, color='black')
ax.text(0, 6.2, 'Y', fontsize=12, color='black')
ax.arrow(x1,y1, 0, 1, head_width=0.2, head_length=0.3, fc='black', ec='black', linewidth=1)
ax.arrow(x1,y1, 1, 0, head_width=0.2, head_length=0.3, fc='black', ec='black', linewidth=1)
# Configurar límites, cuadrícula y proporción
ax.set_xlim(-1, 7)
ax.set_ylim(-1, 7)
ax.grid(True, linestyle='--', alpha=0.5)
ax.set_aspect('equal')

# Mostrar la gráfica
plt.show()
