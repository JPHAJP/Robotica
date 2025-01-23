import numpy as np
import matplotlib.pyplot as plt

# Punto Inicial
x1, y1 = 3.5, 2
theta = 30
theta_rad = np.radians(theta)

# Matriz de Rotación
rot_mat = np.array([
    [np.cos(theta_rad), -np.sin(theta_rad)],
    [np.sin(theta_rad), np.cos(theta_rad)]
])


# Aplicar la matriz de rotación
x2, y2 = rot_mat @ np.array([x1, y1])


plt.figure(figsize=(8, 8))


plt.arrow(0, 0, x1, y1, head_width=0.1, head_length=0.1, fc='blue', ec='blue', linewidth=4.0)
plt.arrow(0, 0, x2, y2, head_width=0.1, head_length=0.1, fc='green', ec='green', linewidth=4.0)

mid_x1, mid_y1 = x1 / 2, y1 / 2  
mid_x2, mid_y2 = x2 / 2, y2 / 2  

plt.annotate('h', 
             (mid_x1, mid_y1), 
             textcoords="offset points", xytext=(10, 10), ha='center', color='blue', fontsize=12, weight='bold')

plt.annotate('h', 
             (mid_x2 - 0.1, mid_y2), 
             textcoords="offset points", xytext=(10, 10), ha='center', color='green', fontsize=12, weight='bold')

plt.xlabel("X-axis")
plt.ylabel("Y-axis")
plt.axhline(0, color='black', linewidth=0.5, linestyle='dotted')
plt.axvline(0, color='black', linewidth=0.5, linestyle='dotted')
plt.grid(color='gray', linestyle='--', linewidth=0.5)
plt.legend()

plt.axis("equal")
plt.show()
