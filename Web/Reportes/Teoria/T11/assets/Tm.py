"""
Código para generar la trayectoria de la letra M y un retorno circular
en el espacio articular (de la última pose al primer waypoint).
"""

import numpy as np
import time
import matplotlib.pyplot as plt
from spatialmath import SE3
import roboticstoolbox as rtb
from roboticstoolbox import ctraj, jtraj

# ---------------------------
#    1. Definición del robot
# ---------------------------
a1 = 0.02
a2 = 0.10
a3 = 0.08

name = "Robot_3ejes"
robot = rtb.DHRobot(
    [
        rtb.RevoluteDH(alpha=-np.pi/2, a=0,    d=a1, offset=0, qlim=(-np.deg2rad(360), np.deg2rad(360))),
        rtb.RevoluteDH(alpha=0,        a=a2,   d=0,  offset=0, qlim=(-np.deg2rad(210), np.deg2rad(30))),
        rtb.RevoluteDH(alpha=0,        a=a3,   d=0,  offset=0, qlim=(-np.deg2rad(170), np.deg2rad(170)))
    ],
    name=name
)
print("Robot details:")
print(robot)

# ---------------------------
#    2. Creación de matriz de coordenadas para trayectoria
# ---------------------------
# Para este ejemplo se genera la letra M en el plano XZ, con offset en Y
offset_y = 0.05  # Desplazamiento en Y

# Definir los vértices 3D (en este caso en el plano XZ, Y constante)
vertices_3d = [
    (-0.10, 0, offset_y),       # punto inferior izquierdo
    (-0.09, 0.075, offset_y),   # punto superior izquierdo
    (-0.06, 0.05, offset_y),    # punto medio izquierdo (ajustado)
    (-0.03, 0.075, offset_y),   # punto medio derecho (ajustado)
    (-0.02, 0, offset_y),       # punto inferior derecho
]

# Inicializar P con el primer vértice
P = np.array(vertices_3d[0]).reshape(3, 1)
# Concatenar el resto de los vértices
for v in vertices_3d[1:]:
    new_col = np.array(v).reshape(3, 1)
    P = np.concatenate((P, new_col), axis=1)

#########################################
#        Revisar espacio de trabajo
#########################################
def es_punto_lineal_o_articulado(x, y, z, radio_max, apertura_cono):
    # Calcula el lado del cubo a partir de la esfera
    lado_cubo = 2 * radio_max / np.sqrt(3)
    dentro_del_cubo = abs(x) <= lado_cubo/2 and abs(y) <= lado_cubo/2 and abs(z) <= lado_cubo/2
    angulo_cono = np.arctan2(np.sqrt(x**2 + y**2), -z)
    fuera_del_cono = angulo_cono > apertura_cono
    es_lineal = dentro_del_cubo and fuera_del_cono
    distancia_al_origen = np.sqrt(x**2 + y**2 + z**2)
    es_articular = distancia_al_origen <= radio_max and fuera_del_cono
    return es_lineal, es_articular

radio_max = a2 + a3
apertura_cono = np.deg2rad(60)

# Verificar cada waypoint
for i in range(P.shape[1]):
    x, y, z = P[:, i].flatten()
    es_lineal, es_articular = es_punto_lineal_o_articulado(x, y, z, radio_max, apertura_cono)
    print(f"WP{i+1}: lineal - {es_lineal}, articulado - {es_articular}")

# Matriz de rotación (identidad en este caso)
R = np.array([[1, 0, 0],
              [0, 1, 0],
              [0, 0, 1]])

# Crear una lista de poses SE3 para cada waypoint
coordenadas = [SE3.Rt(R, [x, y, z]) for x, y, z in zip(P[0], P[1], P[2])]
print("\nCoordenadas en el espacio de trabajo:")
for i, pos in enumerate(coordenadas):
    print(f"WP{i+1}: {pos}")

# ---------------------------
#    3. Generar la trayectoria usando ctraj (trayectoria cartesiana)
# ---------------------------
T_segment = 10      # Duración conceptual de cada segmento en segundos
n_points = 50       # Número de puntos por segmento
q_traj_segments = []  # Lista para almacenar trayectorias en espacio articular
q_current = np.zeros(robot.n)  # Condición inicial para IK

ik_start = time.time()

# Para cada segmento entre waypoints consecutivos (trayectoria cartesiana)
for i in range(len(coordenadas) - 1):
    cart_traj = ctraj(coordenadas[i], coordenadas[i+1], n_points)
    q_traj_seg = []  # Trayectoria articular para este segmento
    for pose in cart_traj:
        sol = robot.ikine_LM(pose, q0=q_current, mask=(1,1,1,0,0,0))
        if sol.success:
            q_traj_seg.append(sol.q)
            q_current = sol.q  # Actualizar q_current para la siguiente iteración
        else:
            print(f"IK falló en el segmento {i+1} para una pose intermedia")
    q_traj_segments.append(np.array(q_traj_seg))

ik_end = time.time()
print("\nTiempo total de cálculo de IK en segmentos: {:.4f} segs".format(ik_end - ik_start))

# ---------------------------
#    4. Generar trayectoria de retorno circular en el espacio articular
#    (de la última pose al primer waypoint)
# ---------------------------
# Obtener las configuraciones articulares para el primer y último waypoint
sol_first = robot.ikine_LM(coordenadas[0], q0=q_current, mask=(1,1,1,0,0,0))
if sol_first.success:
    q_first = sol_first.q
    q_current = sol_first.q
    print(f"IK para primer waypoint resuelta: {q_first}")
else:
    print("IK falló para primer waypoint")

sol_last = robot.ikine_LM(coordenadas[-1], q0=q_current, mask=(1,1,1,0,0,0))
if sol_last.success:
    q_last = sol_last.q
    q_current = sol_last.q
    print(f"IK para último waypoint resuelta: {q_last}")
else:
    print("IK falló para último waypoint")

# Definir la parametrización del círculo en el espacio articular
# Queremos pasar de q_last (inicio del retorno) a q_first (final del retorno)
m = (q_last + q_first) / 2           # Centro del arco
d = q_first - q_last                 # Vector de la cuerda
r = np.linalg.norm(d) / 2            # Radio del círculo

# Elegir un vector arbitrario para definir el plano del círculo
v = np.array([0, 0, 1])
if np.allclose(np.cross(d, v), 0):
    v = np.array([0, 1, 0])
u = np.cross(d, v)
u = u / np.linalg.norm(u)

n_points_circular = 50
theta = np.linspace(0, np.pi, n_points_circular)
q_circular = []
for t in theta:
    # Parametrización del círculo:
    # Para t=0 se obtiene q_last y para t=pi se obtiene q_first
    q_t = m + (q_last - m) * np.cos(t) + u * r * np.sin(t)
    q_circular.append(q_t)
q_circular = np.array(q_circular)

# Agregar la trayectoria circular a la lista de segmentos de trayectoria
q_traj_segments.append(q_circular)
print("Trayectoria de retorno circular generada exitosamente.")

# Concatenar todas las trayectorias en una secuencia completa de configuraciones articulares
q_traj = np.vstack(q_traj_segments)

# ---------------------------
#    5. Evaluar la trayectoria generada usando FK
# ---------------------------
task_traj = []
for q in q_traj:
    T_fk = robot.fkine(q)
    task_traj.append(T_fk.t)  # Extraer la traslación de la pose
task_traj = np.array(task_traj)

# Para comparar, se cierra la trayectoria ideal agregando el primer waypoint al final
P_closed = np.hstack((P, P[:, 0:1]))

# ---------------------------
#    6. Graficar Trayectorias, Ángulos, Velocidades y Aceleraciones
# ---------------------------
# Plot de la trayectoria en el espacio de trabajo
traj_fig = plt.figure("Trayectorias")
ax = traj_fig.add_subplot(111, projection='3d')
ax.plot(task_traj[:, 0], task_traj[:, 1], task_traj[:, 2],
        'b-', label='Trayectoria generada (IK)')
ax.plot(P_closed[0], P_closed[1], P_closed[2],
        'r:', linewidth=2, label='Trayectoria Ideal')
ax.scatter(P[0], P[1], P[2],
           color='red', marker='o', s=50, label='Coordenadas')
ax.set_title('Trayectoria en el Espacio de Trabajo')
ax.set_xlabel('X [m]')
ax.set_ylabel('Y [m]')
ax.set_zlabel('Z [m]')
ax.legend()
ax.set_box_aspect([1, 1, 1])

# Vector de tiempo para la trayectoria completa.
num_segments = len(q_traj_segments)
time_total = T_segment * num_segments
t_vec = np.linspace(0, time_total, q_traj.shape[0])

# Calcular velocidades y aceleraciones usando diferencias finitas
qd_traj = np.gradient(q_traj, t_vec[1]-t_vec[0], axis=0)
qdd_traj = np.gradient(qd_traj, t_vec[1]-t_vec[0], axis=0)

# Plot de Ángulo de Articulaciones vs Tiempo
fig_joint = plt.figure("Ángulo vs Tiempo")
for i in range(q_traj.shape[1]):
    plt.plot(t_vec, q_traj[:, i], label=f'J{i+1}')
plt.xlabel("Tiempo (s)")
plt.ylabel("Ángulo (rad)")
plt.title("Ángulo de Articulaciones vs Tiempo")
plt.legend()
plt.grid(True)

# Plot de Velocidades de Articulaciones vs Tiempo
fig_vel = plt.figure("Velocidades vs Tiempo")
for i in range(qd_traj.shape[1]):
    plt.plot(t_vec, qd_traj[:, i], label=f'J{i+1}')
plt.xlabel("Tiempo (s)")
plt.ylabel("Velocidad (rad/s)")
plt.title("Velocidades de Articulaciones vs Tiempo")
plt.legend()
plt.grid(True)

# Plot de Aceleraciones de Articulaciones vs Tiempo
fig_acc = plt.figure("Aceleraciones vs Tiempo")
for i in range(qdd_traj.shape[1]):
    plt.plot(t_vec, qdd_traj[:, i], label=f'J{i+1}')
plt.xlabel("Tiempo (s)")
plt.ylabel("Aceleración (rad/s²)")
plt.title("Aceleraciones de Articulaciones vs Tiempo")
plt.legend()
plt.grid(True)

# Animacion del robot utilizando la trayectoria en espacio articular
robot.plot(q_traj, block=False, loop=True)
plt.show()