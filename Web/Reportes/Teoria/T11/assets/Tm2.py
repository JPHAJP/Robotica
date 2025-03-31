"""
Código combinado para generar la trayectoria de la letra M con movimientos lineales
(en los segmentos de la M) y un retorno articulado (medio círculo) para cerrar la M.

Los segmentos de la M se ejecutan con movimientos lineales en el espacio cartesiano,
mientras que el retorno del último vértice al primero se hace mediante un movimiento
articulado en el espacio de articulaciones.
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

# Definir los vértices 3D (en este caso en el plano XZ, Y constante) para definir M
vertices_3d = [
    (-0.10, offset_y, 0     ),      # punto inferior izquierdo
    (-0.09, offset_y, 0.075 ),      # punto superior izquierdo
    (-0.06, offset_y, 0.05  ),      # punto medio izquierdo (ajustado)
    (-0.03, offset_y, 0.075 ),      # punto medio derecho (ajustado)
    (-0.02, offset_y, 0     ),      # punto inferior derecho
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
#    3. Generar la trayectoria lineal para los segmentos de la M
# ---------------------------
T_segment = 5      # Duración conceptual de cada segmento en segundos
n_points = 50       # Número de puntos por segmento
q_traj_segments = []  # Lista para almacenar trayectorias en espacio articular
q_current = np.zeros(robot.n)  # Condición inicial para IK

print("\nGenerando trayectorias lineales para los segmentos de la M:")
ik_start = time.time()

# Para cada segmento entre waypoints consecutivos (trayectoria cartesiana LINEAL)
for i in range(len(coordenadas) - 1):
    print(f"Generando trayectoria lineal para el segmento {i+1}/{len(coordenadas)-1}")
    # ctraj genera una trayectoria LINEAL en el espacio cartesiano
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
print("\nTiempo total de cálculo de IK en segmentos lineales: {:.4f} segs".format(ik_end - ik_start))

# ---------------------------
#    4. Generar trayectoria de retorno circular en el espacio articular
#    (de la última pose al primer waypoint)
# ---------------------------
print("\nGenerando trayectoria articulada para el medio círculo de retorno:")

# Generar puntos 3D para el medio círculo
n_points_arc = 50
P_inferior_derecho = np.array(vertices_3d[-1]).reshape(3,1)  # (-0.02, offset_y, 0)
P_inferior_izquierdo = np.array(vertices_3d[0]).reshape(3,1)  # (-0.10, offset_y, 0)

# Calcular el centro (m) y el radio (r) del arco
m = (P_inferior_derecho + P_inferior_izquierdo) / 2  # centro del arco
r = np.linalg.norm(P_inferior_derecho - m)           # radio del arco

# Generar puntos del medio círculo en el plano XZ (Y constante)
theta = np.linspace(0, np.pi, n_points_arc)
P_arc = np.zeros((3, n_points_arc))
for i, t in enumerate(theta):
    # Parametrización: P(t) = m + r*[cos(t), 0, sin(t)]
    # NOTA: Ajustamos para que en theta=0 estemos en P_inferior_derecho
    x = m[0,0] + r * np.cos(t)
    y = offset_y
    z = m[2,0] + r * np.sin(t)
    P_arc[:, i] = [x, y, z]

# Crear poses SE3 para cada punto del arco
arc_poses = []
for i in range(n_points_arc):
    arc_poses.append(SE3.Rt(R, P_arc[:, i]))

# Resolver la cinemática inversa para cada pose del arco
# Usar la última configuración articular como punto de partida
q_arc = []
q_current = q_traj_segments[-1][-1]  # Última configuración articular de los segmentos de la M

ik_arc_start = time.time()
print("\nResolviendo cinemática inversa para los puntos del arco:")

for i, pose in enumerate(arc_poses):
    sol = robot.ikine_LM(pose, q0=q_current, mask=(1,1,1,0,0,0))
    if sol.success:
        q_arc.append(sol.q)
        q_current = sol.q  # Actualizar para la siguiente iteración
        if i % 10 == 0:  # Mostrar progreso cada 10 puntos
            print(f"IK resuelta para punto {i+1}/{n_points_arc} del arco")
    else:
        print(f"IK falló para el punto {i+1} del arco")
        # Si falla, intentar mantener la última configuración válida
        if len(q_arc) > 0:
            q_arc.append(q_arc[-1])
        else:
            # Si no hay configuración previa, usar la última de los segmentos de la M
            q_arc.append(q_traj_segments[-1][-1])

ik_arc_end = time.time()
print(f"Tiempo total de cálculo de IK para el arco: {ik_arc_end - ik_arc_start:.4f} segs")

# Convertir a array numpy y añadir a los segmentos
q_arc = np.array(q_arc)
q_traj_segments.append(q_arc)
print("Trayectoria del arco generada exitosamente.")

# Concatenar todas las trayectorias en una secuencia completa de configuraciones articulares
q_traj = np.vstack(q_traj_segments)

# ---------------------------
#    5. Evaluar la trayectoria generada usando FK
# ---------------------------
print("\nEvaluando la trayectoria generada usando cinemática directa (FK)...")
task_traj = []
for q in q_traj:
    T_fk = robot.fkine(q)
    task_traj.append(T_fk.t)  # Extraer la traslación de la pose
task_traj = np.array(task_traj)

# Para comparar, crear una trayectoria ideal completa (la M lineal + el medio círculo)
P_ideal = np.hstack((P, P_arc))  # Concatenar la M y el arco
P_ideal_closed = np.hstack((P_ideal, P_ideal[:, 0:1]))  # Cerrar la trayectoria ideal

# ---------------------------
#    6. Graficar Trayectorias, Ángulos, Velocidades y Aceleraciones
# ---------------------------
print("\nGenerando gráficas...")

# Plot de la trayectoria en el espacio de trabajo
traj_fig = plt.figure("Trayectorias")
ax = traj_fig.add_subplot(111, projection='3d')
ax.plot(task_traj[:, 0], task_traj[:, 1], task_traj[:, 2],
        'b-', label='Trayectoria generada (FK)')
ax.plot(P_ideal_closed[0], P_ideal_closed[1], P_ideal_closed[2],
        'r:', linewidth=2, label='Trayectoria Ideal')
ax.scatter(P[0], P[1], P[2],
           color='red', marker='o', s=50, label='Waypoints M')
ax.scatter(P_arc[0], P_arc[1], P_arc[2],
           color='green', marker='.', s=20, label='Puntos del arco')
ax.set_title('Trayectoria en el Espacio de Trabajo')
ax.set_xlabel('X [m]')
ax.set_ylabel('Y [m]')
ax.set_zlabel('Z [m]')
ax.legend()
ax.set_box_aspect([1, 1, 1])

# Calcular un vector de tiempo apropiado para toda la trayectoria
num_segments_linear = len(q_traj_segments) - 1  # Excluyendo el arco
T_arc = 5  # Duración del arco

# Vector de tiempo total
t_vec = np.zeros(q_traj.shape[0])
idx = 0

# Asignar tiempos para los segmentos lineales
for i in range(num_segments_linear):
    n_seg = q_traj_segments[i].shape[0]
    t_seg = np.linspace(i*T_segment, (i+1)*T_segment, n_seg)
    t_vec[idx:idx+n_seg] = t_seg
    idx += n_seg

# Asignar tiempos para el arco
n_arc = q_traj_segments[-1].shape[0]
t_arc = np.linspace(num_segments_linear*T_segment, num_segments_linear*T_segment + T_arc, n_arc)
t_vec[idx:idx+n_arc] = t_arc

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

# Añadir líneas verticales para indicar dónde comienza la trayectoria del arco
tiempo_inicio_arco = num_segments_linear*T_segment
plt.figure("Ángulo vs Tiempo")
plt.axvline(x=tiempo_inicio_arco, color='k', linestyle='--', label='Inicio Arco')
plt.figure("Velocidades vs Tiempo")
plt.axvline(x=tiempo_inicio_arco, color='k', linestyle='--', label='Inicio Arco')
plt.figure("Aceleraciones vs Tiempo")
plt.axvline(x=tiempo_inicio_arco, color='k', linestyle='--', label='Inicio Arco')

print("Mostrando animación del robot...")
# Animacion del robot utilizando la trayectoria en espacio articular
robot.plot(q_traj, block=False, loop=True)
plt.show()