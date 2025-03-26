import numpy as np
import time
import matplotlib.pyplot as plt
from spatialmath import SE3
import roboticstoolbox as rtb
from roboticstoolbox import ctraj

#---------------------------
#    1. Definicion del robot
#---------------------------
a1 = 0.02
a2 = 0.10
a3 = 0.08

name = "Robot_3ejes"
robot = rtb.DHRobot(
    [
        rtb.RevoluteDH(alpha=-np.pi/2,  a=0,    d=a1,  offset=0,  qlim=(-np.deg2rad(360), np.deg2rad(360))),
        rtb.RevoluteDH(alpha=0,         a=a2,   d=0,   offset=0,  qlim=(-np.deg2rad(210), np.deg2rad(30))),
        rtb.RevoluteDH(alpha=0,         a=a3,   d=0,   offset=0,  qlim=(-np.deg2rad(170), np.deg2rad(170)))
    ],
    name=name
)
print("Robot details:")
print(robot)

'''
---------------------------
    2. Creacion de matriz de coordenadas para trayectoria
---------------------------
'''
# Inicializa el primer waypoint
P = np.array([[0.005],                              # X
              [0.005 * np.cos(np.deg2rad(0))],        # Y
              [0.005 * np.sin(np.deg2rad(0)) + 0.15]])  # Z

# Genera waypoints a lo largo de una circunferencia (179 puntos adicionales cada 2 grados)
for i in range(179):
    new_col = np.array([0.005, 
                        0.005 * np.cos(np.deg2rad(i*2)), 
                        0.005 * np.sin(np.deg2rad(i*2)) + 0.15]).reshape(3, 1)
    P = np.concatenate((P, new_col), axis=1)

# Matriz de rotacion para la orientacion
R = np.array([[1, 0, 0],
              [0, 1, 0],
              [0, 0, 1]])

# Se crea una lista de objetos SE3 para cada waypoint
coordenadas = [SE3.Rt(R, [x, y, z]) for x, y, z in zip(P[0], P[1], P[2])]

print("\nCoordenadas en el espacio de trabajo:")
for i, pos in enumerate(coordenadas):
    print(f"WP{i+1}: {pos}")

'''
---------------------------
    3. Generar la trayectoria usando ctraj (trayectoria cartesiana)
---------------------------
'''
T_segment = 5      # Duracion conceptual de cada segmento en segundos
n_points = 5       # Numero de puntos por segmento (cartesiano)
q_traj_segments = []  # Lista para almacenar trayectorias en espacio articular
q_current = np.zeros(robot.n)  # Condicion inicial para IK

ik_start = time.time()

# Para cada segmento entre waypoints consecutivos
for i in range(len(coordenadas) - 1):
    cart_traj = ctraj(coordenadas[i], coordenadas[i+1], n_points)
    q_traj_seg = []  # Trayectoria articular para este segmento
    for pose in cart_traj:
        sol = robot.ikine_LM(pose, q0=q_current, mask=(1,1,1,0,0,0))
        if sol.success:
            q_traj_seg.append(sol.q)
            q_current = sol.q  # Actualiza q_current para la siguiente pose
        else:
            print(f"IK fallo en el segmento {i+1} para una pose intermedia")
    q_traj_segments.append(np.array(q_traj_seg))

# Trayectoria de retorno: de la ultima pose al primer waypoint
cart_traj = ctraj(coordenadas[-1], coordenadas[0], n_points)
q_traj_seg = []
for pose in cart_traj:
    sol = robot.ikine_LM(pose, q0=q_current, mask=(1,1,1,0,0,0))
    if sol.success:
        q_traj_seg.append(sol.q)
        q_current = sol.q
    else:
        print("IK fallo en el segmento de retorno para una pose intermedia")
q_traj_segments.append(np.array(q_traj_seg))

ik_end = time.time()
print("\nTiempo total de calculo de IK: {:.4f} segs".format(ik_end - ik_start))

# Concatena todas las trayectorias en una sola secuencia de configuraciones articulares
q_traj = np.vstack(q_traj_segments)

'''
---------------------------
    4. Evaluamos la trayectoria generada usando FK
---------------------------
'''
task_traj = []
for q in q_traj:
    T_fk = robot.fkine(q)
    task_traj.append(T_fk.t)  # Extrae la traslacion de la pose
task_traj = np.array(task_traj)

# Para comparacion, se cierra la trayectoria ideal agregando el primer waypoint al final
P_closed = np.hstack((P, P[:, 0:1]))

'''
---------------------------
    5. Graficar Trayectorias, Ángulos, Velocidades y Aceleraciones
---------------------------
'''
# Plot de la trayectoria en el espacio de trabajo
traj_fig = plt.figure("Trayectorias")
ax = traj_fig.add_subplot(111, projection='3d')
ax.plot(task_traj[:, 0], task_traj[:, 1], task_traj[:, 2],
        'b-', label='Trayectoria generada (IK)')
ax.plot(P_closed[0], P_closed[1], P_closed[2],
        'r:', linewidth=2, label='Trayectoria Ideal')
ax.scatter(P[0], P[1], P[2],
           color='red', marker='o', s=50, label='Coordenadas')
# for i in range(P.shape[1]):
    # ax.text(P[0, i], P[1, i], P[2, i], f' WP{i+1}', color='black', fontsize=12)
ax.set_title('Trayectoria en el Espacio de Trabajo')
ax.set_xlabel('X [m]')
ax.set_ylabel('Y [m]')
ax.set_zlabel('Z [m]')
ax.legend()
ax.set_box_aspect([1, 1, 1])

# Construir el vector de tiempo para la trayectoria completa.
# Cada segmento se asume que dura T_segment segundos.
num_segments = len(q_traj_segments)
time_total = T_segment * num_segments
t_vec = np.linspace(0, time_total, q_traj.shape[0])

# Calcular velocidades y aceleraciones de las articulaciones usando diferencias finitas.
qd_traj = np.gradient(q_traj, t_vec[1]-t_vec[0], axis=0)#Derivada de posi
qdd_traj = np.gradient(qd_traj, t_vec[1]-t_vec[0], axis=0)#Derivada de vel

# Plot de Ángulo de Articulaciones vs Tiempo
fig_joint = plt.figure("Ángulo vs Tiempo")
for i in range(q_traj.shape[1]):
    plt.plot(t_vec, q_traj[:, i], label=f'J {i+1}')
plt.xlabel("Tiempo (s)")
plt.ylabel("Ángulo (rad)")
plt.title("Ángulo de Articulaciones vs Tiempo")
plt.legend()
plt.grid(True)

# Plot de Velocidades de Articulaciones vs Tiempo
fig_vel = plt.figure("Velocidades vs Tiempo")
for i in range(qd_traj.shape[1]):
    plt.plot(t_vec, qd_traj[:, i], label=f'J {i+1}')
plt.xlabel("Tiempo (s)")
plt.ylabel("Velocidad (rad/s)")
plt.title("Velocidades de Articulaciones vs Tiempo")
plt.legend()
plt.grid(True)

# Plot de Aceleraciones de Articulaciones vs Tiempo
fig_acc = plt.figure("Aceleraciones vs Tiempo")
for i in range(qdd_traj.shape[1]):
    plt.plot(t_vec, qdd_traj[:, i], label=f'J {i+1}')
plt.xlabel("Tiempo (s)")
plt.ylabel("Aceleración (rad/s²)")
plt.title("Aceleraciones de Articulaciones vs Tiempo")
plt.legend()
plt.grid(True)

# Animacion del robot utilizando la trayectoria en espacio articular
robot.plot(q_traj, block=False, loop=True)
plt.show()