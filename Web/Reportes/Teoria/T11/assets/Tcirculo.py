#Para trayectoria de un círculo

import numpy as np
import time
import matplotlib.pyplot as plt
from spatialmath import SE3
import roboticstoolbox as rtb
from roboticstoolbox import jtraj, ctraj

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
P = np.array([[0.001],                              # X
              [0.001 * np.cos(np.deg2rad(0))],        # Y
              [0.001 * np.sin(np.deg2rad(0)) + 0.10]])  # Z

# Genera waypoints a lo largo de una circunferencia (179 puntos adicionales cada 2 grados)
for i in range(179):
    new_col = np.array([0.001, 
                        0.001 * np.cos(np.deg2rad(i*2)), 
                        0.001 * np.sin(np.deg2rad(i*2)) + 0.10]).reshape(3, 1)
    P = np.concatenate((P, new_col), axis=1)

#########################################
#        Revisar espacio de trabajo
#########################################
# Función para verificar si un punto es verde o azul
def es_punto_lineal_o_articulado(x, y, z, radio_max, apertura_cono):
    # El cubo debe estar dentro de la esfera, y su diagonal debe ser <= al diámetro de la esfera
    lado_cubo = 2 * radio_max / np.sqrt(3)  # Calcular el lado del cubo
    
    # Comprobación del cubo: El punto debe estar dentro de los límites del cubo
    dentro_del_cubo = abs(x) <= lado_cubo/2 and abs(y) <= lado_cubo/2 and abs(z) <= lado_cubo/2
    
    # Comprobación del cono: El punto debe estar fuera del cono (se calcula el ángulo)
    angulo_cono = np.arctan2(np.sqrt(x**2 + y**2), -z)  # Ángulo con respecto al eje Z negativo
    fuera_del_cono = angulo_cono > apertura_cono

    # Verificar si el punto es verde
    es_lineal = dentro_del_cubo and fuera_del_cono
    
    # Verificar si el punto es azul
    # El punto debe estar dentro de la esfera y fuera del cono
    distancia_al_origen = np.sqrt(x**2 + y**2 + z**2)
    es_articular = distancia_al_origen <= radio_max and fuera_del_cono

    return es_lineal, es_articular

# Revisar si los puntos de la trayectoria son lineales o articulados
radio_max = a2+a3  # Radio máximo del robot
apertura_cono = np.deg2rad(60)  # Ángulo de apertura del cono

# Verificar si es lineal o articulado
for i in range(P.shape[1]):
    x, y, z = P[:, i].flatten()  # Aplanar la columna para obtener x, y, z
    es_lineal, es_articular = es_punto_lineal_o_articulado(x, y, z, radio_max, apertura_cono)
    print(f"WP{i+1}:lineal {es_lineal},articulado {es_articular}")


# Matriz de rotacion para la orientacion
R = np.array([[1, 0, 0],
              [0, 1, 0],
              [0, 0, 1]])

# Se crea una lista de objetos SE3 para cada waypoint
coordenadas = [SE3.Rt(R, [x, y, z]) for x, y, z in zip(P[0], P[1], P[2])]

# print("\nCoordenadas en el espacio de trabajo:")
# for i, pos in enumerate(coordenadas):
#     print(f"WP{i+1}: {pos}")

'''
---------------------------
    3. Resolver la Cinematica inversa de cada coordenada
---------------------------
'''
#region
q_sol = []                           
q_actual = np.zeros(robot.n)         
ik_init = time.time()

for i, pos in enumerate(coordenadas):
    sol = robot.ikine_LM(pos, q0=q_actual, mask=(1,1,1,0,0,0))
    if sol.success:
        q_sol.append(sol.q)
        q_actual = sol.q
        print(f"IK para coord{i+1} resuelta: {sol.q}")
    else:
        print(f"IK fallo para coord: {i+1}")

ik_end = time.time()
print("\nTiempo total de calculo de IK: {:.4f} segs".format(ik_end - ik_init))
#endregion
'''
---------------------------
    4. Generar la trayectoria para movimiento por articulacion
---------------------------
'''
#region
tray_segments = []  # Lista de posiciones
vel_segments = []   # Lista de velocidades
acc_segments = []   # Lista de aceleraciones
T_segment = 1       # Duracion de cada segmento en segs
n_points = 5       # Numero de puntos por segmento

# Generar trayectorias entre cada par de configuraciones
for i in range(len(q_sol) - 1):
    t_segment = np.linspace(0, T_segment, n_points)
    traj_seg = jtraj(q_sol[i], q_sol[i+1], t_segment)
    tray_segments.append(traj_seg.q)
    vel_segments.append(traj_seg.qd)
    acc_segments.append(traj_seg.qdd)

# Trayectoria de regreso al punto inicial
t_segment = np.linspace(0, T_segment, n_points)
traj_return = jtraj(q_sol[-1], q_sol[0], t_segment)
tray_segments.append(traj_return.q)
vel_segments.append(traj_return.qd)
acc_segments.append(traj_return.qdd)

# Apilar los segmentos en una trayectoria completa
q_traj  = np.vstack(tray_segments)
qd_traj = np.vstack(vel_segments)
qdd_traj = np.vstack(acc_segments)
#endregion
'''
---------------------------
    5. Evaluamos la trayectoria generada usando FK
---------------------------
'''
#region
task_traj = []
for q in q_traj:
    T_fk = robot.fkine(q)
    task_traj.append(T_fk.t)
task_traj = np.array(task_traj)
P_closed = np.hstack((P, P[:, 0:1]))
#endregion
'''
---------------------------
    6. Graficar Trayectorias y Angulos de Articulacion
---------------------------
'''
#region
# Plot de la trayectoria en el espacio
traj_fig = plt.figure("Trayectorias")
ax = traj_fig.add_subplot(111, projection='3d')
ax.plot(task_traj[:, 0], task_traj[:, 1], task_traj[:, 2],
        'b-', label='Trayectoria J')
ax.plot(P_closed[0], P_closed[1], P_closed[2],
        'r:', linewidth=2, label='Trayectoria Ideal')
ax.scatter(P[0], P[1], P[2],
           color='red', marker='o', s=50, label='Coordenadas')
# for i in range(P.shape[1]):
#     ax.text(P[0, i], P[1, i], P[2, i], f' WP{i+1}', color='black', fontsize=12)
ax.set_title('Trayectoria Espacio Articular')
ax.set_xlabel('X [m]')
ax.set_ylabel('Y [m]')
ax.set_zlabel('Z [m]')
ax.legend()
#endregion
'''
---------------------------
    7. Graficar Posicione, Velocidades y Aceleraciones
---------------------------
'''
#region
q_traj  = np.vstack(tray_segments)  # Posiciones
qd_traj = np.vstack(vel_segments)    # Velocidad
qdd_traj = np.vstack(acc_segments)   # Aceleraciones

tiempo_total = len(tray_segments) * T_segment  
eje_t = np.linspace(0, tiempo_total, q_traj.shape[0]) 

fig_joint = plt.figure("Ángulo vs Tiempo")
for i in range(q_traj.shape[1]):
    plt.plot(eje_t, q_traj[:, i], label=f'J {i+1}')
plt.xlabel("Tiempo (s)")
plt.ylabel("Ángulo (rad)")
plt.title("Ángulo vs Tiempo")
plt.legend()
plt.grid(True)


fig_vel = plt.figure("Velocidades vs Tiempo")
for i in range(qd_traj.shape[1]):
    plt.plot(eje_t, qd_traj[:, i], label=f'J {i+1}')
plt.xlabel("Tiempo (s)")
plt.ylabel("Velocidad (rad/s)")
plt.title("Velocidades de Articulaciones vs Tiempo")
plt.legend()
plt.grid(True)

fig_acc = plt.figure("Aceleraciones vs Tiempo")
for i in range(qdd_traj.shape[1]):
    plt.plot(eje_t, qdd_traj[:, i], label=f'J {i+1}')
plt.xlabel("Tiempo (s)")
plt.ylabel("Aceleración (rad/s²)")
plt.title("Aceleraciones de Articulaciones vs Tiempo")
plt.legend()
plt.grid(True)
#endregion
robot.plot(q_traj, block=False, loop=True)
plt.show()
