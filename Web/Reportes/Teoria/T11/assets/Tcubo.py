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
# Definir las dimensiones del cubo
lado_cubo = 2 * 0.1 / np.sqrt(3)  # Usando el radio maximo

# Definir los vértices del cubo
vertices_cubo = [
    [-lado_cubo / 2, -lado_cubo / 2, -lado_cubo / 2], # 1
    [lado_cubo / 2, -lado_cubo / 2, -lado_cubo / 2],  # 2
    [lado_cubo / 2, lado_cubo / 2, -lado_cubo / 2],   # 3
    [-lado_cubo / 2, lado_cubo / 2, -lado_cubo / 2],  # 4
    [-lado_cubo / 2, lado_cubo / 2, lado_cubo / 2],   # 5
    [lado_cubo / 2, lado_cubo / 2, lado_cubo / 2],    # 6
    [lado_cubo / 2, -lado_cubo / 2, lado_cubo / 2],   # 7
    [-lado_cubo / 2, -lado_cubo / 2, lado_cubo / 2],  # 8
    # Puntos para completar aristas del cubo
    [-lado_cubo / 2, lado_cubo / 2, lado_cubo / 2],   # 5
    [lado_cubo / 2, lado_cubo / 2, lado_cubo / 2],    # 6
    [lado_cubo / 2, lado_cubo / 2, -lado_cubo / 2],   # 3
    [-lado_cubo / 2, lado_cubo / 2, -lado_cubo / 2],  # 4
    [-lado_cubo / 2, -lado_cubo / 2, -lado_cubo / 2], # 1
    [-lado_cubo / 2, -lado_cubo / 2, lado_cubo / 2],  # 8
    [lado_cubo / 2, -lado_cubo / 2, lado_cubo / 2],   # 7
    [lado_cubo / 2, -lado_cubo / 2, -lado_cubo / 2]   # 2
]

# Inicializar P con el primer vértice del cubo
P = np.array(vertices_cubo[0]).reshape(3, 1)  # Primer punto del cubo

# Concatenar los vértices restantes
for v in vertices_cubo[1:]:  # Comienza desde el segundo vértice
    new_col = np.array(v).reshape(3, 1)  # Convertir a columna 3x1
    P = np.concatenate((P, new_col), axis=1)  # Concatenar en el eje 1 (columnas)

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
    print(f"WP{i+1}:lineal - {es_lineal}, articulado - {es_articular}")

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