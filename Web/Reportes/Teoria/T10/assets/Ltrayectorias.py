import numpy as np
import time
import matplotlib.pyplot as plt
from spatialmath import SE3
import roboticstoolbox as rtb
from roboticstoolbox import jtraj, ctraj

def inverse_kinematics(robot, Td, q0, ilimit, mask, save_path, name="IK_Solution", status_block=False):
    print("Matriz de transformación (Inversa):")
    print(Td)

    # Método 1: Levenberg-Marquardt (Numérico)
    sol_LM = robot.ikine_LM(Td, q0=q0, ilimit=ilimit, slimit=ilimit, mask=mask, )
    print("Levenberg-Marquardt (ikine_LM):", sol_LM)
    if sol_LM.success:
        print("IK LM (Grados): ", np.rad2deg(sol_LM.q))
        print("FK con NR:\n", robot.fkine(sol_LM.q))
        robot.plot(sol_LM.q, block=status_block, jointaxes=True, eeframe=True, jointlabels=True)
        plt.savefig(f"{save_path}/IK_LM_{name}.png", dpi=600, bbox_inches='tight', pad_inches=0.1)
    else:
        print("IK LM: No se encontró solución")

    print("\nResumen de success:")
    print("Levenberg-Marquardt:", sol_LM.success)

    return {"Levenberg-Marquardt": sol_LM}

def forward_kinematics(robot, q0, save_path, name="FK_Solution", status_block=False):
    """Calcula la cinemática directa con Denavit-Hartenberg de un robot"""
    T = robot.fkine(q0)  # Forward kinematics
    print("Matriz de transformación (Directa):")
    print(T)
    try:
        robot.plot(q0, block=status_block, jointaxes=True, eeframe=True, jointlabels=True)
        plt.savefig(f"{save_path}/FK_{name}.png", dpi=600, bbox_inches='tight', pad_inches=0.1)
    except Exception as e:
        print("Error al graficar la cinemática directa:", e)

#---------------------------
#    1. Definicion del robot
#---------------------------
a1 = 0.02
a2 = 0.10
a3 = 0.08

save_path = "Web/Reportes/Teoria/T10/assets"
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

# Define the displacement values for each revolute joint
t_values = [None, None, None, None, None]
t_values[0] = [0, 0, 0]
t_values[1] = [0, np.pi/4, 0]
t_values[2] = [np.pi/4, np.pi/4, 0]
t_values[3] = [np.pi/2, np.pi/4, 0]
t_values[4] = [0, np.pi/4, np.pi/2]
q_values = np.array(t_values).T
print("Joint values:")
print(q_values)

listaXYZ = []

# Ahora, itera sobre los valores de las articulaciones y calcula la cinemática directa (FK)
for i, q in enumerate(q_values.T):  # q_values.T transpone la matriz de q_values para iterar por columnas
    print(f"\nCalculando FK para el conjunto de posiciones {i + 1}:")
    forward_kinematics(robot, q, save_path, name=f"{name}_Pos_{i + 1}", status_block=False)

print("\n\n\n")
#Imprimir los puntos de la trayectoria
print("Puntos de la trayectoria:")
for i, q in enumerate(q_values.T):
    x, y, z = robot.fkine(q).t
    print(f"Posición {i + 1}: ({x:.3f}, {y:.3f}, {z:.3f})")
    # Guardar los puntos de la trayectoria
    listaXYZ.append([x, y, z])

#print(listaXYZ)
#Convertir lista a array
listaXYZ = np.array(np.round(listaXYZ, 3)).T
print(listaXYZ)

P= listaXYZ
# Matriz de rotacion para la orientacion 
R = np.array([[1, 0, 0],
              [0, 1, 0],
              [0, 0, 1]])


# Se crea una lista de objetos SE3 que representan los waypoints
coordenadas = [SE3.Rt(R, [x, y, z]) for x, y, z in zip(P[0], P[1], P[2])]

print("\nCoordenadas en el espacio de trabajo:")
for i, pos in enumerate(coordenadas):
    print(f"WP{i+1}: {pos}")

'''
---------------------------
    3. Generar la trayectoria usando ctraj (trayectoria cartesiana)
---------------------------
'''
tray_segment = []  # Lista para guardar trayectorias por segmento
T_segment = 5      # Duracion (conceptual) de cada segmento (no se usa directamente en ctraj)
n_points = 50      # Numero de puntos por segmento

# Usamos ctraj para generar trayectorias en el espacio cartesiano y resolvemos IK para cada pose.
q_traj_segments = []  # Lista para almacenar trayectorias en espacio articular
q_current = np.zeros(robot.n)  # Suposicion inicial para IK

ik_start = time.time()

# Para cada segmento entre waypoints consecutivos
for i in range(len(coordenadas) - 1):
    # Genera una trayectoria cartesiana entre dos poses
    cart_traj = ctraj(coordenadas[i], coordenadas[i+1], n_points)
    q_traj_seg = []  # Para guardar la trayectoria articular de este segmento
    for pose in cart_traj:
        sol = robot.ikine_LM(pose, q0=q_current, mask=(1,1,1,0,0,0))
        if sol.success:
            q_traj_seg.append(sol.q)
            q_current = sol.q  # Actualiza la condicion inicial para el siguiente paso
        else:
            print(f"IK fallo en el segmento {i+1} para una pose intermedia")
    q_traj_segments.append(np.array(q_traj_seg))

# Agregar trayectoria de retorno de la ultima pose al primer waypoint
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

# Concatena todas las trayectorias en una sola secuencia
q_traj = np.vstack(q_traj_segments)

'''
---------------------------
    4. Evaluamos la trayectoria generada usando FK
---------------------------
'''
task_traj = []
for q in q_traj:
    T_fk = robot.fkine(q)
    task_traj.append(T_fk.t)  # Extraemos la posicion (traslacion)
task_traj = np.array(task_traj)

# Para comparacion, se cierra la trayectoria ideal agregando el primer waypoint al final
P_closed = np.hstack((P, P[:, 0:1]))

'''
---------------------------
    5. Hacemos los plots, para las trayectorias ideal vs real y animamos el robot
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
for i in range(P.shape[1]):
    ax.text(P[0, i], P[1, i], P[2, i], f' WP{i+1}', 
            color='black', fontsize=12)

ax.set_title('Trayectoria en el Espacio de Trabajo')
ax.set_xlabel('X [m]')
ax.set_ylabel('Y [m]')
ax.set_zlabel('Z [m]')
ax.legend()

# ---------------
# Grafica de Angulo contra tiempo por articulacion
# ---------------
tiempo_total = len(q_traj_segments) * T_segment  # Se usa el número de segmentos reales
eje_t = np.linspace(0, tiempo_total, q_traj.shape[0]) 

fig_joint = plt.figure("Angulo Articulacion vs Tiempo")
for i in range(q_traj.shape[1]):
    plt.plot(eje_t, q_traj[:, i], label=f'J {i+1}')
plt.xlabel("Tiempo (s)")
plt.ylabel("Ángulo (rad)")
plt.title("Ángulo vs Tiempo")
plt.legend()
plt.grid(True)

# Animacion del robot utilizando la trayectoria en espacio articular
robot.plot(q_traj, block=False, loop=True)
plt.show()
    