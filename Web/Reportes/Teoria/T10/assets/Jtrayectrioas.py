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


# -------------------------
# Creacion de matriz de Rotacion para la orientacion de las coordenadas
# -------------------------
'''
theta = np.pi / 4
R = np.array([[np.cos(theta), -np.sin(theta), 0],
              [np.sin(theta), np.cos(theta) , 0],
              [0            , 0             ,-1]])
'''
R = np.array([[1, 0, 0],
              [0, 1, 0],
              [0, 0, 1]])
# Utilizando SE3 se crea un serie de coordenadas, dada la matriz de rotacion y vector de posicion
coordenadas = [SE3.Rt(R, [x, y, z]) for x, y, z in zip(P[0], P[1], P[2])]

print("\n Coordenadas dentro de espacio de trabajo:")
for i, pos in enumerate(coordenadas):
    print(f"WP{i+1}: {pos}")
'''
-------------------------
    2. Resolver la Cinematica inversa de cada coordenada
-------------------------'
'''
q_sol = []                            # Lista para guardar soluciones de cada articulacion
q_actual = np.zeros(robot.n)         # Suposicion inicial (0,0,0,0,0,0)
ik_init = time.time()

for i, pos in enumerate(coordenadas):
    # Usando LM o el metodo de IK preferente se busca la solucion
    sol = robot.ikine_LM(pos, q0=q_actual, mask=(1,1,1,0,0,0) )
    if sol.success:
        q_sol.append(sol.q)
        q_actual = sol.q             # update initial guess for next IK solve
        print(f"IK para coord{i+1} resuelta: {sol.q}")
    else:
        print(f"IK fallo para coord: {i+1}")
        
ik_end = time.time()
print("\nTiempo total de calculo de IK: {:.4f} segs".format(ik_end - ik_init))
'''
-------------------------
    3. Generar la trayectoria para movimiento por articulacion
-------------------------
'''
tray_segment = []  # Lista de trayectorias finales
T_segment = 5      # Duracion de cada segmento en segundo
n_points = 50      # Numero de puntos por trayectoria

# -----------------------
# Las funciones de trayectorias requieren un vecotr de tiempo para interpolar
# las velocidades, recordemos que las velocidades no son constantes a lo largo
# de mis movimientos entonces se le da el tiempo deseado para que haga el computo
# necesario para generar la trayectoria
# -----------------------
for i in range(len(q_sol) - 1):
    # Genera un vector de tiempo que va desde 0 hasta T_segment (5 segundos), dividido en 50 puntos.
    t_segment = np.linspace(0, T_segment, n_points) 
    # Genera la trayectoria de la solucion actual a la siguiente, interpolando con el vector de tiempo
    traj_seg = jtraj(q_sol[i], q_sol[i+1], t_segment)
    #Se agrega la trayectoria generada la lista
    tray_segment.append(traj_seg.q)

# Se repite la generacion para regresar a la coordenada 1
t_segment = np.linspace(0, T_segment, n_points)
traj_return = jtraj(q_sol[-1], q_sol[0], t_segment)
tray_segment.append(traj_return.q)

# Apila verticalmente las trayectorias
q_traj = np.vstack(tray_segment)
'''
-------------------------
    4. Evaluamos la trayectoria generada usando FK
-------------------------
'''
task_traj = []
for q in q_traj:
    T_fk = robot.fkine(q)
    task_traj.append(T_fk.t)    # Extraemos la traslacion
task_traj = np.array(task_traj)

# Creamos una trayectoria ideal, usando el vector de posicion y cerramos agregando la pos1 de nuevo
P_closed = np.hstack((P, P[:, 0:1]))
'''
-------------------------
    5. Hacemos los plots, para las trayectorias ideal vs real y animamos el robot
-------------------------
'''
# Creamos la ventana del plot de trayectorias
traj_fig = plt.figure("Trayectorias")
ax = traj_fig.add_subplot(111, projection='3d')
ax.plot(task_traj[:, 0], task_traj[:, 1], task_traj[:, 2],
        'b-', label='Trayectoria J')
ax.plot(P_closed[0], P_closed[1], P_closed[2],
        'r:', linewidth=2, label='Trayectoria Ideal')
ax.scatter(P[0], P[1], P[2],
           color='red', marker='o', s=50, label='Coordenadas')
for i in range(P.shape[1]):
    ax.text(P[0, i], P[1, i], P[2, i], f' WP{i+1}', 
            color='black', fontsize=12)

ax.set_title('Trayectoria Espacio Articular')
ax.set_xlabel('X [m]')
ax.set_ylabel('Y [m]')
ax.set_zlabel('Z [m]')
ax.legend()

# Creamos la animacion del robot en otra ventana
robot.plot(q_traj, block=False, loop=True)
plt.show()
