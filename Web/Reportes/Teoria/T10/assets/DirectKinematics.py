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
        rtb.RevoluteDH(alpha=-np.pi/2,  a=0,    d=a1,  offset=0,  qlim=(-np.pi, np.pi)),
        rtb.RevoluteDH(alpha=0,         a=a2,   d=0,   offset=0,  qlim=(-np.pi, np.pi)),
        rtb.RevoluteDH(alpha=0,         a=a3,   d=0,   offset=0,  qlim=(-np.pi, np.pi))
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