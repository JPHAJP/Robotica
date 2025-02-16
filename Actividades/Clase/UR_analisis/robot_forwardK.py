import roboticstoolbox as rtb
from spatialmath import SE3
import numpy as np
import matplotlib.pyplot as plt

Status_block = False

# Robot UR5 --------------------------------------------------------------
name = "UR5"
a1 = 0.1625
a2 = 0.425
a3 = 0.3922
a4 = 0.1333
a5 = 0.0997
a6 = 0.0996

#https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/
#https://www.universal-robots.com/media/1829771/kinematicuroverview.png?width=408.94854586129753&height=800


# Define the articulated robot with 6 revolute joints
robot = rtb.DHRobot(
    [
        rtb.RevoluteDH( alpha=np.pi/2,      a=0,    d=a1,       offset=0),
        rtb.RevoluteDH( alpha=0,            a=-a2,   d=0,        offset=-np.pi/2),
        rtb.RevoluteDH( alpha=0,            a=-a3,   d=0,        offset=0),
        rtb.RevoluteDH( alpha=np.pi/2,      a=0,    d=a4,    offset=-np.pi/2),
        rtb.RevoluteDH( alpha=-np.pi/2,     a=0,    d=a5,        offset=0),
        rtb.RevoluteDH( alpha=0,            a=0,    d=a6,    offset=np.pi)
    ],
    name=name
)

print("Detalles del Robot: ", name)
print(robot)


# Forward kinematics ------------------------------------------------------
# Define the displacement values for each revolute joint
t_values = [0, 0, 0, 0, 0, 0]
d_values = [0, 0, 0, 0, 0, 0]  # Example: [X, Y, Z] displacements in meters
q_values = [t_values[0], t_values[1], t_values[2], t_values[3], t_values[4], t_values[5]]

print("Matriz de transformación (Directa):")
T = robot.fkine(q_values)  # Forward kinematics
print(T)


# Inverse kinematics ------------------------------------------------------
#Definir la pose deseada del efector final (posición y orientación)
Td = SE3(0.5, 0.2, 0.3) * SE3.RPY([0, np.pi/2, 0], order="xyz")

print("Matriz de transformación (Inversa):")
print(Td)
# Método 1: Levenberg-Marquardt (Numérico)
sol_LM = robot.ikine_LM(Td)
print("Levenberg-Marquardt (ikine_LM):", sol_LM.q)

# Método 2: Gauss-Newton (Numérico)
sol_GN = robot.ikine_GN(Td)
print("Gauss-Newton (ikine_GN):", sol_GN.q)

# Método 3: Newton-Raphson (Jacobiano)
sol_NR = robot.ikine_NR(Td)
print("Newton-Raphson (ikine_NR):", sol_NR.q)

# Método: Solución analítica
try:
    
    sol_a = robot.ikine_a(Td)
    print("Analítica (ikine_a):", sol_a.q)
except:
    print("No se puede calcular la solución analítica")

# --- Validar con Cinemática Directa ---
print("\n--- Validación con Cinemática Directa ---")
print("FK con LM:\n", robot.fkine(sol_LM.q))
print("FK con GN:\n", robot.fkine(sol_GN.q))
print("FK con NR:\n", robot.fkine(sol_NR.q))

robot.plot(q_values, block=Status_block, jointaxes=True, eeframe=True, jointlabels=True)
plt.savefig(f"Actividades/Clase/UR_analisis/FK {name}.png", dpi=600, bbox_inches='tight',  pad_inches=0.1)