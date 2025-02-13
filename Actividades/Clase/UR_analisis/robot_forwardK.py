#Codigo para ejecutar la simulación de los 5 robots

from roboticstoolbox import DHRobot, PrismaticDH, RevoluteDH
from spatialmath import SE3
import numpy as np
import matplotlib.pyplot as plt

Status_block = True

# Robot UR5 --------------------------------------------------------------
name = "UR5"
a1 = 0.1625
a2 = -0.425
a3 = -0.3922
a4 = 0.1333
a5 = 0.0997
a6 = 0.0996

#https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/
#https://www.universal-robots.com/media/1829771/kinematicuroverview.png?width=408.94854586129753&height=800


# Define the articulated robot with 6 revolute joints
robot = DHRobot(
    [
        RevoluteDH( alpha=np.pi/2,      a=0,    d=a1,       offset=0),
        RevoluteDH( alpha=0,            a=a2,   d=0,        offset=-np.pi/2),
        RevoluteDH( alpha=0,            a=a3,   d=0,        offset=0),
        RevoluteDH( alpha=np.pi/2,      a=0,    d=a4,    offset=-np.pi/2),
        RevoluteDH( alpha=-np.pi/2,     a=0,    d=a5,        offset=0),
        RevoluteDH( alpha=0,            a=0,    d=a6,    offset=np.pi)
    ],
    name=name
)

print("Detalles del Robot: ", name)
print(robot)

# Define the displacement values for each revolute joint
t_values = [0, 0, 0, 0, 0, 0]
d_values = [0, 0, 0, 0, 0, 0]  # Example: [X, Y, Z] displacements in meters
q_values = [t_values[0], t_values[1], t_values[2], t_values[3], t_values[4], t_values[5]]

print("Matriz de transformación (Directa):")
T = robot.fkine(q_values)  # Forward kinematics
print(T)

robot.plot(q_values, block=Status_block, jointaxes=True, eeframe=True, jointlabels=True)
plt.savefig(f"Actividades/Clase/Tarea_3/FK {name}.png", dpi=600, bbox_inches='tight',  pad_inches=0.1)