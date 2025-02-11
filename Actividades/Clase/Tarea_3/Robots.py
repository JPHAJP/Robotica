#Codigo para ejecutar la simulación de los 5 robots

from roboticstoolbox import DHRobot, PrismaticDH, RevoluteDH
from spatialmath import SE3
import numpy as np
import matplotlib.pyplot as plt

Status_block = False

# Robot 1 --------------------------------------------------------------
name = "1_Planar"
# Define the planar robot with 1 revolute joint and 1 prismatic joint
a1 = 0.1  # Desplazamiento para la articulación prismática
a2 = 0.05 # Longitud del eslabón para la articulación revoluta
a3 = 0.1  # Desplazamiento adicional para la articulación prismática

robot = DHRobot(
    [
        RevoluteDH( alpha=np.pi/2, a=a2, d=0, offset=np.pi/2),
        PrismaticDH(                          offset=a1 + a3 )
    ],
    name=name
)
print("Detalles del Robot: ", name)
print(robot)

# Define the displacement values for each prismatic joint
t_values = [np.radians(0)]
d_values = [0.1]  # Example: [X, Y, Z] displacements in meters
q_values = [t_values[0], d_values[0]]

print("Matriz de transformación (Directa):")
T = robot.fkine(q_values)  # Forward kinematics
print(T)
robot.plot(q_values, block=Status_block, jointaxes=True, eeframe=True, jointlabels=True)
plt.savefig(f"Actividades/Clase/Tarea_3/CF {name}.png", dpi=600, bbox_inches='tight',  pad_inches=0.1)


# Robot 2 --------------------------------------------------------------
name = "2_Cartesiano"
a1 = 0.1  # Desplazamiento para la articulación prismática
a2 = 0.1  # Desplazamiento para la articulación prismática
a3 = 0.1  # Desplazamiento para la articulación prismática
# Define the Cartesian robot with 3 prismatic joints
robot = DHRobot(
    [
        PrismaticDH(offset=a1,    alpha=-np.pi/2,  a=0, theta=np.pi/2),
        PrismaticDH(offset=a2,    alpha=np.pi/2,   a=0, theta=-np.pi/2),
        PrismaticDH(offset=a3,    alpha=0,         a=0, theta=0)
 
    ],
    base = SE3(0, 0, 0),   # shift the entire robot 0.5m along Z
    name=name
)
print("Detalles del Robot: ", name)
print(robot)

# Define the displacement values for each prismatic joint
t_values = [0] 
d_values = [0.1, 0.1, 0.1]  # Example: [X, Y, Z] displacements in meters
q_values = [d_values[0], d_values[1], d_values[2]]

print("Matriz de transformación (Directa):")
T = robot.fkine(q_values)  # Forward kinematics
print(T)

robot.plot(q_values, block=Status_block, jointaxes=True, eeframe=True, jointlabels=True)
plt.savefig(f"Actividades/Clase/Tarea_3/CF {name}.png", dpi=600, bbox_inches='tight',  pad_inches=0.1)



# Robot 3 --------------------------------------------------------------
name = "3_Articulado"
a1 = 0.1  
a2 = 0.1
a3 = 0.1
a4 = 0.1
a5 = 0.1
a6 = 0.1
# longitud del eslabón para la articulación revoluta

# Define the articulated robot with 6 revolute joints
robot = DHRobot(
    [
        RevoluteDH( alpha=-np.pi/2,     a=0,    d=a1,       offset=0),
        RevoluteDH( alpha=0,            a=a2,   d=0,        offset=-np.pi/2),
        RevoluteDH( alpha=-np.pi/2,     a=0,    d=0,        offset=0),
        RevoluteDH( alpha=np.pi/2,      a=0,    d=a3+a4,    offset=np.pi/2),
        RevoluteDH( alpha=-np.pi/2,     a=0,    d=0,        offset=0),
        RevoluteDH( alpha=0,            a=0,    d=a5+a6,    offset=0)
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
plt.savefig(f"Actividades/Clase/Tarea_3/CF {name}.png", dpi=600, bbox_inches='tight',  pad_inches=0.1)


# Robot 4 --------------------------------------------------------------
name = "4_Articulado"
a1 = 0.2  
a2 = 0.1
a3 = 0.1
a4 = 0.1
a5 = 0.1
a6 = 0.1

# Define the articulated robot with 6 revolute joints
robot = DHRobot(
    [
        RevoluteDH( alpha=-np.pi/2,     a=0,    d=a1,       offset=0),
        RevoluteDH( alpha=-np.pi/2,     a=a2,   d=0,        offset=0),
        RevoluteDH( alpha=-np.pi/2,     a=0,    d=0,        offset=0),
        RevoluteDH( alpha=np.pi/2,      a=0,    d=a3+a4,    offset=0),
        RevoluteDH( alpha=-np.pi/2,     a=0,    d=0,        offset=0),
        RevoluteDH( alpha=0,            a=0,    d=a5+a6,    offset=0)
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
plt.savefig(f"Actividades/Clase/Tarea_3/CF {name}.png", dpi=600, bbox_inches='tight',  pad_inches=0.1)


# Robot 5 --------------------------------------------------------------
name = "5_Articulado"
a1 = 0.1
a2 = 0.1
a3 = 0.1
a4 = 0.1
a5 = 0.1
a6 = 0.1

# Define the articulated robot with 6 revolute joints
robot = DHRobot(
    [
        RevoluteDH( alpha=-np.pi/2,     a=0,    d=a1,       offset=0),
        RevoluteDH( alpha=0,            a=a2,   d=0,        offset=np.pi/2),
        RevoluteDH( alpha=np.pi/2,      a=0,    d=0,        offset=0),
        RevoluteDH( alpha=-np.pi/2,     a=0,    d=a3+a4,    offset=0),
        RevoluteDH( alpha=np.pi/2,      a=0,    d=0,        offset=0),
        RevoluteDH( alpha=0,            a=0,    d=a5+a6,    offset=0)
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
plt.savefig(f"Actividades/Clase/Tarea_3/CF {name}.png", dpi=600, bbox_inches='tight',  pad_inches=0.1)

