import numpy as np
import roboticstoolbox as rtb
from roboticstoolbox import DHRobot, PrismaticDH
from spatialmath import SE3
import matplotlib.pyplot as plt
 
# =============================================================================
# Definición de constantes para las dimensiones del robot
# =============================================================================
a1 = 0.2  # Desplazamiento piston 1
a2 = 0.2 # Desplazamiento piston 2
a3 = 0.2  # Desplazamiento piston 3
a4 = 0.2  # Desplazamiento piston 4
 
# =============================================================================
# Creación del modelo del robot tipo cartesiano utilizando parámetros DH
# =============================================================================
robot = DHRobot(
    [
        PrismaticDH(alpha=np.pi/2, offset=a1, theta=np.pi/2, qlim=[0, 0.2]),
        PrismaticDH(alpha=np.pi/2, offset=a2, theta=np.pi/2, qlim=[0, 0.2]),
        PrismaticDH(alpha=np.pi/2, offset=a3, qlim=[0, 0.2]),
        PrismaticDH(alpha=np.pi/2, offset=a4, qlim=[0, 0.2])
    ],
    name="Robot_Cartesiano"
)
 
# =============================================================================
# Impresión de los detalles del robot
# =============================================================================
print("Detalles del Robot:")
print(robot)
 
# =============================================================================
# Definir una configuración articular
# =============================================================================
d1 = 0.1
d2 = 0.1
d3 = 0.1
d4 = 0.1
q = [d1, d2, d3, d4]
 
# =============================================================================
# Cálculo de la cinemática directa (FK)
# =============================================================================
T = robot.fkine(q)
print("\nCinemática Directa (Matriz de transformación):")
print(T)
 
T_all = robot.fkine_all(q)
T_joint1 = T_all[0]  
print("\nOrientación de articulación 1:")
print(T_joint1.R)
print("\nTraslación de articulación 1:")
print(T_joint1.t)
 
# =============================================================================
# Visualización del robot en la configuración 'q'
# =============================================================================
robot.plot(q, block=False, jointlabels=True, jointaxes=True)
plt.savefig("Sebas/configuration.png", dpi=600, bbox_inches='tight',  pad_inches=0.1)