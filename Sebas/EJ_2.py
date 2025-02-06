import numpy as np
import roboticstoolbox as rtb
from roboticstoolbox import DHRobot, RevoluteDH, PrismaticDH
from spatialmath import SE3

# =============================================================================
# Definición de constantes para las dimensiones del robot
# IMPORTANTE: Modificar estos valores para ajustar el tamaño del robot, las unidades son metros y radianes.
# =============================================================================
a1 = 0.1  # Desplazamiento para la articulación prismática
a2 = 0.05 # Longitud del eslabón para la articulación revoluta
a3 = 0.1  # Desplazamiento adicional para la articulación prismática

# =============================================================================
# https://petercorke.github.io/robotics-toolbox-python/arm_dh.html
#
# Creación del modelo del robot utilizando parámetros DH
#
# En este ejemplo definimos un robot con dos articulaciones:
#   1. Articulación revoluta:
#       - 'alpha' se establece en π/2, que define el ángulo de torsión entre eslabones.
#       - 'a' es la longitud del eslabón (se utiliza a2).
#       - 'd' es el desplazamiento del eslabón (en este caso 0).
#       - 'theta' es el ángulo de rotación de la articulación (en radianes).
#       - 'offset' se utiliza para definir la rotación constante alrededor del eje.
#      - 'qlim' es una lista que define los límites de movimiento de la articulación, en este caso de 0 a π radianes.
#
#   2. Articulación prismática:
#       - 'offset' se utiliza para definir el desplazamiento constante a lo largo del eje.
#       - 'qlim' es una lista que define los límites de movimiento de la articulación, en este caso de 0 a 0.2 metros.
#   
# El robot se construye utilizando la clase DHRobot, la cual recibe una lista de objetos 
# que definen cada articulación.
# =============================================================================
robot = DHRobot(
    [
        RevoluteDH(alpha=np.pi/2, a=a2, d=0, offset=np.pi/2, qlim=[0, np.pi]),
        PrismaticDH(offset=a1 + a3 , qlim=[0, 0.2])
    ],
    name="Robot_Simple"
)

# =============================================================================
# Configuración de la transformación de la base del robot.
#
# La base del robot se puede modificar utilizando una transformación SE3.
# En este ejemplo, se rota la base del robot 90 grados alrededor del eje x.
# =============================================================================

#robot.base = SE3.Ry(np.deg2rad(90))
#robot.base = SE3(0, 0, 0.5)  
#robot.base = SE3(0, 0, 0.5) * SE3.Rx(np.pi/2)

# =============================================================================
# Impresión de los detalles del robot
#
# Se imprime un resumen del robot que incluye las articulaciones, parámetros DH y nombre.
# =============================================================================
print("Detalles del Robot:")
print(robot)

# =============================================================================
# Definir una configuración articular
#
# El vector de configuración 'q' contiene:
#   - q[0]: el ángulo (en radianes) para la articulación revoluta.
#   - q[1]: el desplazamiento para la articulación prismática.
# =============================================================================
th1=np.deg2rad(0)
d1=0
q = [th1, d1]

# =============================================================================
# Cálculo de la cinemática directa (FK)
#
# El método 'fkine' calcula la matriz de transformación desde la base hasta el efector final.
# La variable 'T' es un objeto SE3 que representa la posición y orientación del efector final.
# La variable 'T_all' es una lista de objetos SE3 que representan la posición y orientación de todas las articulaciones.
# La variable 'T_joint1' es un objeto SE3 que representa la posición y orientación de la articulación 1.
# =============================================================================
T = robot.fkine(q) 
print("\nCinemática Directa (Matriz de transformación):")
print(T)
T_all = robot.fkine_all(q)
T_joint1 = T_all[0]  
print("\nOrientacion de articulacion 1:")
print(T_joint1.R)
print("\nTraslacion de articulacion 1:")
print(T_joint1.t)

# =============================================================================
# Visualización del robot en la configuración 'q'
#
# El método 'plot' muestra el robot. El argumento 'block=True' mantiene la ventana 
# de la gráfica abierta hasta que se cierre, y 'jointlabels=True' muestra las etiquetas 
# de las articulaciones.
# =============================================================================
robot.plot(q, block=True, jointlabels=True, jointaxes=True)