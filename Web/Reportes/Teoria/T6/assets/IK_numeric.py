import numpy as np
import matplotlib.pyplot as plt
import roboticstoolbox as rtb
from spatialmath import SE3
import numpy as np
import matplotlib.pyplot as plt

save_path = "Web/Reportes/Teoria/T6/assets"

def inverse_kinematics(robot, Td, q0, ilimit, mask, save_path, name="IK_Solution", status_block=False):
    """
    Calcula la cinemática inversa de un robot utilizando tres métodos:
    - Levenberg-Marquardt
    - Gauss-Newton
    - Newton-Raphson
    
    Args:
        robot: Robot de la librería Robotics Toolbox
        Td: Matriz de transformación deseada (SE3)
        q0: Configuración inicial de los ángulos articulares
        ilimit: Límites de iteraciones (opcional, depende del método)
        mask: Máscara de restricciones (opcional, depende del método)
        save_path: Ruta donde se guardarán las imágenes de los resultados
        name: Nombre para guardar los archivos de imagen
        status_block: Booleano para bloquear la visualización del gráfico
        
    Returns:
        dict: Diccionario con los resultados de los tres métodos
    """

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
    
    # Método 2: Gauss-Newton (Numérico)
    sol_GN = robot.ikine_GN(Td, q0=q0, ilimit=ilimit, slimit=ilimit, mask=mask)
    print("Gauss-Newton (ikine_GN):", sol_GN)
    if sol_LM.success:
        print("IK LM (Grados): ", np.rad2deg(sol_GN.q))
        print("FK con GN:\n", robot.fkine(sol_GN.q))
        robot.plot(sol_GN.q, block=status_block, jointaxes=True, eeframe=True, jointlabels=True)
        plt.savefig(f"{save_path}/IK_GN_{name}.png", dpi=600, bbox_inches='tight', pad_inches=0.1)
    else:
        print("IK LM: No se encontró solución")
    
    # Método 3: Newton-Raphson (Jacobiano)
    sol_NR = robot.ikine_NR(Td, q0=q0, ilimit=ilimit, slimit=ilimit, mask=mask)
    print("Newton-Raphson (ikine_NR):", sol_NR)
    if sol_LM.success:
        print("IK LM (Grados): ", np.rad2deg(sol_NR.q))
        print("FK con LM:\n", robot.fkine(sol_NR.q))
        robot.plot(sol_NR.q, block=status_block, jointaxes=True, eeframe=True, jointlabels=True)
        plt.savefig(f"{save_path}/IK_NR_{name}.png", dpi=600, bbox_inches='tight', pad_inches=0.1)
    else:
        print("IK LM: No se encontró solución")
    
    # Verificar cual es la mejor solución
    # # Filtrar la de menores pasos
    print("\nMenores iteraciones:")
    if sol_LM.success and sol_LM.iterations < sol_GN.iterations and sol_LM.iterations < sol_NR.iterations:
        print("Levenberg-Marquardt", sol_LM.iterations)
    elif sol_GN.success and sol_GN.iterations < sol_LM.iterations and sol_GN.iterations < sol_NR.iterations:
        print("Gauss-Newton", sol_GN.iterations)
    elif sol_NR.success:
        print("Newton-Raphson", sol_NR.iterations)
    else:
        print("No se encontró una solución válida")
        

    # Verificar menores errores
    print("\nMenor error:")
    if sol_LM.success and sol_LM.residual < sol_GN.residual and sol_LM.residual < sol_NR.residual:
        print("Levenberg-Marquardt", sol_LM.residual)
    elif sol_GN.success and sol_GN.residual < sol_LM.residual and sol_GN.residual < sol_NR.residual:
        print("Gauss-Newton", sol_GN.residual)
    elif sol_NR.success:
        print("Newton-Raphson", sol_NR.residual)
    else:
        print("No se encontró una solución válida")

    # Resumen de success
    print("\nResumen de success:")
    print("Levenberg-Marquardt:", sol_LM.success)
    print("Gauss-Newton:", sol_GN.success)
    print("Newton-Raphson:", sol_NR.success)

    return {
        "Levenberg-Marquardt": sol_LM,
        "Gauss-Newton": sol_GN,
        "Newton-Raphson": sol_NR
    }

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

# Robot UR5 --------------------------------------------------------------
name = "UR5"
# Definir los parámetros de Denavit-Hartenberg
a1 = 0.1625
a2 = 0.425
a3 = 0.3922
a4 = 0.1333
a5 = 0.0997
a6 = 0.0996
# Define the articulated robot with 6 revolute joints
robot = rtb.DHRobot(
    [
        rtb.RevoluteDH( alpha=np.pi/2,      a=0,    d=a1,       offset=0 , qlim=[-np.pi, np.pi]),
        rtb.RevoluteDH( alpha=0,            a=-a2,   d=0,        offset=-np.pi/2, qlim=[-np.pi, np.pi]),
        rtb.RevoluteDH( alpha=0,            a=-a3,   d=0,        offset=0, qlim=[-np.pi, np.pi]),
        rtb.RevoluteDH( alpha=np.pi/2,      a=0,    d=a4,    offset=-np.pi/2, qlim=[-np.pi, np.pi]),
        rtb.RevoluteDH( alpha=-np.pi/2,     a=0,    d=a5,        offset=0, qlim=[-np.pi, np.pi]),
        rtb.RevoluteDH( alpha=0,            a=0,    d=a6,    offset=np.pi, qlim=[-2*np.pi, 2*np.pi])
    ],
    name=name
)

print("Detalles del Robot: ", name)
print(robot)

# Define the displacement values for each revolute joint
t_values = [0, 0, 0, 0, 0, 0]
d_values = [0, 0, 0, 0, 0, 0]  # Example: [X, Y, Z] displacements in meters
q_values = [t_values[0], t_values[1], t_values[2], t_values[3], t_values[4], t_values[5]]

forward_kinematics(robot, q_values, save_path, name=name)

results =  inverse_kinematics(robot, 
                              SE3(0.5, 0.2, 0.3) * SE3.RPY([0, np.pi/2, 0], order="xyz"),
                              q0=[np.pi/2, np.pi/2, np.pi/2, np.pi/2, np.pi/2, np.pi/2],
                              ilimit=100, 
                              mask=[1, 1, 1, 1, 1, 1], 
                              save_path=save_path, name=name, status_block=False)



# Robot 1 --------------------------------------------------------------
name = "1_Planar"
# Define the planar robot with 1 revolute joint and 1 prismatic joint
a1 = 0.1  # Desplazamiento para la articulación prismática
a2 = 0.05 # Longitud del eslabón para la articulación revoluta
a3 = 0.1  # Desplazamiento adicional para la articulación prismática

robot = rtb.DHRobot(
    [
        rtb.RevoluteDH( alpha=np.pi/2, a=a2, d=0, offset=np.pi/2),
        rtb.PrismaticDH( qlim=[0, 0.2], offset=a1 + a3)  # Límite de la articulación prismática
    ],
    name=name
)

print("Detalles del Robot: ", name)
print(robot)

# Define the displacement values for each prismatic joint
t_values = [np.radians(45)]
d_values = [0.1]  # Example: [X, Y, Z] displacements in meters
q_values = [t_values[0], d_values[0]]

forward_kinematics(robot, q_values, save_path, name=name, status_block=False)
resutls = inverse_kinematics(robot,
                            Td = np.array([
                                        [-0.707, 0, 0.707, 0.176],
                                        [0.707, 0, 0.707, 0.245],
                                        [0, 1, 0, 0],
                                        [0, 0, 0, 1]
                                    ]),
                            q0=[0, 0],
                            ilimit=1000,
                            mask=[1, 1, 0, 1, 1, 0],
                            save_path=save_path, name=name, status_block=False)

# Robot 2 --------------------------------------------------------------
name = "2_Cartesiano"
a1 = 0.1  # Desplazamiento para la articulación prismática
a2 = 0.1  # Desplazamiento para la articulación prismática
a3 = 0.1  # Desplazamiento para la articulación prismática
# Define the Cartesian robot with 3 prismatic joints
robot = rtb.DHRobot(
    [
        rtb.PrismaticDH(offset=a1,    alpha=-np.pi/2,  a=0, theta=np.pi/2, qlim=[0, 0.2]),
        rtb.PrismaticDH(offset=a2,    alpha=np.pi/2,   a=0, theta=-np.pi/2, qlim=[0, 0.2]),
        rtb.PrismaticDH(offset=a3,    alpha=0,         a=0, theta=0, qlim=[0, 0.2])
 
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

forward_kinematics(robot, q_values, save_path, name=name, status_block=False)
results = inverse_kinematics(robot,
                            Td = np.array([
                                        [1, 0, 0, 0.1],
                                        [0, 1, 0, 0.1],
                                        [0, 0, 1, 0.1],
                                        [0, 0, 0, 1]
                                    ]),
                            q0=[.98, .98, .98],
                            ilimit=1000,
                            mask=[1, 1, 1, 0, 0, 0],
                            save_path=save_path, name=name, status_block=False)

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
robot = rtb.DHRobot(
    [
        rtb.RevoluteDH( alpha=-np.pi/2,     a=0,    d=a1,       offset=0),
        rtb.RevoluteDH( alpha=0,            a=a2,   d=0,        offset=-np.pi/2),
        rtb.RevoluteDH( alpha=-np.pi/2,     a=0,    d=0,        offset=0),
        rtb.RevoluteDH( alpha=np.pi/2,      a=0,    d=a3+a4,    offset=np.pi/2),
        rtb.RevoluteDH( alpha=-np.pi/2,     a=0,    d=0,        offset=0),
        rtb.RevoluteDH( alpha=0,            a=0,    d=a5+a6,    offset=0)
    ],
    name=name
)

print("Detalles del Robot: ", name)
print(robot)

# Define the displacement values for each revolute joint
t_values = [np.pi, 0, np.pi, 0, 0, 0]
d_values = [0, 0, 0, 0, 0, 0]  # Example: [X, Y, Z] displacements in meters
q_values = [t_values[0], t_values[1], t_values[2], t_values[3], t_values[4], t_values[5]]

forward_kinematics(robot, q_values, save_path, name=name, status_block=False)
results = inverse_kinematics(robot,
                            Td = np.array([
                                        [0, 0, 1, 0.4],
                                        [1, 0, 0, 0],
                                        [0, 1, 0, 0.2],
                                        [0, 0, 0, 1]
                                    ]),
                            q0=[np.pi, 0, np.pi, 0, 0, 0],
                            ilimit=1000,
                            mask=[1, 1, 1, 0, 0, 0],
                            save_path=save_path, name=name, status_block=False)


# Robot 4 --------------------------------------------------------------
name = "4_Articulado"
a1 = 0.2  
a2 = 0.1
a3 = 0.1
a4 = 0.1
a5 = 0.1
a6 = 0.1

# Define the articulated robot with 6 revolute joints
robot = rtb.DHRobot(
    [
        rtb.RevoluteDH( alpha=-np.pi/2,     a=0,    d=a1,       offset=0),
        rtb.RevoluteDH( alpha=-np.pi/2,     a=a2,   d=0,        offset=0),
        rtb.RevoluteDH( alpha=-np.pi/2,     a=0,    d=0,        offset=0),
        rtb.RevoluteDH( alpha=np.pi/2,      a=0,    d=a3+a4,    offset=0),
        rtb.RevoluteDH( alpha=-np.pi/2,     a=0,    d=0,        offset=0),
        rtb.RevoluteDH( alpha=0,            a=0,    d=a5+a6,    offset=0)
    ],
    name=name
)

print("Detalles del Robot: ", name)
print(robot)

# Define the displacement values for each revolute joint
t_values = [np.pi, 0, np.pi, 0, 0, 0]
d_values = [0, 0, 0, 0, 0, 0]  # Example: [X, Y, Z] displacements in meters
q_values = [t_values[0], t_values[1], t_values[2], t_values[3], t_values[4], t_values[5]]

forward_kinematics(robot, q_values, save_path, name=name, status_block=False)

results = inverse_kinematics(robot,
                            Td = np.array([
                                        [1, 0, 0, -0.1],
                                        [0, 0, -1,-0.4],
                                        [0, 1, 0, 0.2],
                                        [0, 0, 0, 1]
                                    ]),
                            q0=[np.pi, 0, np.pi/2, 0, 0, 0],
                            ilimit=1000,
                            mask=[1, 1, 1, 0, 0, 0],
                            save_path=save_path, name=name, status_block=False)

# Robot 5 --------------------------------------------------------------
name = "5_Articulado"
a1 = 0.1
a2 = 0.1
a3 = 0.1
a4 = 0.1
a5 = 0.1
a6 = 0.1

# Define the articulated robot with 6 revolute joints
robot = rtb.DHRobot(
    [
        rtb.RevoluteDH( alpha=-np.pi/2,     a=0,    d=a1,       offset=0),
        rtb.RevoluteDH( alpha=0,            a=a2,   d=0,        offset=-np.pi/2),
        rtb.RevoluteDH( alpha=np.pi/2,      a=0,    d=0,        offset=0),
        rtb.RevoluteDH( alpha=-np.pi/2,     a=0,    d=a3+a4,    offset=0),
        rtb.RevoluteDH( alpha=np.pi/2,      a=0,    d=0,        offset=0),
        rtb.RevoluteDH( alpha=0,            a=0,    d=a5+a6,    offset=0)
    ],
    name=name
)

print("Detalles del Robot: ", name)
print(robot)

# Define the displacement values for each revolute joint
t_values = [np.pi, 0, np.pi, 0, 0, 0]
d_values = [0, 0, 0, 0, 0, 0]  # Example: [X, Y, Z] displacements in meters
q_values = [t_values[0], t_values[1], t_values[2], t_values[3], t_values[4], t_values[5]]

forward_kinematics(robot, q_values, save_path, name=name, status_block=False)

results = inverse_kinematics(robot,
                            Td = np.array([
                                        [0, 0, -1, -0.4],
                                        [0, -1, 0, 0],
                                        [-1, 0, 0, 0.2],
                                        [0, 0, 0, 1]
                                    ]),
                            q0=[np.pi, 0, np.pi/2, 0, 0, 0],
                            ilimit=1000,
                            mask=[1, 1, 1, 0, 0, 0],
                            save_path=save_path, name=name, status_block=False)