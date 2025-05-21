"""
Código para generar la trayectoria de la letra M y un retorno circular
en el espacio articular (de la última pose al primer waypoint).
"""

import numpy as np
import time
import matplotlib.pyplot as plt
from spatialmath import SE3
import roboticstoolbox as rtb
from roboticstoolbox import ctraj, jtraj

# ---------------------------
#    1. Definición del robot
# ---------------------------
a1 = 141/1000
a2 = 120/1000
a3 = 118/1000

name = "Robot_3ejes"
robot = rtb.DHRobot(
    [
        rtb.RevoluteDH(alpha=-np.pi/2, a=0,    d=a1, offset=0, qlim=(-np.deg2rad(-180), np.deg2rad(180))),
        rtb.RevoluteDH(alpha=0,        a=a2,   d=0,  offset=-np.deg2rad(90), qlim=(-np.deg2rad(-(260/2)), np.deg2rad((260/2)))),
        rtb.RevoluteDH(alpha=0,        a=a3,   d=0,  offset=0, qlim=(-np.deg2rad(-(225/2)), np.deg2rad((225/2))))
    ],
    name=name
)
print("Robot details:")
print(robot)

# ---------------------------
#    2. Creación de matriz de coordenadas para trayectoria
# ---------------------------
# Para este ejemplo se genera la letra M en el plano XZ, con offset en Y
offset = 0.10 # Desplazamiento en Y

# Definir los vértices 3D (en este caso en el plano XZ, Y constante)
vertices_3d = [
    (-0.10, offset, 0.05),
    #(-0.06, offset, 0.075),
    (-0.06, offset, 0.075),
    #(-0.03, offset, 0.075),
    (-0.02, offset, 0.05)
]

# Inicializar P con el primer vértice
P = np.array(vertices_3d[0]).reshape(3, 1)
# Concatenar el resto de los vértices
for v in vertices_3d[1:]:
    new_col = np.array(v).reshape(3, 1)
    P = np.concatenate((P, new_col), axis=1)

#########################################
#        Revisar espacio de trabajo
#########################################
def es_punto_lineal_o_articulado(x, y, z, radio_max, apertura_cono):
    # El cubo debe estar dentro de la esfera, y su diagonal debe ser <= al diámetro de la esfera
    lado_cubo = 2 * radio_max / np.sqrt(3)
    # Comprobación del cubo
    dentro_del_cubo = abs(x) <= lado_cubo/2 and abs(y) <= lado_cubo/2 and abs(z) <= lado_cubo/2
    # Comprobación del cono con punta en origen y apertura hacia Z negativo
    r = np.sqrt(x**2 + y**2 + z**2)
    if r > 0:
        if z < 0:  # Solo verificar el cono en el hemisferio inferior
            angulo_desde_z_neg = np.arccos(-z / r)
            # Un punto está FUERA del cono si su ángulo es MAYOR que apertura_cono/2
            fuera_del_cono = angulo_desde_z_neg > apertura_cono/2
        else:
            # Puntos en el hemisferio superior siempre están fuera del cono
            fuera_del_cono = True
    else:
        # El origen está en la punta del cono (dentro del cono)
        fuera_del_cono = False
    # Verificar si el punto es verde (dentro del cubo y fuera del cono)
    es_lineal = dentro_del_cubo and fuera_del_cono
    # Verificar si el punto es azul (dentro de la esfera y fuera del cono)
    distancia_al_origen = r
    es_articular = distancia_al_origen <= radio_max and fuera_del_cono
    return es_lineal, es_articular

radio_max = a2 + a3
apertura_cono = np.deg2rad(60)

# Verificar cada waypoint
for i in range(P.shape[1]):
    x, y, z = P[:, i].flatten()
    es_lineal, es_articular = es_punto_lineal_o_articulado(x, y, z, radio_max, apertura_cono)
    print(f"WP{i+1}: lineal - {es_lineal}, articulado - {es_articular}")

# Matriz de rotación (identidad en este caso)
R = np.array([[1, 0, 0],
              [0, 1, 0],
              [0, 0, 1]])

# Crear una lista de poses SE3 para cada waypoint
coordenadas = [SE3.Rt(R, [x, y, z]) for x, y, z in zip(P[0], P[1], P[2])]
print("\nCoordenadas en el espacio de trabajo:")
for i, pos in enumerate(coordenadas):
    print(f"WP{i+1}: {pos}")

# ---------------------------
#    3. Generar la trayectoria usando ctraj (trayectoria cartesiana)
# ---------------------------
T_segment = 10      # Duración conceptual de cada segmento en segundos
n_points = 50       # Número de puntos por segmento
q_traj_segments = []  # Lista para almacenar trayectorias en espacio articular
q_current = np.zeros(robot.n)  # Condición inicial para IK

ik_start = time.time()

# Para cada segmento entre waypoints consecutivos (trayectoria cartesiana)
for i in range(len(coordenadas) - 1):
    cart_traj = ctraj(coordenadas[i], coordenadas[i+1], n_points)
    q_traj_seg = []  # Trayectoria articular para este segmento
    for pose in cart_traj:
        sol = robot.ikine_LM(pose, q0=q_current, mask=(1,1,1,0,0,0))
        if sol.success:
            q_traj_seg.append(sol.q)
            q_current = sol.q  # Actualizar q_current para la siguiente iteración
        else:
            print(f"IK falló en el segmento {i+1} para una pose intermedia")
    q_traj_segments.append(np.array(q_traj_seg))

ik_end = time.time()

print("\nTiempo total de cálculo de IK en segmentos: {:.4f} segs".format(ik_end - ik_start))

# ---------------------------
#    4. Generar trayectoria de retorno circular en el espacio articular
#    (de la última pose al primer waypoint)
# ---------------------------
# Obtener las configuraciones articulares para el primer y último waypoint
sol_first = robot.ikine_LM(coordenadas[0], q0=q_current, mask=(1,1,1,0,0,0))
if sol_first.success:
    q_first = sol_first.q
    q_current = sol_first.q
    print(f"IK para primer waypoint resuelta: {q_first}")
else:
    print("IK falló para primer waypoint")

sol_last = robot.ikine_LM(coordenadas[-1], q0=q_current, mask=(1,1,1,0,0,0))
if sol_last.success:
    q_last = sol_last.q
    q_current = sol_last.q
    print(f"IK para último waypoint resuelta: {q_last}")
else:
    print("IK falló para último waypoint")

# Definir la parametrización del círculo en el espacio articular
# Queremos pasar de q_last (inicio del retorno) a q_first (final del retorno)
m = (q_last + q_first) / 2           # Centro del arco
d = q_first - q_last                 # Vector de la cuerda
r = np.linalg.norm(d) / 2            # Radio del círculo

# Elegir un vector arbitrario para definir el plano del círculo
v = np.array([0, 0, 1])
if np.allclose(np.cross(d, v), 0):
    v = np.array([0, 1, 0])
u = np.cross(d, v)
u = u / np.linalg.norm(u)

n_points_circular = 50
theta = np.linspace(0, np.pi, n_points_circular)
q_circular = []
for t in theta:
    # Parametrización del círculo:
    # Para t=0 se obtiene q_last y para t=pi se obtiene q_first
    q_t = m + (q_last - m) * np.cos(t) + u * r * np.sin(t)
    q_circular.append(q_t)
q_circular = np.array(q_circular)

# Agregar la trayectoria circular a la lista de segmentos de trayectoria
q_traj_segments.append(q_circular)
print("Trayectoria de retorno circular generada exitosamente.")

# Concatenar todas las trayectorias en una secuencia completa de configuraciones articulares
q_traj = np.vstack(q_traj_segments)

# ---------------------------
#    5. Evaluar la trayectoria generada usando FK
# ---------------------------
task_traj = []
for q in q_traj:
    T_fk = robot.fkine(q)
    task_traj.append(T_fk.t)  # Extraer la traslación de la pose
task_traj = np.array(task_traj)

# Para comparar, se cierra la trayectoria ideal agregando el primer waypoint al final
P_closed = np.hstack((P, P[:, 0:1]))

# ---------------------------
#    6. Graficar Trayectorias, Ángulos, Velocidades y Aceleraciones
# ---------------------------
# Plot de la trayectoria en el espacio de trabajo
traj_fig = plt.figure("Trayectorias")
ax = traj_fig.add_subplot(111, projection='3d')
ax.plot(task_traj[:, 0], task_traj[:, 1], task_traj[:, 2],
        'b-', label='Trayectoria generada (IK)')
ax.plot(P_closed[0], P_closed[1], P_closed[2],
        'r:', linewidth=2, label='Trayectoria Ideal')
ax.scatter(P[0], P[1], P[2],
           color='red', marker='o', s=50, label='Coordenadas')
ax.set_title('Trayectoria en el Espacio de Trabajo')
ax.set_xlabel('X [m]')
ax.set_ylabel('Y [m]')
ax.set_zlabel('Z [m]')
ax.legend()
ax.set_box_aspect([1, 1, 1])

# Vector de tiempo para la trayectoria completa.
num_segments = len(q_traj_segments)
time_total = T_segment * num_segments
t_vec = np.linspace(0, time_total, q_traj.shape[0])

# Calcular velocidades y aceleraciones usando diferencias finitas
qd_traj = np.gradient(q_traj, t_vec[1]-t_vec[0], axis=0)
qdd_traj = np.gradient(qd_traj, t_vec[1]-t_vec[0], axis=0)

# Plot de Ángulo de Articulaciones vs Tiempo
fig_joint = plt.figure("Ángulo vs Tiempo")
for i in range(q_traj.shape[1]):
    plt.plot(t_vec, q_traj[:, i], label=f'J{i+1}')
plt.xlabel("Tiempo (s)")
plt.ylabel("Ángulo (rad)")
plt.title("Ángulo de Articulaciones vs Tiempo")
plt.legend()
plt.grid(True)

# Plot de Velocidades de Articulaciones vs Tiempo
fig_vel = plt.figure("Velocidades vs Tiempo")
for i in range(qd_traj.shape[1]):
    plt.plot(t_vec, qd_traj[:, i], label=f'J{i+1}')
plt.xlabel("Tiempo (s)")
plt.ylabel("Velocidad (rad/s)")
plt.title("Velocidades de Articulaciones vs Tiempo")
plt.legend()
plt.grid(True)

# Plot de Aceleraciones de Articulaciones vs Tiempo
fig_acc = plt.figure("Aceleraciones vs Tiempo")
for i in range(qdd_traj.shape[1]):
    plt.plot(t_vec, qdd_traj[:, i], label=f'J{i+1}')
plt.xlabel("Tiempo (s)")
plt.ylabel("Aceleración (rad/s²)")
plt.title("Aceleraciones de Articulaciones vs Tiempo")
plt.legend()
plt.grid(True)



# ---------------------------
#    7. Conversión de radianes a pasos (CORREGIDA)
# ---------------------------

def rad_to_steps_and_derivatives():
    """
    Convierte las trayectorias de radianes, rad/s y rad/s² a pasos
    considerando las relaciones de engranes y resolución de motores
    """
    
    # Parámetros de los motores
    step_resolution = 1.8  # grados por paso
    step_resolution_rad = np.deg2rad(step_resolution)  # radianes por paso
    
    # Relaciones de engranes (conductores/conducidos)
    # Cuando el motor gira 1 revolución, la articulación gira (relación) revoluciones
    gear_ratios = {
        'M1': 12/31,  # Motor 1: por cada vuelta del motor, la articulación gira 31/12 vueltas
        'M2': 12/31,  
        'M3': 20/60   
    }
    # Calcular pasos por radián para cada motor
    # pasos_motor = angulo_articulacion * (1/gear_ratio) * (1/step_resolution_rad)
    steps_per_rad = {}
    for motor, ratio in gear_ratios.items():
        # Número de pasos = (radianes_motor) / (radianes_por_paso)
        steps_per_rad[motor] = 1 / (step_resolution_rad * ratio)
    
    print("Resolución de cada motor:")
    for motor, spr in steps_per_rad.items():
        print(f"{motor}: {spr:.2f} pasos/rad = {spr * 180/np.pi:.2f} pasos/grado")
    
    # Convertir posiciones (rad -> pasos)
    q_steps = np.zeros_like(q_traj)
    q_steps[:, 0] = q_traj[:, 0] * steps_per_rad['M1']
    q_steps[:, 1] = q_traj[:, 1] * steps_per_rad['M2'] 
    q_steps[:, 2] = q_traj[:, 2] * steps_per_rad['M3']
    
    # Convertir velocidades (rad/s -> pasos/s)
    qd_steps = np.zeros_like(qd_traj)
    qd_steps[:, 0] = qd_traj[:, 0] * steps_per_rad['M1']
    qd_steps[:, 1] = qd_traj[:, 1] * steps_per_rad['M2']
    qd_steps[:, 2] = qd_traj[:, 2] * steps_per_rad['M3']
    
    # Convertir aceleraciones (rad/s² -> pasos/s²)
    qdd_steps = np.zeros_like(qdd_traj)
    qdd_steps[:, 0] = qdd_traj[:, 0] * steps_per_rad['M1']
    qdd_steps[:, 1] = qdd_traj[:, 1] * steps_per_rad['M2']
    qdd_steps[:, 2] = qdd_traj[:, 2] * steps_per_rad['M3']
    
    return q_steps, qd_steps, qdd_steps, steps_per_rad


def create_absolute_movement_list(q_steps, qd_steps, qdd_steps, simplify_factor=1,
                                vel_scale=1, acc_scale=1):
    """
    Crea movimientos absolutos en lugar de incrementales para obtener valores más grandes
    """
    # Simplificar la trayectoria
    indices = np.arange(0, len(q_steps), simplify_factor)
    q_simplified = q_steps[indices]
    qd_simplified = qd_steps[indices] 
    qdd_simplified = qdd_steps[indices]
    
    movements = []
    
    for i in range(len(q_simplified)):
        positions = q_simplified[i]
        velocities = np.abs(qd_simplified[i])
        accelerations = np.abs(qdd_simplified[i])
        
        movement = []
        for motor_id in range(3):
            steps = int(round(positions[motor_id]))
            velocity = int(round(velocities[motor_id] * vel_scale))
            acceleration = int(round(accelerations[motor_id] * acc_scale))
            
            velocity = max(min(velocity, 3000), 500)
            acceleration = max(min(acceleration, 3000), 500)
            
            movement.append((motor_id + 1, steps, velocity, acceleration))
        
        movements.append(movement)
    
    return movements

# ---------------------------
#    8. Ejecutar conversiones y crear lista de movimientos (CORREGIDO)
# ---------------------------
# Convertir a pasos
q_steps, qd_steps, qdd_steps, steps_per_rad = rad_to_steps_and_derivatives()

print(f"\nRango de posiciones en pasos:")
for i in range(3):
    print(f"Motor {i+1}: min={q_steps[:, i].min():.1f}, max={q_steps[:, i].max():.1f}")

print(f"\nRango de velocidades en pasos/s:")
for i in range(3):
    print(f"Motor {i+1}: min={qd_steps[:, i].min():.1f}, max={qd_steps[:, i].max():.1f}")

print("\n Movimientos absolutos")
print("\n" + "="*50)
movements_absolute = create_absolute_movement_list(q_steps, qd_steps, qdd_steps,
                                                 simplify_factor=10, vel_scale=1, acc_scale=1)
print(f"Primeros 3 movimientos absolutos:")
for i, mov in enumerate(movements_absolute[:3]):
    print(f"Movimiento {i+1}: {mov}")

movements = movements_absolute
# Guardar movimientos en un archivo en el mismo formato de lista con print
with open("Web/Reportes/Teoria/Proy_final/assets/movements.txt", "w") as f:
    f.write(str(movements))

# ---------------------------

# Animacion del robot utilizando la trayectoria en espacio articular
robot.plot(q_traj, block=False, loop=True)
plt.show()