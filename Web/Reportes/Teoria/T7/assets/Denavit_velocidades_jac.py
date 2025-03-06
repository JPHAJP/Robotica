import sympy as sp

# Función para calcular la matriz homogénea
def dh_matrix(a, alpha, d, theta):
    return sp.Matrix([
        [sp.cos(theta), -sp.sin(theta)*sp.cos(alpha),  sp.sin(theta)*sp.sin(alpha), a*sp.cos(theta)],
        [sp.sin(theta),  sp.cos(theta)*sp.cos(alpha), -sp.cos(theta)*sp.sin(alpha), a*sp.sin(theta)],
        [0,              sp.sin(alpha),                sp.cos(alpha),               d],
        [0,              0,                            0,                           1]
    ])
#dh_matrix(θ, d , a, α)

# Función para generar una matriz de rotación homogénea
def rotation_matrix(x_angle=0, y_angle=0, z_angle=0):
    # Rotación alrededor del eje X
    R_x = sp.Matrix([
        [1, 0, 0, 0],
        [0, sp.cos(x_angle), -sp.sin(x_angle), 0],
        [0, sp.sin(x_angle), sp.cos(x_angle), 0],
        [0, 0, 0, 1]
    ])
    
    # Rotación alrededor del eje Y
    R_y = sp.Matrix([
        [sp.cos(y_angle), 0, sp.sin(y_angle), 0],
        [0, 1, 0, 0],
        [-sp.sin(y_angle), 0, sp.cos(y_angle), 0],
        [0, 0, 0, 1]
    ])
    
    # Rotación alrededor del eje Z
    R_z = sp.Matrix([
        [sp.cos(z_angle), -sp.sin(z_angle), 0, 0],
        [sp.sin(z_angle), sp.cos(z_angle), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    
    # Matriz de rotación combinada
    return R_x @ R_y @ R_z  # Usar @ para multiplicación de matrices

def reemplazar_trigonometria(matriz, n=3):
    """
    Reemplaza las funciones trigonométricas en la matriz:
    sin -> S
    cos -> C
    y evalúa los valores numéricos con 2 decimales.
    """
    matriz_reemplazada = matriz.applyfunc(lambda expr: expr.replace(sp.sin, lambda x: sp.Symbol(f'S({x})')).replace(sp.cos, lambda x: sp.Symbol(f'C({x})')))
    return matriz_reemplazada.evalf(n)

# Ejercicio 1:
print("\nEjercicio 1:")
# Definir las variables simbólicas
th1, a1, a2, a3, a4, th2 = sp.symbols('th1 a1 a2 a3 a4 th2')
# Primera fila de la tabla DH:
T1 = dh_matrix(a2,0,a1,th1)
# Segunda fila de la tabla DH: 
T2 = dh_matrix(a4,0,a3,th2)
# Matriz final: Producto de las transformaciones
T = T1 @ T2
# Mostrar las matrices individuales y el producto final, redondeado a 2 decimales
print("Matriz T1:")
sp.pprint(reemplazar_trigonometria(T1))  # Redondear a 2 decimales
print("\nMatriz T2:")
sp.pprint(reemplazar_trigonometria(T2))  # Redondear a 2 decimales
print("\nMatriz homogénea total T:")
sp.pprint(reemplazar_trigonometria(T))  # Redondear a 2 decimales

# Jacobiano de velocidades
 
x_v,y_v,z_v,w_x,w_y,w_z = sp.symbols('x_v y_v z_v w_x w_y w_z')

#Vector de velocidades
V = sp.Matrix([x_v, y_v, z_v, w_x, w_y, w_z])

#Obtencion de vector de velocidades lineales th1
R=sp.Matrix([0, 0, 1])
#Tomar solo los valores de la matriz de transformación homogénea
D = T[0:3,3]
Vth1_lin = R.cross(D)
print("\nVector de velocidades lineales th1:")
sp.pprint(Vth1_lin)

#Obtencion de vector de velocidades lineales th2
R=sp.Matrix([0, 0, 1])
D=T[0:3,3]-T1[0:3,3]
Vth2_lin = R.cross(D)
print("\nVector de velocidades lineales th2:")
sp.pprint(Vth2_lin)

#Obtencion de vector de velocidades angulares th1
R=sp.Matrix([0, 0, 1])
Vth1_ang = R
print("\nVector de velocidades angulares th1:")
sp.pprint(Vth1_ang)

#Obtencion de vector de velocidades angulares th2
R=sp.Matrix([0, 0, 1])
Vth2_ang = R
print("\nVector de velocidades angulares th2:")
sp.pprint(Vth2_ang)

#Meterlo en una matriz de 6x2 y multiplicar por el vector de [th1, th2]
V_R=sp.Matrix([
    [Vth1_lin[0], Vth2_lin[0]],
    [Vth1_lin[1], Vth2_lin[1]],
    [Vth1_lin[2], Vth2_lin[2]],
    [Vth1_ang[0], Vth2_ang[0]],
    [Vth1_ang[1], Vth2_ang[1]],
    [Vth1_ang[2], Vth2_ang[2]]
])
th_v = sp.Matrix([th1, th2])

J = V_R @ th_v
print("\nJacobiano de velocidades:")
sp.pprint(J)

