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