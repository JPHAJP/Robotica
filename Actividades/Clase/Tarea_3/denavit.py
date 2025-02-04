import numpy as np
from sympy import symbols, cos, sin, Matrix, pprint, simplify, pi

# Función para crear una matriz de transformación usando los parámetros DH
def dh_matrix(theta, d, a, alpha):
    # Verificar si los ángulos son numéricos o simbólicos
    if theta.replace('-', '', 1).replace('.', '', 1).isdigit():
        theta_rad = float(theta) * (pi / 180)  # Convertir a radianes
    else:
        theta_rad = symbols(theta)

    if alpha.replace('-', '', 1).replace('.', '', 1).isdigit():
        alpha_rad = float(alpha) * (pi / 180)  # Convertir a radianes
    else:
        alpha_rad = symbols(alpha)

    return Matrix([
        [cos(theta_rad), -sin(theta_rad) * cos(alpha_rad), sin(theta_rad) * sin(alpha_rad) , a * cos(theta_rad)],
        [sin(theta_rad), cos(theta_rad) * cos(alpha_rad) , -cos(theta_rad) * sin(alpha_rad), a * sin(theta_rad)],
        [      0       ,           sin(alpha_rad)        ,          cos(alpha_rad)         ,        d          ],
        [      0       ,                0                ,               0                 ,        1          ]
    ])

# Solicitar el número de filas
total_filas = int(input("Ingresa el número de filas de la tabla DH: "))

# Lista para almacenar las matrices de transformación
matrices = []

# Recolectar los parámetros DH y calcular la matriz de cada fila
for i in range(total_filas):
    print(f"\nParámetros para la fila {i+1}:")
    a = input("  Ingresa a: ")
    alpha = input("  Ingresa α : ")
    d = input("  Ingresa d : ")
    theta = input("  Ingresa θ : ")

    # Convertir a números si es posible
    d = float(d) if d.replace('-', '', 1).replace('.', '', 1).isdigit() else symbols(d)
    a = float(a) if a.replace('-', '', 1).replace('.', '', 1).isdigit() else symbols(a)

    T = dh_matrix(theta, d, a, alpha)
    matrices.append(T)

    print(f"\nMatriz de transformación T{i+1}:")
    pprint(simplify(T))  # Simplificar la matriz antes de mostrarla

# Calcular la matriz de transformación total
T_total = Matrix.eye(4)  # Matriz identidad 4x4
for T in matrices:
    T_total = T_total * T

# Simplificar la matriz final
T_total_simplificada = simplify(T_total)

print("\nMatriz de transformación final (T_total):")
pprint(T_total_simplificada)
