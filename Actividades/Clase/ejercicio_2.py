import numpy as np

def rotacion_x(angulo_grados):
    """Genera una matriz de rotación alrededor del eje X."""
    angulo_rad = np.radians(angulo_grados)
    R_x = np.array([
        [1, 0, 0],
        [0, np.cos(angulo_rad), -np.sin(angulo_rad)],
        [0, np.sin(angulo_rad), np.cos(angulo_rad)]
    ])
    R_x = np.round(R_x, 4)
    return R_x

def matriz_transformacion_homogenea(R, t):
    """Construye la matriz de transformación homogénea dada una rotación y una traslación."""
    T = np.eye(4)  # Matriz identidad 4x4
    T[:3, :3] = R  # Inserta la matriz de rotación
    T[:3, 3] = t   # Inserta el vector de traslación
    #imprimir matriz de transformación homogénea a 2 decimales
    T = np.round(T, 4)
    return T

# Ángulo de rotación y vector de traslación
angulo_x = 30  # grados
traslacion = np.array([5, 10, 0])  # Vector de traslación

# Matriz de rotación alrededor de X
R_x = rotacion_x(angulo_x)

# Construcción de la matriz de transformación homogénea
T = matriz_transformacion_homogenea(R_x, traslacion)

# Imprimir resultados
print("Matriz de rotación alrededor de X:")
print(R_x)
print("\nMatriz de transformación homogénea:")
print(T)
