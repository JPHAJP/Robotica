import numpy as np
import matplotlib.pyplot as plt

# Función para calcular el espacio de trabajo (esfera con cono invertido)
def calcular_espacio_trabajo(radio_max, apertura_cono, puntos=10000):
    puntos_3d = []
    for _ in range(puntos):
        # Generar coordenadas esféricas aleatorias
        r = radio_max * np.cbrt(np.random.rand())
        theta = np.random.uniform(0, 2 * np.pi)
        phi = np.random.uniform(0, np.pi)
        
        # Convertir a coordenadas cartesianas
        x = r * np.sin(phi) * np.cos(theta)
        y = r * np.sin(phi) * np.sin(theta)
        z = r * np.cos(phi)

        # Cono con punta en origen y apertura hacia Z negativo
        # Solo consideramos puntos con z <= 0 para el cono
        if z < 0:  # Solo puntos en el hemisferio inferior
            # Calcular el ángulo desde el eje Z negativo
            angulo_desde_z_neg = np.arccos(-z / r)
            # Un punto está FUERA del cono si su ángulo es MAYOR que apertura_cono/2
            if angulo_desde_z_neg > apertura_cono/2:
                puntos_3d.append((x, y, z))
        else:
            # Puntos en el hemisferio superior (z >= 0) siempre están fuera del cono
            puntos_3d.append((x, y, z))
    
    return np.array(puntos_3d)

# Función para filtrar los puntos dentro del cubo que son afectados por el cono
def filtrar_puntos_por_cono(puntos_cubo, apertura_cono):
    puntos_filtrados = []
    for punto in puntos_cubo:
        x, y, z = punto
        r = np.sqrt(x**2 + y**2 + z**2)
        
        if r > 0:
            if z < 0:  # Solo verificar el cono en el hemisferio inferior
                angulo_desde_z_neg = np.arccos(-z / r)
                # Un punto está FUERA del cono si su ángulo es MAYOR que apertura_cono/2
                if angulo_desde_z_neg > apertura_cono/2:
                    puntos_filtrados.append(punto)
            else:
                # Puntos en el hemisferio superior siempre están fuera del cono
                puntos_filtrados.append(punto)
        else:
            # El origen está dentro del cono (es la punta)
            pass
    
    return np.array(puntos_filtrados)

# Función para verificar si un punto es verde o azul
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

# Parámetros del sistema
radio_max = 18/100  # Radio máximo de la esfera
apertura_cono = np.deg2rad(100)  # Ángulo de apertura del cono

print(f"Apertura del cono: {apertura_cono} radianes ({np.rad2deg(apertura_cono)} grados)")
print(f"Radio máximo: {radio_max}")

# ===== PUNTOS DE PRUEBA =====
# Aquí puedes agregar todos los puntos que quieras
puntos_prueba = [
    (-0.10, 0.09, 0.05, "Punto 1"),
    (-0.06, 0.09, 0.075, "Punto 2"),
    (-0.02, 0.09, 0.05, "Punto 3")
]

# Colores para cada punto de prueba
colores_prueba = ['magenta', 'red', 'orange', 'yellow', 'cyan', 'purple']

print("\n===== ANÁLISIS DE PUNTOS DE PRUEBA =====")
for i, (x, y, z, nombre) in enumerate(puntos_prueba):
    es_lineal, es_articular = es_punto_lineal_o_articulado(x, y, z, radio_max, apertura_cono)
    print(f"{nombre} ({x}, {y}, {z}):")
    print(f"  ¿Es verde (lineal)? {es_lineal}")
    print(f"  ¿Es azul (articular)? {es_articular}")
    print()

# Función para dibujar el cubo dentro de la esfera
def dibujar_cubo(radio_max):
    lado_cubo = 2 * radio_max / np.sqrt(3)
    puntos_cubo = []

    for x in np.linspace(-lado_cubo/2, lado_cubo/2, num=15):
        for y in np.linspace(-lado_cubo/2, lado_cubo/2, num=15):
            for z in np.linspace(-lado_cubo/2, lado_cubo/2, num=15):
                puntos_cubo.append((x, y, z))

    return np.array(puntos_cubo)

# Calcular puntos del espacio de trabajo
puntos_trabajo = calcular_espacio_trabajo(radio_max, apertura_cono)

# Dibujar el cubo dentro de la esfera
puntos_cubo = dibujar_cubo(radio_max)

# Filtrar puntos dentro del cubo que no sean afectados por el cono
puntos_cubo_filtrados = filtrar_puntos_por_cono(puntos_cubo, apertura_cono)

# Graficar el espacio de trabajo
fig = plt.figure(figsize=(12, 10))
ax = fig.add_subplot(111, projection='3d')

# Graficar los puntos dentro del área de trabajo (esfera)
ax.scatter(puntos_trabajo[:, 0], puntos_trabajo[:, 1], puntos_trabajo[:, 2], 
          c='blue', s=0.5, alpha=0.6, label="Espacio de trabajo")

# Graficar los puntos verdes dentro del cubo, que están fuera del cono
if len(puntos_cubo_filtrados) > 0:
    ax.scatter(puntos_cubo_filtrados[:, 0], puntos_cubo_filtrados[:, 1], puntos_cubo_filtrados[:, 2], 
              c='green', s=20, label="Puntos del cubo fuera del cono")

# Graficar todos los puntos de prueba
for i, (x, y, z, nombre) in enumerate(puntos_prueba):
    color = colores_prueba[i % len(colores_prueba)]  # Usar colores cíclicamente
    ax.scatter(x, y, z, c=color, s=100, label=nombre, edgecolors='black', linewidth=1)

# Marcar el origen (punta del cono)
ax.scatter(0, 0, 0, c='red', s=100, label="Origen (punta del cono)", marker='x')

# Configuración de la visualización
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title(f"Espacio de trabajo - Cono {np.rad2deg(apertura_cono):.0f}° con punta en origen")

# Establecer límites iguales para todos los ejes
ax.set_xlim([-radio_max, radio_max])
ax.set_ylim([-radio_max, radio_max])
ax.set_zlim([-radio_max, radio_max])

# Mostrar gráfico con leyenda
ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
plt.tight_layout()
plt.show()