import numpy as np
import matplotlib.pyplot as plt

# Función para calcular el espacio de trabajo (esfera con cono invertido)
def calcular_espacio_trabajo(radio_max, apertura_cono, puntos=10000):
    # Generar puntos aleatorios dentro de una esfera
    puntos_3d = []
    for _ in range(puntos):
        # Generar coordenadas esféricas aleatorias
        r = radio_max * np.cbrt(np.random.rand())  # Para que la distribución sea uniforme
        theta = np.random.uniform(0, 2 * np.pi)
        phi = np.random.uniform(0, np.pi)
        
        # Convertir a coordenadas cartesianas
        x = r * np.sin(phi) * np.cos(theta)
        y = r * np.sin(phi) * np.sin(theta)
        z = r * np.cos(phi)

        # Comprobar si el punto está fuera del cono invertido
        angulo_cono = np.arctan2(np.sqrt(x**2 + y**2), -z)  # Ahora comprobamos el ángulo con respecto al eje Z negativo
        if angulo_cono > apertura_cono:
            puntos_3d.append((x, y, z))
    
    return np.array(puntos_3d)

# Función para dibujar el cubo dentro de la esfera
def dibujar_cubo(radio_max):
    # El cubo debe estar dentro de la esfera, y su diagonal debe ser <= al diámetro de la esfera
    lado_cubo = 2 * radio_max / np.sqrt(3)  # Calcular el lado del cubo
    puntos_cubo = []

    for x in np.linspace(-lado_cubo/2, lado_cubo/2, num=15):  # Añadir puntos intermedios para una mejor visualización
        for y in np.linspace(-lado_cubo/2, lado_cubo/2, num=15):
            for z in np.linspace(-lado_cubo/2, lado_cubo/2, num=15):
                puntos_cubo.append((x, y, z))

    return np.array(puntos_cubo)

# Función para filtrar los puntos dentro del cubo que son afectados por el cono
def filtrar_puntos_por_cono(puntos_cubo, apertura_cono):
    puntos_filtrados = []
    for punto in puntos_cubo:
        x, y, z = punto
        angulo_cono = np.arctan2(np.sqrt(x**2 + y**2), -z)  # Ángulo con respecto al eje Z negativo
        if angulo_cono > apertura_cono:  # Solo conservar puntos fuera del cono
            puntos_filtrados.append((x, y, z))
    return np.array(puntos_filtrados)


########################################################################################
#                           FUNCIONES DE PRUEBA
########################################################################################

# Función para verificar si un punto es verde o azul
def es_punto_lineal_o_articulado(x, y, z, radio_max, apertura_cono):
    # El cubo debe estar dentro de la esfera, y su diagonal debe ser <= al diámetro de la esfera
    lado_cubo = 2 * radio_max / np.sqrt(3)  # Calcular el lado del cubo
    
    # Comprobación del cubo: El punto debe estar dentro de los límites del cubo
    dentro_del_cubo = abs(x) <= lado_cubo/2 and abs(y) <= lado_cubo/2 and abs(z) <= lado_cubo/2
    
    # Comprobación del cono: El punto debe estar fuera del cono (se calcula el ángulo)
    angulo_cono = np.arctan2(np.sqrt(x**2 + y**2), -z)  # Ángulo con respecto al eje Z negativo
    fuera_del_cono = angulo_cono > apertura_cono

    # Verificar si el punto es verde
    es_lineal = dentro_del_cubo and fuera_del_cono
    
    # Verificar si el punto es azul
    # El punto debe estar dentro de la esfera y fuera del cono
    distancia_al_origen = np.sqrt(x**2 + y**2 + z**2)
    es_articular = distancia_al_origen <= radio_max and fuera_del_cono

    return es_lineal, es_articular

# Ejemplo de uso:
x, y, z = 0.005, 0.004, 0.15  # Punto de ejemplo
radio_max = 18/100  # Radio máximo de la esfera
apertura_cono = np.deg2rad(60)  # Ángulo de apertura del cono

print(radio_max, apertura_cono)

# Verificar si el punto es verde o azul
es_lineal, es_articular = es_punto_lineal_o_articulado(x, y, z, radio_max, apertura_cono)
print(f"¿Es el punto verde? {es_lineal}")
print(f"¿Es el punto azul? {es_articular}")
#Revisando WP10... (0.005, 0.004806308479691594, 0.151378186779085)

########################################################################################
#                           PLOT DE ESPACIO DE TRABAJO
########################################################################################

# Calcular puntos del espacio de trabajo
puntos_trabajo = calcular_espacio_trabajo(radio_max, apertura_cono)

# Dibujar el cubo dentro de la esfera
puntos_cubo = dibujar_cubo(radio_max)

# Filtrar puntos dentro del cubo que no sean afectados por el cono
puntos_cubo_filtrados = filtrar_puntos_por_cono(puntos_cubo, apertura_cono)

# Graficar el espacio de trabajo
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Graficar los puntos dentro del área de trabajo (esfera)
ax.scatter(puntos_trabajo[:, 0], puntos_trabajo[:, 1], puntos_trabajo[:, 2], c='b', s=0.5, label="Espacio de trabajo")

# Graficar los puntos del cubo (rojos)
#ax.scatter(puntos_cubo[:, 0], puntos_cubo[:, 1], puntos_cubo[:, 2], c='r', s=50, label="Puntos del cubo")

# Graficar los puntos verdes dentro del cubo, que están fuera del cono
ax.scatter(puntos_cubo_filtrados[:, 0], puntos_cubo_filtrados[:, 1], puntos_cubo_filtrados[:, 2], c='g', s=20, label="Puntos afectados por el cono (dentro del cubo)")

# Graficar punto de prueba como morado
ax.scatter(x, y, z, c='m', s=50, label="Punto de prueba")

# Configuración de la visualización
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title("Espacio de trabajo con cubo y puntos afectados por el cono")

# Mostrar gráfico
ax.legend()
plt.show()