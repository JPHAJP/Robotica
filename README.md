# Robótica

## Descripción
Este repositorio contiene el código desarrollado para la clase de robótica, utilizando **Python** y la librería **roboticstoolbox-python**. El proyecto tiene como objetivo principal explorar la simulación y el control de robots mediante herramientas avanzadas.

## Características Principales
- Simulación de manipuladores robóticos en entornos virtuales.
- Desarrollo de algoritmos de cinemática directa e inversa.
- Visualización interactiva en 3D de modelos robóticos.
- Generación de trayectorias y diseño de controles básicos.

## Requisitos del Proyecto
Antes de comenzar, verifica que tengas instaladas las siguientes dependencias:

- Python 3.8 o superior.
- **roboticstoolbox-python**: para las herramientas de simulación.
- **NumPy**: para el manejo eficiente de datos numéricos.
- **Matplotlib**: para la visualización de resultados.

Para instalar las dependencias, utiliza el archivo `requirements.txt`:
```bash
pip install -r requirements.txt
```


## Cómo Ejecutar el Proyecto
1. Clona el repositorio:
   ```bash
   git clone https://github.com/JPHAJP/Robotica.git
   ```

2. Cambia al directorio del proyecto:
   ```bash
   cd Robotica
   ```

3. Instala las dependencias requeridas:
   ```bash
   pip install -r requirements.txt
   ```

4. Ejecuta el script que desees:
   ```bash
   python src/kinematics.py
   ```

## Cómo Contribuir
¡Tu ayuda es bienvenida! Sigue estos pasos para contribuir:
1. Realiza un fork del repositorio.
2. Crea una rama nueva para tu aporte.
3. Realiza un pull request con los cambios propuestos.

## Licencia
Este proyecto está disponible bajo la licencia MIT. Consulta el archivo `LICENSE` para más detalles.

K33BN38P3Y1HHK8PCFWBVWY1


El formato para ingresar una trayectoria manual sería:
1,50,1000,1000;2,30,800,800;3,-40,1200,1200|1,-50,1000,1000;2,-30,800,800;3,40,1200,1200
Donde:

El carácter | separa movimientos secuenciales
El carácter ; separa configuraciones de motor dentro de un mismo movimiento
Cada configuración de motor tiene 4 parámetros separados por comas: ID del motor, pasos, velocidad, aceleración