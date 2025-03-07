# Robotica

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

## Estructura del Repositorio
- **src/**: Contiene los scripts principales del proyecto.
  - `kinematics.py`: Ejemplos y funciones relacionadas con cinemática directa e inversa.
  - `trajectory_planning.py`: Scripts para la planificación de trayectorias robóticas.
- **notebooks/**: Notebooks de Jupyter para experimentación interactiva y demostraciones.
- **tests/**: Conjunto de pruebas unitarias para garantizar la funcionalidad del código.

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

## Ejemplos Destacados
### Cálculo de Cinemática Directa
El archivo `kinematics.py` incluye un ejemplo práctico para calcular la posición final de un manipulador robótico a partir de sus ángulos articulares:
```python
from roboticstoolbox import DHRobot, RevoluteDH

# Definir el robot mediante parámetros DH
robot = DHRobot([
    RevoluteDH(d=0.3, a=0.5, alpha=0),
    RevoluteDH(d=0, a=0.5, alpha=0)
])

# Definir los ángulos articulares
q = [0.5, 0.3]  # en radianes

# Calcular la transformación homogenea final
T = robot.fkine(q)
print(T)
```

### Planificación de Trayectorias
En el script `trajectory_planning.py` se aborda el diseño de trayectorias suaves entre dos configuraciones del robot, empleando funciones avanzadas de interpolación.

## Cómo Contribuir
¡Tu ayuda es bienvenida! Sigue estos pasos para contribuir:
1. Realiza un fork del repositorio.
2. Crea una rama nueva para tu aporte.
3. Realiza un pull request con los cambios propuestos.

## Licencia
Este proyecto está disponible bajo la licencia MIT. Consulta el archivo `LICENSE` para más detalles.

