import cv2
import numpy as np
import time
import math
from aruco_tracker_2 import BluetoothController, RobotTargetTracker, RobotTargetObstacleTracker

class RobotController:
    """
    Controlador para un robot móvil usando detección de ArUco para navegación
    """
    def __init__(self, robot_id=0, target_id=1, 
                 bluetooth_mac="88:13:BF:70:40:72", 
                 k=0.5, visual=True):
        """
        Inicializa el controlador del robot
        
        Args:
            robot_id: ID del marcador ArUco del robot
            target_id: ID del marcador ArUco del objetivo
            bluetooth_mac: Dirección MAC del dispositivo Bluetooth del robot
            k: Ganancia del controlador proporcional
            visual: Si se debe mostrar la interfaz visual
        """
        self.robot_id = robot_id
        self.target_id = target_id
        self.bluetooth_mac = bluetooth_mac
        self.k = k
        self.visual = visual
        
        # Parámetros del robot
        self.wmax = 150  # Velocidad angular máxima
        self.l = -1       #-185    # Distancia del centro del robot al centro de las llantas (mm)
        self.r = 127     # Radio de llanta (mm)
        self.L = 260     # Distancia entre centros de llantas (mm)
        self.pwm_min = 180  # PWM mínimo para el movimiento
        
        # Inicializar controlador Bluetooth
        self.bt_controller = BluetoothController(esp32_addr=bluetooth_mac)
        
        # Inicializar tracker
        self.tracker = RobotTargetTracker(
            robot_id=robot_id, 
            target_id=target_id,
            robot_offset_px=190, 
            target_offset_px=83,
            robot_offset_mm=268, 
            target_offset_mm=138, 
            visual=visual
        )
        
        # Valores para almacenar datos
        self.last_robot_pos = None
        self.last_target_pos = None
        self.center_x = None
        self.center_y = None
        
        # Arreglos para almacenar datos de trayectoria
        self.max_points = 100
        self.x, self.y, self.th, self.t, self.ex, self.ey, self.ux, self.uy = self.inicializar_arreglos(self.max_points)
        self.i = 0  # Índice actual en los arreglos
    
    def inicializar_arreglos(self, j):
        """
        Inicializa arreglos para almacenar datos de trayectoria
        
        Args:
            j: Tamaño de los arreglos
            
        Returns:
            Tupla de arreglos inicializados
        """
        x = np.zeros(j+1, dtype=float)
        y = np.zeros(j+1, dtype=float)
        t = np.arange(j+1, dtype=float)
        ex = np.zeros(j+1, dtype=float)
        ey = np.zeros(j+1, dtype=float)
        ux = np.zeros(j+1, dtype=float)
        uy = np.zeros(j+1, dtype=float)
        th = np.zeros(j+1, dtype=float)
        return x, y, th, t, ex, ey, ux, uy
    
    def convergencia(self, x, y, xs, ys):
        """
        Calcula el control de convergencia
        
        Args:
            x: Posición actual en x
            y: Posición actual en y
            xs: Posición deseada en x
            ys: Posición deseada en y
            
        Returns:
            Tupla de (ex, ey, ux, uy) - errores y señales de control
        """
        # Cálculo de error
        ex = x - xs
        ey = y - ys
        
        # Control proporcional
        ux = -self.k * ex
        uy = -self.k * ey
        
        return ex, ey, ux, uy
    
    def ajustar_pwm(self, valor):
        """
        Ajusta el valor PWM para respetar el mínimo requerido
        
        Args:
            valor: Valor PWM original
            
        Returns:
            Valor PWM ajustado
        """
        if abs(valor) < self.pwm_min and abs(valor) > 0:
            return self.pwm_min if valor > 0 else -self.pwm_min
        return valor
    
    def connect(self):
        """
        Establece conexión Bluetooth con el robot
        
        Returns:
            True si la conexión fue exitosa, False de lo contrario
        """
        return self.bt_controller.connect()
    
    def disconnect(self):
        """
        Cierra la conexión Bluetooth con el robot
        """
        self.bt_controller.disconnect()
    
    def run(self):
        """
        Ejecuta el bucle principal de control del robot
        """
        # Inicializar cámara a través del tracker
        if not self.tracker.detector.initialize_camera():
            print("Error al inicializar la cámara. Saliendo...")
            return
        
        # Establecer conexión Bluetooth
        if not self.connect():
            print("Error al conectar con el robot. Saliendo...")
            self.tracker.detector.close()
            return
        
        print("Conexión con robot establecida. Iniciando control...")
        
        # Obtener dimensiones de resolución
        self.center_x = self.tracker.detector.resolution_x // 2
        self.center_y = self.tracker.detector.resolution_y // 2
        
        try:
            while True:
                # Obtener frame y detectar marcadores ArUco
                frame, points, ids = self.tracker.detector.buscar_aruco()
                
                if frame is None:
                    print("No se pudo obtener frame. Comprobando conexión...")
                    time.sleep(0.5)
                    continue
                
                # Procesar frame
                if points is not None and len(points) > 0 and ids is not None:
                    # Dibujar marcadores ArUco
                    self.tracker.detector.dibujar_aruco(frame, points, ids)
                    
                    # Buscar robot y objetivo
                    robot_found = False
                    target_found = False
                    
                    robot_pos_mm = None
                    target_pos_mm = None
                    robot_angle = None
                    
                    ids = ids.flatten()
                    for (marker_corner, marker_id) in zip(points, ids):
                        # Procesar marcadores y obtener información
                        if marker_id == self.robot_id or marker_id == self.target_id:
                            # Obtener coordenadas y ángulo
                            corners = marker_corner.reshape((4, 2))
                            (top_left, top_right, bottom_right, bottom_left) = corners
                            
                            # Convertir coordenadas a enteros
                            top_right = (int(top_right[0]), int(top_right[1]))
                            bottom_right = (int(bottom_right[0]), int(bottom_right[1]))
                            bottom_left = (int(bottom_left[0]), int(bottom_left[1]))
                            top_left = (int(top_left[0]), int(top_left[1]))
                            
                            # Calcular punto medio
                            mid_x = (top_right[0] + bottom_left[0]) // 2
                            mid_y = (top_right[1] + bottom_left[1]) // 2
                            
                            # Calcular ángulo
                            x = (bottom_right[0] - bottom_left[0])
                            y = (bottom_right[1] - bottom_left[1])
                            angle = math.atan2(y, x)
                            
                            if marker_id == self.robot_id:
                                # Calcular POI (punto de interés) del robot
                                robot_poi_x = int(mid_x + self.tracker.robot_offset_px * np.cos(angle))
                                robot_poi_y = int(mid_y + self.tracker.robot_offset_px * np.sin(angle))
                                
                                # Convertir a mm
                                robot_x_mm = (robot_poi_x - self.center_x) * self.tracker.mm_per_px
                                robot_y_mm = (robot_poi_y - self.center_y) * self.tracker.mm_per_px
                                
                                robot_pos_mm = (robot_x_mm, robot_y_mm)
                                robot_angle = angle
                                robot_found = True
                                
                                # Dibujar POI del robot
                                cv2.circle(frame, (robot_poi_x, robot_poi_y), 10, (0, 255, 0), -1)
                                cv2.putText(frame, f"Robot: {int(robot_x_mm)},{int(robot_y_mm)}", 
                                           (robot_poi_x + 10, robot_poi_y - 10), 
                                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                                
                            elif marker_id == self.target_id:
                                # Calcular POI del objetivo
                                target_poi_x = int(mid_x + self.tracker.target_offset_px * np.cos(angle))
                                target_poi_y = int(mid_y + self.tracker.target_offset_px * np.sin(angle))
                                
                                # Convertir a mm
                                target_x_mm = (target_poi_x - self.center_x) * self.tracker.mm_per_px
                                target_y_mm = (target_poi_y - self.center_y) * self.tracker.mm_per_px
                                
                                target_pos_mm = (target_x_mm, target_y_mm)
                                target_found = True
                                
                                # Dibujar POI del objetivo
                                cv2.circle(frame, (target_poi_x, target_poi_y), 10, (255, 0, 0), -1)
                                cv2.putText(frame, f"Target: {int(target_x_mm)},{int(target_y_mm)}", 
                                           (target_poi_x + 10, target_poi_y - 10), 
                                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
                    
                    # Si se encontraron ambos marcadores, calcular control
                    if robot_found and target_found:
                        # Guardar posición actual
                        self.last_robot_pos = robot_pos_mm
                        self.last_target_pos = target_pos_mm
                        
                        # Calcular control de convergencia
                        self.t[self.i] = self.i
                        self.x[self.i] = robot_pos_mm[0]
                        self.y[self.i] = robot_pos_mm[1]
                        self.th[self.i] = robot_angle
                        
                        # Ajustar posición usando offset l
                        self.x[self.i] = self.x[self.i] + (self.l * np.cos(self.th[self.i]))
                        self.y[self.i] = self.y[self.i] + (self.l * np.sin(self.th[self.i]))
                        
                        # Matriz de transformación cinemática
                        A = np.array([[np.cos(self.th[self.i]), -self.l * np.sin(self.th[self.i])],
                                     [np.sin(self.th[self.i]), self.l * np.cos(self.th[self.i])]])
                        
                        # Calcular errores y señales de control
                        self.ex[self.i], self.ey[self.i], self.ux[self.i], self.uy[self.i] = self.convergencia(
                            self.x[self.i], self.y[self.i], 
                            target_pos_mm[0], target_pos_mm[1]
                        )
                        
                        # Vector de velocidad deseada
                        B = np.array([self.ux[self.i], self.uy[self.i]])
                        
                        # Resolver sistema para obtener velocidades del robot
                        try:
                            U = np.linalg.solve(A, B)
                            V = U[0]  # Velocidad lineal
                            W = U[1]  # Velocidad angular
                            
                            # Calcular velocidades de las ruedas (derecha e izquierda)
                            wd = (V / self.r) + ((self.L * W) / (2 * self.r))
                            wi = (V / self.r) - ((self.L * W) / (2 * self.r))
                            
                            # Limitar velocidades
                            wd = max(-self.wmax, min(self.wmax, wd))
                            wi = max(-self.wmax, min(self.wmax, wi))
                            
                            # Convertir a PWM (asumiendo escala lineal)
                            pwm_d = int(wd / self.wmax * 255)
                            pwm_i = int(wi / self.wmax * 255)
                            
                            # Ajustar PWM mínimo
                            pwm_d = self.ajustar_pwm(pwm_d)
                            pwm_i = self.ajustar_pwm(pwm_i)
                            
                            # Enviar comandos al robot
                            self.bt_controller.send_motor_speeds(pwm_d, pwm_i)
                            
                            # Mostrar información en pantalla
                            cv2.putText(frame, f"PWM D: {pwm_d}, I: {pwm_i}", 
                                       (30, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                            cv2.putText(frame, f"Error: ({int(self.ex[self.i])}, {int(self.ey[self.i])})", 
                                       (30, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                            
                            # Incrementar índice de arreglos
                            self.i = (self.i + 1) % self.max_points
                            
                        except np.linalg.LinAlgError:
                            print("Error al resolver el sistema de ecuaciones")
                            self.bt_controller.stop_motors()
                    
                    else:
                        # Si no se encuentra alguno de los marcadores, detener el robot
                        if robot_found or target_found:
                            cv2.putText(frame, "Buscando marcadores...", 
                                       (30, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                        self.bt_controller.stop_motors()
                
                else:
                    # No se detectaron marcadores, detener el robot
                    cv2.putText(frame, "No se detectaron marcadores", 
                               (30, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    self.bt_controller.stop_motors()
                
                # Dibujar centro de coordenadas
                cv2.line(frame, (self.center_x - 20, self.center_y), (self.center_x + 20, self.center_y), (0, 0, 255), 2)
                cv2.line(frame, (self.center_x, self.center_y - 20), (self.center_x, self.center_y + 20), (0, 0, 255), 2)
                cv2.putText(frame, "(0,0)", (self.center_x + 30, self.center_y - 10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                
                # Mostrar frame
                cv2.putText(frame, "Presiona ESC para salir", (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                cv2.imshow('Robot Control', frame)
                
                if cv2.waitKey(1) & 0xFF == 27:  # Presiona ESC para salir
                    break
                
        finally:
            # Rutina de cierre
            print("Cerrando conexiones...")
            self.bt_controller.stop_motors()  # Detener motores
            self.bt_controller.disconnect()   # Desconectar Bluetooth
            self.tracker.detector.close()     # Cerrar cámara y ventanas
            print("Sistema cerrado correctamente")

class RobotObstacleAvoidanceController(RobotController):
    """
    Controlador para robot móvil con evitación de obstáculos 
    """
    def __init__(self, robot_id=0, target_id=1, 
                 bluetooth_mac="88:13:BF:70:40:72", 
                 k=0.5, k_rep=2.0, safety_radius=50, target_threshold=10, visual=True):
        """
        Inicializa el controlador del robot con evitación de obstáculos
        
        Args:
            robot_id: ID del marcador ArUco del robot
            target_id: ID del marcador ArUco del objetivo
            bluetooth_mac: Dirección MAC del dispositivo Bluetooth del robot
            k: Ganancia del controlador proporcional (atractiva)
            k_rep: Ganancia del controlador repulsivo para obstáculos
            safety_radius: Radio de seguridad alrededor de obstáculos en píxeles
            target_threshold: Umbral de distancia en mm para considerar que se ha alcanzado el objetivo
            visual: Si se debe mostrar la interfaz visual
        """
        # Inicializar la clase padre
        super().__init__(robot_id, target_id, bluetooth_mac, k, visual)
        
        # Reemplazar el tracker normal por uno con detección de obstáculos
        self.tracker = RobotTargetObstacleTracker(
            robot_id=robot_id, 
            target_id=target_id,
            robot_offset_px=-220, 
            target_offset_px=83,
            robot_offset_mm=-320, 
            target_offset_mm=138, 
            safety_radius=safety_radius,
            visual=visual
        )
        
        # Parámetros adicionales para evitación de obstáculos
        self.k_rep = k_rep  # Ganancia repulsiva
        self.obstacle_avoidance_active = True
        self.safety_radius_mm = safety_radius * self.tracker.mm_per_px  # Convertir a mm
        self.target_threshold = target_threshold  # Umbral para considerar objetivo alcanzado
        self.target_reached = False  # Flag para indicar si se alcanzó el objetivo
        
    def campo_potencial_repulsivo(self, robot_pos, obstacle_pos, d0):
        """
        Calcula el campo potencial repulsivo para un obstáculo
        
        Args:
            robot_pos: Posición del robot en (x, y) en mm
            obstacle_pos: Posición del obstáculo en (x, y) en mm
            d0: Distancia de influencia del obstáculo en mm
            
        Returns:
            Tupla (fx, fy) con las fuerzas repulsivas en x e y
        """
        # Calcular distancia al obstáculo
        dx = robot_pos[0] - obstacle_pos[0]
        dy = robot_pos[1] - obstacle_pos[1]
        dist = np.sqrt(dx**2 + dy**2)
        
        # Si la distancia es mayor que d0, no hay fuerza repulsiva
        if dist > d0:
            return 0, 0
            
        # Calcular fuerza repulsiva
        # La fuerza aumenta a medida que nos acercamos al obstáculo
        factor = self.k_rep * (1/dist - 1/d0) * (1/dist**2)
        
        # Dirección de la fuerza (desde el obstáculo hacia el robot)
        if dist > 0:  # Evitar división por cero
            fx = factor * (dx / dist)
            fy = factor * (dy / dist)
        else:
            # Si están en la misma posición (improbable), empujar en dirección aleatoria
            angle = np.random.uniform(0, 2*np.pi)
            fx = factor * np.cos(angle)
            fy = factor * np.sin(angle)
            
        return fx, fy
        
    def control_con_obstaculos(self, robot_pos, target_pos, obstacles):
        """
        Calcula el control combinando atracción al objetivo y repulsión de obstáculos
        
        Args:
            robot_pos: Posición del robot en (x, y) en mm
            target_pos: Posición del objetivo en (x, y) en mm
            obstacles: Diccionario de obstáculos {id: {"mm_position": (x, y)}}
            
        Returns:
            Tupla (ex, ey, ux, uy) con errores y señales de control
        """
        # Cálculo de error (igual que antes)
        ex = robot_pos[0] - target_pos[0]
        ey = robot_pos[1] - target_pos[1]
        
        # Control proporcional (atractivo)
        ux = -self.k * ex
        uy = -self.k * ey
        
        # Sumar fuerzas repulsivas de todos los obstáculos
        fx_total = 0
        fy_total = 0
        
        for obstacle_id, data in obstacles.items():
            # Obtener posición del obstáculo
            obstacle_pos = data["mm_position"]
            
            # Calcular fuerza repulsiva de este obstáculo
            fx, fy = self.campo_potencial_repulsivo(
                robot_pos, 
                obstacle_pos, 
                self.safety_radius_mm * 2  # Radio de influencia = 2*safety_radius
            )
            
            # Acumular fuerzas
            fx_total += fx
            fy_total += fy
        
        # Combinar fuerzas atractivas y repulsivas
        ux += fx_total
        uy += fy_total
        
        return ex, ey, ux, uy
    
    def check_target_reached(self, robot_pos, target_pos):
        """
        Verifica si el robot ha alcanzado el objetivo
        
        Args:
            robot_pos: Posición del robot en (x, y) en mm
            target_pos: Posición del objetivo en (x, y) en mm
            
        Returns:
            Boolean indicando si se alcanzó el objetivo
        """
        # Calcular distancia entre robot y objetivo
        dx = robot_pos[0] - target_pos[0]
        dy = robot_pos[1] - target_pos[1]
        dist = np.sqrt(dx**2 + dy**2)
        
        # Verificar si la distancia es menor que el umbral
        return dist < self.target_threshold
    
    def run(self):
        """
        Ejecuta el bucle principal de control del robot con evitación de obstáculos
        """
        # Inicializar cámara a través del tracker
        if not self.tracker.detector.initialize_camera():
            print("Error al inicializar la cámara. Saliendo...")
            return
        
        # Establecer conexión Bluetooth
        if not self.connect():
            print("Error al conectar con el robot. Saliendo...")
            self.tracker.detector.close()
            return
        
        print("Conexión con robot establecida. Iniciando control...")
        
        # Obtener dimensiones de resolución
        self.center_x = self.tracker.detector.resolution_x // 2
        self.center_y = self.tracker.detector.resolution_y // 2
        
        # Establecer mm por píxel en el detector de obstáculos
        self.tracker.obstacle_detector.set_mm_per_px(self.tracker.mm_per_px)
        
        try:
            while True:
                # Obtener frame y detectar marcadores ArUco
                frame, points, ids = self.tracker.detector.buscar_aruco()
                
                if frame is None:
                    print("No se pudo obtener frame. Comprobando conexión...")
                    time.sleep(0.5)
                    continue
                
                # Procesar frame
                if points is not None and len(points) > 0 and ids is not None:
                    # Dibujar marcadores ArUco
                    self.tracker.detector.dibujar_aruco(frame, points, ids)
                    
                    # Buscar robot y objetivo
                    robot_found = False
                    target_found = False
                    
                    robot_pos_mm = None
                    target_pos_mm = None
                    robot_angle = None
                    
                    # Detectar obstáculos
                    obstacles = self.tracker.obstacle_detector.detect_obstacles(points, ids, frame, self.center_x, self.center_y)
                    
                    ids = ids.flatten()
                    for (marker_corner, marker_id) in zip(points, ids):
                        # Procesar marcadores y obtener información
                        if marker_id == self.robot_id or marker_id == self.target_id:
                            # Obtener coordenadas y ángulo
                            corners = marker_corner.reshape((4, 2))
                            (top_left, top_right, bottom_right, bottom_left) = corners
                            
                            # Convertir coordenadas a enteros
                            top_right = (int(top_right[0]), int(top_right[1]))
                            bottom_right = (int(bottom_right[0]), int(bottom_right[1]))
                            bottom_left = (int(bottom_left[0]), int(bottom_left[1]))
                            top_left = (int(top_left[0]), int(top_left[1]))
                            
                            # Calcular punto medio
                            mid_x = (top_right[0] + bottom_left[0]) // 2
                            mid_y = (top_right[1] + bottom_left[1]) // 2
                            
                            # Calcular ángulo
                            x = (bottom_right[0] - bottom_left[0])
                            y = (bottom_right[1] - bottom_left[1])
                            angle = math.atan2(y, x)
                            
                            if marker_id == self.robot_id:
                                # Calcular POI (punto de interés) del robot
                                robot_poi_x = int(mid_x + self.tracker.robot_offset_px * np.cos(angle))
                                robot_poi_y = int(mid_y + self.tracker.robot_offset_px * np.sin(angle))
                                
                                # Convertir a mm
                                robot_x_mm = (robot_poi_x - self.center_x) * self.tracker.mm_per_px
                                robot_y_mm = (robot_poi_y - self.center_y) * self.tracker.mm_per_px
                                
                                robot_pos_mm = (robot_x_mm, robot_y_mm)
                                robot_angle = angle
                                robot_found = True
                                
                                # Dibujar POI del robot
                                cv2.circle(frame, (robot_poi_x, robot_poi_y), 10, (0, 255, 0), -1)
                                cv2.putText(frame, f"Robot: {int(robot_x_mm)},{int(robot_y_mm)}", 
                                           (robot_poi_x + 10, robot_poi_y - 10), 
                                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                                
                            elif marker_id == self.target_id:
                                # Calcular POI del objetivo
                                target_poi_x = int(mid_x + self.tracker.target_offset_px * np.cos(angle))
                                target_poi_y = int(mid_y + self.tracker.target_offset_px * np.sin(angle))
                                
                                # Convertir a mm
                                target_x_mm = (target_poi_x - self.center_x) * self.tracker.mm_per_px
                                target_y_mm = (target_poi_y - self.center_y) * self.tracker.mm_per_px
                                
                                target_pos_mm = (target_x_mm, target_y_mm)
                                target_found = True
                                
                                # Dibujar POI del objetivo
                                cv2.circle(frame, (target_poi_x, target_poi_y), 10, (255, 0, 0), -1)
                                cv2.putText(frame, f"Target: {int(target_x_mm)},{int(target_y_mm)}", 
                                           (target_poi_x + 10, target_poi_y - 10), 
                                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
                    
                    # Si se encontraron ambos marcadores, calcular control
                    if robot_found and target_found:
                        # Guardar posición actual
                        self.last_robot_pos = robot_pos_mm
                        self.last_target_pos = target_pos_mm
                        
                        # Verificar si se alcanzó el objetivo
                        prev_target_reached = self.target_reached  # Guardar estado anterior
                        self.target_reached = self.check_target_reached(robot_pos_mm, target_pos_mm)
                        
                        # Notificar si se acaba de alcanzar el objetivo
                        if self.target_reached and not prev_target_reached:
                            print("¡OBJETIVO ALCANZADO!")
                            # Detener motores al alcanzar el objetivo
                            self.bt_controller.stop_motors()
                        
                        # Mostrar estado de objetivo en pantalla
                        target_status = "ALCANZADO" if self.target_reached else "NO ALCANZADO"
                        cv2.putText(frame, f"Objetivo: {target_status}", 
                                   (30, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.7, 
                                   (0, 255, 0) if self.target_reached else (0, 0, 255), 2)
                        
                        # Si no se ha alcanzado el objetivo, calcular control
                        if not self.target_reached:
                            # Calcular control considerando obstáculos
                            self.t[self.i] = self.i
                            self.x[self.i] = robot_pos_mm[0]
                            self.y[self.i] = robot_pos_mm[1]
                            self.th[self.i] = robot_angle
                            
                            # Ajustar posición usando offset l
                            self.x[self.i] = self.x[self.i] + (self.l * np.cos(self.th[self.i]))
                            self.y[self.i] = self.y[self.i] + (self.l * np.sin(self.th[self.i]))
                            
                            # Matriz de transformación cinemática
                            A = np.array([[np.cos(self.th[self.i]), -self.l * np.sin(self.th[self.i])],
                                         [np.sin(self.th[self.i]), self.l * np.cos(self.th[self.i])]])
                            
                            # Calcular errores y señales de control con obstáculos
                            self.ex[self.i], self.ey[self.i], self.ux[self.i], self.uy[self.i] = self.control_con_obstaculos(
                                (self.x[self.i], self.y[self.i]), 
                                target_pos_mm,
                                obstacles
                            )
                            
                            # Vector de velocidad deseada
                            B = np.array([self.ux[self.i], self.uy[self.i]])
                            
                            # Resolver sistema para obtener velocidades del robot
                            try:
                                U = np.linalg.solve(A, B)
                                V = U[0]  # Velocidad lineal
                                W = U[1]  # Velocidad angular
                                
                                # Calcular velocidades de las ruedas (derecha e izquierda)
                                wd = (V / self.r) + ((self.L * W) / (2 * self.r))
                                wi = (V / self.r) - ((self.L * W) / (2 * self.r))
                                
                                # Limitar velocidades
                                wd = max(-self.wmax, min(self.wmax, wd))
                                wi = max(-self.wmax, min(self.wmax, wi))
                                
                                # Convertir a PWM (asumiendo escala lineal)
                                pwm_d = int(wd / self.wmax * 255)
                                pwm_i = int(wi / self.wmax * 255)
                                
                                # Ajustar PWM mínimo
                                pwm_d = self.ajustar_pwm(pwm_d)
                                pwm_i = self.ajustar_pwm(pwm_i)
                                
                                # Enviar comandos al robot
                                self.bt_controller.send_motor_speeds(pwm_d, pwm_i)
                                
                                # Mostrar información en pantalla
                                cv2.putText(frame, f"PWM D: {pwm_d}, I: {pwm_i}", 
                                           (30, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                                cv2.putText(frame, f"Error: ({int(self.ex[self.i])}, {int(self.ey[self.i])})", 
                                           (30, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                                
                                # Incrementar índice de arreglos
                                self.i = (self.i + 1) % self.max_points
                                
                            except np.linalg.LinAlgError:
                                print("Error al resolver el sistema de ecuaciones")
                                self.bt_controller.stop_motors()
                        
                        # Dibujar línea de robot a objetivo
                        if frame is not None and robot_found and target_found:
                            robot_poi_x = int(self.center_x + robot_pos_mm[0] / self.tracker.mm_per_px)
                            robot_poi_y = int(self.center_y + robot_pos_mm[1] / self.tracker.mm_per_px)
                            target_poi_x = int(self.center_x + target_pos_mm[0] / self.tracker.mm_per_px)
                            target_poi_y = int(self.center_y + target_pos_mm[1] / self.tracker.mm_per_px)
                            
                            # Dibujar línea con color dependiendo del estado
                            line_color = (0, 255, 0) if self.target_reached else (255, 255, 255)
                            cv2.line(frame, (robot_poi_x, robot_poi_y), (target_poi_x, target_poi_y), line_color, 2)
                            
                            # Verificar si hay obstáculos en el camino y mostrar estado
                            is_clear, blocking_obstacles = self.tracker.obstacle_detector.is_path_clear(
                                robot_pos_mm, target_pos_mm
                            )
                            path_status = "LIBRE" if is_clear else f"BLOQUEADO por {blocking_obstacles}"
                            cv2.putText(frame, f"Camino: {path_status}", 
                                       (30, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.7, 
                                       (0, 255, 0) if is_clear else (0, 0, 255), 2)
                    
                    else:
                        # Si no se encuentra alguno de los marcadores, detener el robot
                        if robot_found or target_found:
                            cv2.putText(frame, "Buscando marcadores...", 
                                       (30, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                        self.bt_controller.stop_motors()
                
                else:
                    # No se detectaron marcadores, detener el robot
                    cv2.putText(frame, "No se detectaron marcadores", 
                               (30, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    self.bt_controller.stop_motors()
                
                # Dibujar centro de coordenadas
                cv2.line(frame, (self.center_x - 20, self.center_y), (self.center_x + 20, self.center_y), (0, 0, 255), 2)
                cv2.line(frame, (self.center_x, self.center_y - 20), (self.center_x, self.center_y + 20), (0, 0, 255), 2)
                cv2.putText(frame, "(0,0)", (self.center_x + 30, self.center_y - 10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                
                # Mostrar frame
                cv2.putText(frame, "Presiona ESC para salir", (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                cv2.imshow('Robot Control with Obstacles', frame)
                #put robot motors on stop
                time.sleep(0.01)
                self.bt_controller.stop_motors()
                
                if cv2.waitKey(1) & 0xFF == 27:  # Presiona ESC para salir
                    break
                
        finally:
            # Rutina de cierre
            print("Cerrando conexiones...")
            self.bt_controller.stop_motors()  # Detener motores
            self.bt_controller.disconnect()   # Desconectar Bluetooth
            self.tracker.detector.close()     # Cerrar cámara y ventanas
            print("Sistema cerrado correctamente")


# Ejemplo de uso
if __name__ == "__main__":
    # Crear controlador con parámetros específicos
    controller = RobotObstacleAvoidanceController(
        robot_id=0,
        target_id=1,
        bluetooth_mac="88:13:BF:70:40:72",  # Ajustar según tu dispositivo
        k=5.0,                              # Ganancia proporcional atractiva
        k_rep=5.0,                         # Ganancia proporcional repulsiva
        safety_radius=80,                  # Radio de seguridad en píxeles
        target_threshold=15,               # Umbral para considerar objetivo alcanzado en mm
        visual=True
    )
    
    # Ejecutar controlador
    controller.run()