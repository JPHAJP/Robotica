#!/usr/bin/env python3
import cv2
import numpy as np
import math
import time
import socket
import struct
import sys
import os
import warnings

# Suprimir advertencias de OpenCV que causan "Corrupt JPEG data" mensajes
warnings.filterwarnings("ignore", category=UserWarning)
os.environ["OPENCV_LOG_LEVEL"] = "SILENT"  # Intenta silenciar mensajes de OpenCV

class RobotController:
    def __init__(self, bt_addr, camera_index=0, resolution=(1280, 720), camera_height=225):
        """
        Inicializar el controlador del robot
        
        Args:
            bt_addr: Dirección MAC del dispositivo Bluetooth ESP32
            camera_index: Índice de la cámara (por defecto 0)
            resolution: Resolución de la cámara (ancho, alto)
            camera_height: Altura de la cámara en cm
        """
        self.bt_addr = bt_addr
        self.bt_port = 1  # Puerto RFCOMM
        self.bt_socket = None
        self.camera = None
        self.camera_index = camera_index
        self.resolution_x, self.resolution_y = resolution
        
        # Parámetros del robot
        self.wheel_radius = 6  # Radio de la rueda en cm
        self.wheel_distance = 20  # Distancia entre centros de las ruedas en cm
        self.center_to_wheel = 10  # Distancia del centro del robot al centro de las ruedas en cm
        
        # Offset entre el aruco y la punta del robot (en cm)
        self.robot_front_offset = -25  # -25 cm en X (el frente está en dirección -X del aruco)
        
        # Configuración PWM
        self.max_pwm = 255  # PWM máximo
        self.min_pwm = 180  # PWM mínimo para mover el robot
        
        # Escala de píxeles a cm (considerando altura de la cámara)
        self.pixel_to_cm = self._calculate_pixel_to_cm_ratio(camera_height)
        
        # Parámetros de control
        self.k = 0.5  # Ganancia proporcional
        
        # Estado actual del robot
        self.robot_id = 0  # ID del marcador ArUco que identifica al robot (cambiado a 0)
        self.target_id = 5  # ID del marcador ArUco que identifica al objetivo (establecido a 5)
        
        self.robot_state = {
            'x': 0,      # Posición x en píxeles (relativo al centro)
            'y': 0,      # Posición y en píxeles (relativo al centro)
            'theta': 0,  # Orientación en radianes
        }
        
        # Estado del objetivo
        self.target_state = {
            'x': 0,      # Posición x en píxeles (relativo al centro)
            'y': 0,      # Posición y en píxeles (relativo al centro)
            'theta': 0,  # Orientación en radianes
            'x_axis': [0, 0],  # Vector dirección del eje x del objetivo
            'y_axis': [0, 0],  # Vector dirección del eje y del objetivo
        }
        
        # Punto objetivo
        self.target = {
            'x': 0,  # Posición x objetivo (relativo al centro)
            'y': 0   # Posición y objetivo (relativo al centro)
        }
        
        # Parámetros para alineación y proximidad
        self.alignment_threshold = 0.1  # Tolerancia para alineación angular (radianes)
        self.distance_target = 5.0  # Distancia objetivo en cm
        self.distance_tolerance = 0.5  # Tolerancia de 10% de 5cm = 0.5cm
        
        # Control de tiempo para impresión
        self.last_print_time = 0
        self.print_interval = 0.5  # Intervalo de impresión en segundos
        
        # Estado de finalización
        self.mission_completed = False
    
    def _calculate_pixel_to_cm_ratio(self, camera_height):
        """
        Calcula la relación de píxeles a cm basado en la altura de la cámara
        y el tamaño del robot conocido
        """
        # Valor aproximado basado en la altura de la cámara
        # En la realidad, esto debería calibrarse con mediciones reales
        return 10.0 / camera_height  # Ajustable según sea necesario
    
    def connect_bluetooth(self):
        """
        Establece conexión Bluetooth con el ESP32
        """
        try:
            # Crear socket Bluetooth RFCOMM
            self.bt_socket = socket.socket(
                socket.AF_BLUETOOTH,
                socket.SOCK_STREAM,
                socket.BTPROTO_RFCOMM
            )
            
            print(f"Conectando a {self.bt_addr}:{self.bt_port} ...")
            self.bt_socket.connect((self.bt_addr, self.bt_port))
            print("✅ Conexión Bluetooth establecida")
            return True
        except OSError as e:
            print(f"❌ Error al conectar por Bluetooth: {e}")
            return False
    
    def disconnect_bluetooth(self):
        """
        Cierra la conexión Bluetooth
        """
        if self.bt_socket:
            self.bt_socket.close()
            self.bt_socket = None
            print("🔌 Conexión Bluetooth cerrada")
    
    def initialize_camera(self):
        """
        Inicializa la cámara con manejo mejorado de errores
        """
        capture = None
        
        # Intenta con acceso directo al dispositivo primero (más confiable en Linux)
        for i in range(10):  # Revisa hasta 10 dispositivos de video posibles
            device_path = f"/dev/video{i}"
            try:
                if os.path.exists(device_path):
                    print(f"Dispositivo encontrado: {device_path}, intentando abrir...")
                    
                    # Intenta formato MJPEG primero (a menudo más compatible)
                    capture = cv2.VideoCapture(device_path)
                    if capture.isOpened():
                        # Fuerza formato MJPEG para Linux
                        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                        print(f"Dispositivo {device_path} abierto exitosamente")
                        break
            except Exception as e:
                print(f"Error con {device_path}: {e}")
        
        # Si el acceso directo al dispositivo no funcionó, intenta con índices estándar
        if capture is None or not capture.isOpened():
            for camera_index in range(3):
                print(f"Intentando abrir cámara {camera_index}...")
                
                # Intenta con VideoCapture(index)
                try:
                    capture = cv2.VideoCapture(camera_index)
                    if capture.isOpened():
                        print(f"Cámara {camera_index} abierta exitosamente")
                        # Establece tamaño del buffer al mínimo para reducir latencia
                        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                        break
                except Exception as e:
                    print(f"Error con método estándar: {e}")
                    
                # Intenta con V4L2
                try:
                    capture = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
                    if capture.isOpened():
                        print(f"Cámara {camera_index} abierta exitosamente con V4L2")
                        # Establece tamaño del buffer al mínimo para reducir latencia
                        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                        break
                except Exception as e:
                    print(f"Error con V4L2: {e}")
        
        # Si no se pudo abrir ninguna cámara
        if capture is None or not capture.isOpened():
            print("No se pudo abrir ninguna cámara. Verifica la conexión.")
            return None
        
        # Configura propiedades de la cámara
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution_x)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution_y)
        
        # Para compatibilidad con V4L2, usar MJPG para evitar errores de JPEG
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        capture.set(cv2.CAP_PROP_FPS, 30)
        
        # Da tiempo a la cámara para inicializarse
        time.sleep(2)
        
        # Verifica que la cámara funciona tratando de leer un frame
        for i in range(5):
            ret, test_frame = capture.read()
            if ret and test_frame is not None:
                print("Prueba de cámara exitosa - capaz de leer frames")
                
                # Ahora intenta establecer mayor resolución si es necesario
                capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution_x)
                capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution_y)
                self.camera = capture
                return True
            else:
                print(f"Intento de prueba de cámara {i+1}/5 falló, reintentando...")
                time.sleep(1)
        
        print("Error al leer frames de la cámara después de la inicialización")
        return False
    
    def release_camera(self):
        """
        Libera la cámara
        """
        if self.camera:
            self.camera.release()
            self.camera = None
            print("📷 Cámara liberada")
    
    def change_brightness(self, img, value):
        """
        Modifica imagen, ajusta el brillo
        """
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)
        v = cv2.add(v, value)
        v[v > 255] = 255
        v[v < 0] = 0
        final_hsv = cv2.merge((h, s, v))
        img = cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)
        return img
    
    def get_mid_points(self, pt1, pt2):
        """
        Obtiene el punto medio entre dos puntos
        """
        return [(pt1[0] + pt2[0]) / 2, (pt1[1] + pt2[1]) / 2]
    
    def get_angle_rad(self, bottom_right, bottom_left):
        """
        Obtiene el ángulo de rotación de los códigos aruco en radianes
        """
        x = (bottom_right[0] - bottom_left[0])
        y = (bottom_right[1] - bottom_left[1])
        angle = math.atan2(y, x)
        angle = round(angle, 2)
        return angle
    
    def get_coordinates(self, marker_corner):
        """
        Obtiene las coordenadas del marcador ArUco
        """
        corners = marker_corner.reshape((4, 2))
        (top_left, top_right, bottom_right, bottom_left) = corners
        
        # Convierte cada par de coordenadas (x, y) a enteros
        top_right = (int(top_right[0]), int(top_right[1]))
        bottom_right = (int(bottom_right[0]), int(bottom_right[1]))
        bottom_left = (int(bottom_left[0]), int(bottom_left[1]))
        top_left = (int(top_left[0]), int(top_left[1]))
        
        return top_left, top_right, bottom_left, bottom_right
    
    def draw_aruco(self, frame, top_left, top_right, bottom_left, bottom_right, mid_p, x, y, angle, marker_id):
        """
        Dibuja información sobre el marcador ArUco en el frame
        """
        # Dibuja el cuadrado
        cv2.line(frame, top_left, top_right, (0, 255, 0), 2)
        cv2.line(frame, top_right, bottom_right, (0, 255, 0), 2)
        cv2.line(frame, bottom_right, bottom_left, (0, 255, 0), 2)
        cv2.line(frame, bottom_left, top_left, (0, 255, 0), 2)
        
        # Dibuja las líneas de orientación
        line_thickness = 3
        cv2.line(frame, tuple(map(int, x)), tuple(map(int, mid_p)), (0, 0, 255), thickness=line_thickness)
        cv2.line(frame, tuple(map(int, y)), tuple(map(int, mid_p)), (255, 0, 0), thickness=line_thickness)
        
        # Imprime información de posición
        pos_x = mid_p[0] - (self.resolution_x / 2)
        pos_y = mid_p[1] - (self.resolution_y / 2)
        cv2.putText(frame, f"[{pos_x:.1f}, {pos_y:.1f}]", 
                   (int(mid_p[0]), int(mid_p[1]) - 150), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 204, 204), 2, cv2.LINE_AA)
        cv2.putText(frame, f"ID: {marker_id}", 
                   (int(mid_p[0]) - 50, int(mid_p[1]) - 100), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 204, 204), 2, cv2.LINE_AA)
        cv2.putText(frame, f"{math.degrees(angle):.1f} grados", 
                   (int(mid_p[0]) - 400, int(mid_p[1]) - 200), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 204, 204), 2, cv2.LINE_AA)
    
    def draw_text_title(self, frame, text, color):
        """
        Dibuja texto de título en el frame
        """
        cv2.putText(frame, text, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2, cv2.LINE_AA)
    
    def draw_point(self, frame, text, color, pos_x, pos_y):
        """
        Dibuja un punto en el frame con texto
        """
        rel_x = pos_x - self.resolution_x / 2
        rel_y = pos_y - self.resolution_y / 2
        cv2.putText(frame, text, (pos_x + 20, pos_y + 20), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2, cv2.LINE_AA)
        cv2.putText(frame, f"{rel_x:.1f}, {rel_y:.1f}", 
                   (pos_x + 20, pos_y + 50), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2, cv2.LINE_AA)
        cv2.circle(frame, (pos_x, pos_y), 10, color, 5)
    
    def detect_aruco(self):
        """
        Detecta marcadores ArUco en el frame actual
        """
        if self.camera is None or not self.camera.isOpened():
            return None, None, None
        
        ret, frame = self.camera.read()
        if not ret or frame is None:
            return None, None, None
        
        frame = cv2.resize(frame, (self.resolution_x, self.resolution_y))
        frame = self.change_brightness(frame, 10)
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
        aruco_params = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
        points, ids, rejected = detector.detectMarkers(gray)
        
        return frame, points, ids
    
    def update_states(self, points, ids):
        """
        Actualiza las posiciones del robot y del objetivo basado en la detección de marcadores ArUco
        """
        robot_detected = False
        target_detected = False
        
        if points is not None and len(points) > 0 and ids is not None:
            # Aplana la lista de IDs de ArUco
            ids = ids.flatten()
            
            # Busca el marcador correspondiente al robot y al objetivo
            for (marker_corner, marker_id) in zip(points, ids):
                if marker_id == self.robot_id:
                    top_left, top_right, bottom_left, bottom_right = self.get_coordinates(marker_corner)
                    
                    # Calcula el punto medio
                    mid_point = self.get_mid_points(top_right, bottom_left)
                    
                    # Actualiza el estado del robot
                    self.robot_state['x'] = mid_point[0] - (self.resolution_x / 2)
                    self.robot_state['y'] = mid_point[1] - (self.resolution_y / 2)
                    self.robot_state['theta'] = self.get_angle_rad(bottom_right, bottom_left)
                    
                    # Considera el offset del frente del robot en coordenadas del robot
                    robot_detected = True
                    
                elif marker_id == self.target_id:
                    top_left, top_right, bottom_left, bottom_right = self.get_coordinates(marker_corner)
                    
                    # Calcula el punto medio
                    mid_point = self.get_mid_points(top_right, bottom_left)
                    
                    # Actualiza el estado del objetivo
                    self.target_state['x'] = mid_point[0] - (self.resolution_x / 2)
                    self.target_state['y'] = mid_point[1] - (self.resolution_y / 2)
                    self.target_state['theta'] = self.get_angle_rad(bottom_right, bottom_left)
                    
                    # Calcula los ejes del objetivo en coordenadas globales
                    length = 50  # Longitud de visualización del eje
                    self.target_state['x_axis'] = [
                        length * math.cos(self.target_state['theta']),
                        length * math.sin(self.target_state['theta'])
                    ]
                    self.target_state['y_axis'] = [
                        length * math.cos(self.target_state['theta'] + math.pi/2),
                        length * math.sin(self.target_state['theta'] + math.pi/2)
                    ]
                    
                    target_detected = True
        
        return robot_detected, target_detected
    
    def calculate_alignment_target(self):
        """
        Calcula el punto objetivo basado en la alineación con los ejes del objetivo
        """
        # Vector desde el objetivo hacia el robot (para determinar qué eje usar)
        dx = self.robot_state['x'] - self.target_state['x']
        dy = self.robot_state['y'] - self.target_state['y']
        
        # Calcular ángulos con respecto a los ejes del objetivo
        angle_to_x_axis = math.atan2(dy, dx) - self.target_state['theta']
        angle_to_y_axis = math.atan2(dy, dx) - (self.target_state['theta'] + math.pi/2)
        
        # Normalizar ángulos
        angle_to_x_axis = (angle_to_x_axis + math.pi) % (2 * math.pi) - math.pi
        angle_to_y_axis = (angle_to_y_axis + math.pi) % (2 * math.pi) - math.pi
        
        # Determinar qué eje está más alineado con la dirección robot-objetivo
        if abs(angle_to_x_axis) < abs(angle_to_y_axis):
            # Alinear con eje X del objetivo
            target_angle = self.target_state['theta']
            target_direction = [math.cos(target_angle), math.sin(target_angle)]
        else:
            # Alinear con eje Y del objetivo
            target_angle = self.target_state['theta'] + math.pi/2
            target_direction = [math.cos(target_angle), math.sin(target_angle)]
        
        # Calcular punto objetivo a 5cm del objetivo en la dirección del eje elegido
        self.target['x'] = self.target_state['x'] + (self.distance_target / self.pixel_to_cm) * target_direction[0]
        self.target['y'] = self.target_state['y'] + (self.distance_target / self.pixel_to_cm) * target_direction[1]
        
        return target_angle, target_direction
    
    def control_proportional(self, x, y, x_target, y_target, k):
        """
        Control proporcional para convergencia a un punto
        """
        # Cálculo de error
        ex = x - x_target
        ey = y - y_target
        
        # Control proporcional
        ux = -k * ex
        uy = -k * ey
        
        return ex, ey, ux, uy
    
    def move_robot(self, x, y, theta, x_target, y_target):
        """
        Calcula velocidades para mover el robot hacia el punto objetivo
        """
        # Primero, ajustamos la posición del robot para considerar el offset del frente
        # El offset se aplica en las coordenadas locales del robot
        x_adjusted = x + self.robot_front_offset * math.cos(theta) / self.pixel_to_cm
        y_adjusted = y + self.robot_front_offset * math.sin(theta) / self.pixel_to_cm
        
        # Cálculo de errores y vector de velocidad
        ex, ey, ux, uy = self.control_proportional(x_adjusted, y_adjusted, x_target, y_target, self.k)
        
        # Calcular distancia al objetivo
        distance = math.sqrt(ex**2 + ey**2) * self.pixel_to_cm  # Convertir a cm
        
        # Verificar si hemos llegado al objetivo (distancia 5cm ±10%)
        if distance <= self.distance_target + self.distance_tolerance and \
           distance >= self.distance_target - self.distance_tolerance:
            # Hemos llegado al objetivo
            if not self.mission_completed:
                print("🎯 OBJETIVO ALCANZADO: Robot a 5cm (±10%) del objetivo y alineado")
                self.mission_completed = True
            return 0, 0  # Detener el robot
        else:
            self.mission_completed = False
        
        # Matriz de transformación
        A = np.array([
            [math.cos(theta), -self.center_to_wheel * math.sin(theta)],
            [math.sin(theta), self.center_to_wheel * math.cos(theta)]
        ])
        
        # Vector de velocidad
        B = np.array([ux, uy])
        
        # Resuelve el sistema para obtener velocidades lineal y angular
        try:
            U = np.linalg.solve(A, B)
            V = U[0]  # Velocidad lineal
            W = U[1]  # Velocidad angular
            
            # Calcular velocidades de las ruedas usando el modelo cinemático
            wd = (V / self.wheel_radius) + ((self.wheel_distance * W) / (2 * self.wheel_radius))
            wi = (V / self.wheel_radius) - ((self.wheel_distance * W) / (2 * self.wheel_radius))
            
            # Convertir de velocidad angular a PWM en el rango [min_pwm, max_pwm]
            # Asegurar que los valores estén en el rango correcto, teniendo en cuenta el valor mínimo de PWM
            def angular_to_pwm(w):
                if abs(w) < 0.1:  # Si la velocidad es muy baja, detener
                    return 0
                
                # Normalizar al rango [0, 1] y luego escalar al rango [min_pwm, max_pwm]
                pwm = int(self.min_pwm + (self.max_pwm - self.min_pwm) * min(abs(w) / self.max_speed, 1.0))
                return pwm if w >= 0 else -pwm
            
            wd_pwm = angular_to_pwm(wd)
            wi_pwm = angular_to_pwm(wi)
            
            return wd_pwm, wi_pwm
            
        except np.linalg.LinAlgError:
            print("Error en la solución del sistema lineal")
            return 0, 0
    
    def send_motor_command(self, wd, wi):
        """
        Envía comandos de velocidad al robot a través de Bluetooth
        siguiendo el protocolo del ESP32
        """
        if self.bt_socket:
            try:
                # Convertir PWM a valor absoluto y signo
                wd_abs = abs(wd)
                wi_abs = abs(wi)
                
                # Asegurarse de que los valores estén en el rango correcto
                wd_abs = min(max(wd_abs, 0), self.max_pwm)
                wi_abs = min(max(wi_abs, 0), self.max_pwm)
                
                # Si PWM < min_pwm y no es 0, establecer en 0 (para evitar oscilaciones)
                if 0 < wd_abs < self.min_pwm:
                    wd_abs = 0
                if 0 < wi_abs < self.min_pwm:
                    wi_abs = 0
                
                # Formato de paquete: 'H' + wr(lo,hi) + wl(lo,hi)
                # wr = motor derecho, wl = motor izquierdo
                packet = struct.pack('<c hh', b'H', 
                                    wd_abs if wd >= 0 else -wd_abs, 
                                    wi_abs if wi >= 0 else -wi_abs)
                self.bt_socket.send(packet)
                return True
            except OSError as e:
                print(f"❌ Error al enviar datos: {e}")
                return False
        else:
            print("❌ No hay conexión Bluetooth establecida")
            return False
    
    def print_status(self, force=False):
        """
        Imprime información de estado cada 0.5 segundos o cuando sea forzado
        """
        current_time = time.time()
        
        if force or (current_time - self.last_print_time >= self.print_interval):
            self.last_print_time = current_time
            
            # Calcular distancia entre el frente del robot y el objetivo
            dx = self.robot_state['x'] - self.target_state['x']
            dy = self.robot_state['y'] - self.target_state['y']
            distance = math.sqrt(dx**2 + dy**2) * self.pixel_to_cm  # En cm
            
            # Añadir offset del frente
            robot_angle = self.robot_state['theta']
            front_offset_x = self.robot_front_offset * math.cos(robot_angle) / self.pixel_to_cm
            front_offset_y = self.robot_front_offset * math.sin(robot_angle) / self.pixel_to_cm
            
            dx_front = (self.robot_state['x'] + front_offset_x) - self.target_state['x']
            dy_front = (self.robot_state['y'] + front_offset_y) - self.target_state['y']
            distance_front = math.sqrt(dx_front**2 + dy_front**2) * self.pixel_to_cm  # En cm
            
            # Calcular ángulo entre los ejes
            target_angle = self.target_state['theta']
            angle_diff = robot_angle - target_angle
            angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi  # Normalizar a [-pi, pi]
            
            # Información sobre el estado actual
            print("\n" + "="*50)
            print(f"ESTADO DEL ROBOT [{time.strftime('%H:%M:%S')}]:")
            print(f"  Robot: x={self.robot_state['x']:.1f}, y={self.robot_state['y']:.1f}, θ={math.degrees(self.robot_state['theta']):.1f}°")
            print(f"  Objetivo: x={self.target_state['x']:.1f}, y={self.target_state['y']:.1f}, θ={math.degrees(self.target_state['theta']):.1f}°")
            print(f"  Distancia (frente-objetivo): {distance_front:.2f} cm (meta: {self.distance_target:.1f}±{self.distance_tolerance:.1f} cm)")
            print(f"  Diferencia angular: {math.degrees(angle_diff):.2f}°")
            print("="*50 + "\n")
    
    def draw_robot_front(self, frame, robot_x, robot_y, theta):
        """
        Dibuja una línea desde el centro del robot hasta su frente (considerando el offset)
        """
        # Convertir coordenadas relativas a absolutas (píxeles en pantalla)
        abs_x = int(robot_x + self.resolution_x / 2)
        abs_y = int(robot_y + self.resolution_y / 2)
        
        # Calcular punto del frente del robot
        front_length = abs(self.robot_front_offset) / self.pixel_to_cm  # Convertir cm a píxeles
        front_x = int(abs_x + front_length * math.cos(theta))
        front_y = int(abs_y + front_length * math.sin(theta))
        
        # Dibujar línea desde el centro del robot hasta el frente
        cv2.line(frame, (abs_x, abs_y), (front_x, front_y), (255, 0, 255), 3)
        cv2.circle(frame, (front_x, front_y), 8, (255, 0, 255), -1)
        
        # Añadir texto para indicar el frente
        cv2.putText(frame, "FRENTE", (front_x + 10, front_y - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2, cv2.LINE_AA)
        
        return front_x, front_y
    
    def draw_alignment_info(self, frame, target_angle, target_direction):
        """
        Dibuja información de alineación en el frame
        """
        # Coordenadas absolutas del objetivo
        target_x = int(self.target_state['x'] + self.resolution_x / 2)
        target_y = int(self.target_state['y'] + self.resolution_y / 2)
        
        # Dibujar el eje elegido para la alineación
        line_length = 100  # longitud de la línea en píxeles
        end_x = int(target_x + line_length * target_direction[0])
        end_y = int(target_y + line_length * target_direction[1])
        
        cv2.line(frame, (target_x, target_y), (end_x, end_y), (255, 255, 0), 3)
        
        # Dibujar punto objetivo (5cm de distancia)
        target_point_x = int(self.target['x'] + self.resolution_x / 2)
        target_point_y = int(self.target['y'] + self.resolution_y / 2)
        cv2.circle(frame, (target_point_x, target_point_y), 8, (0, 255, 255), -1)
        cv2.putText(frame, "Punto objetivo", (target_point_x + 10, target_point_y - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)
        
        # Dibujar distancia objetivo
        cv2.putText(frame, f"Distancia objetivo: {self.distance_target} cm", 
                   (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)
    
    def run(self):
        """
        Ejecuta el bucle principal del controlador
        """
        if not self.camera:
            print("❌ La cámara no está inicializada")
            return False
        
        if not self.bt_socket:
            print("❌ No hay conexión Bluetooth establecida")
            return False
        
        print("✅ Iniciando control del robot")
        print(f"🤖 Robot ID: {self.robot_id}, Objetivo ID: {self.target_id}")
        print(f"🎯 Distancia objetivo: {self.distance_target} cm (±{self.distance_tolerance} cm)")
        
        try:
            while True:
                # Detecta marcadores ArUco
                frame, points, ids = self.detect_aruco()
                
                if frame is None:
                    print("❌ No se pudo obtener frame de la cámara")
                    time.sleep(0.5)
                    continue
                
                # Actualiza posiciones del robot y objetivo
                robot_detected, target_detected = self.update_states(points, ids)
                
                # Información para visualización
                target_angle = 0
                target_direction = [1, 0]
                
                if robot_detected and target_detected:
                    # Calcular punto objetivo basado en la alineación deseada
                    target_angle, target_direction = self.calculate_alignment_target()
                    
                    # Obtiene las velocidades para el robot
                    wd, wi = self.move_robot(
                        self.robot_state['x'], 
                        self.robot_state['y'], 
                        self.robot_state['theta'],
                        self.target['x'], 
                        self.target['y']
                    )
                    
                    # Envía comandos al robot
                    self.send_motor_command(wd, wi)
                    
                    # Imprime información de estado periódicamente
                    self.print_status()
                else:
                    # Si no detecta el robot o el objetivo, detiene el robot
                    self.send_motor_command(0, 0)
                    
                    if not robot_detected:
                        if time.time() - self.last_print_time >= self.print_interval:
                            print("⚠️ Robot (ID 0) no detectado")
                            self.last_print_time = time.time()
                    
                    if not target_detected:
                        if time.time() - self.last_print_time >= self.print_interval:
                            print("⚠️ Objetivo (ID 5) no detectado")
                            self.last_print_time = time.time()
                
                # Visualización
                if frame is not None:
                    # Dibuja título
                    self.draw_text_title(frame, "Control de Robot - Alineación con Objetivo", (0, 0, 255))
                    
                    # Dibuja información de ArUco y posiciones
                    if points is not None and len(points) > 0 and ids is not None:
                        for (marker_corner, marker_id) in zip(points, ids.flatten()):
                            top_left, top_right, bottom_left, bottom_right = self.get_coordinates(marker_corner)
                            
                            # Punto medio y puntos de orientación
                            mid_point = self.get_mid_points(top_right, bottom_left)
                            y_point = self.get_mid_points(top_right, bottom_right)
                            x_point = self.get_mid_points(bottom_left, bottom_right)
                            
                            # Calcula el ángulo
                            angle = self.get_angle_rad(bottom_right, bottom_left)
                            
                            # Dibuja información en el frame
                            self.draw_aruco(frame, top_left, top_right, bottom_left, bottom_right, 
                                           mid_point, x_point, y_point, angle, marker_id)
                    
                    # Si detectamos el robot, dibuja su frente
                    if robot_detected:
                        self.draw_robot_front(frame, self.robot_state['x'], self.robot_state['y'], 
                                             self.robot_state['theta'])
                    
                    # Si detectamos el objetivo, dibuja información de alineación
                    if robot_detected and target_detected:
                        self.draw_alignment_info(frame, target_angle, target_direction)
                    
                    # Muestra el frame
                    cv2.imshow('Robot Control', frame)
                
                # Salir si se presiona ESC
                if cv2.waitKey(1) & 0xFF == 27:
                    break
                
                # Pequeña pausa para reducir uso de CPU
                time.sleep(0.01)
        
        except KeyboardInterrupt:
            print("\n⚠️ Proceso interrumpido por el usuario")
        finally:
            # Detener el robot
            self.send_motor_command(0, 0)
            cv2.destroyAllWindows()
            return True
    
    def stop(self):
        """
        Detiene el robot y libera recursos
        """
        if self.bt_socket:
            self.send_motor_command(0, 0)
            self.disconnect_bluetooth()
        
        self.release_camera()
        cv2.destroyAllWindows()
        print("✅ Robot detenido y recursos liberados")


def main():
    # Dirección MAC del ESP32 (cámbiala por la dirección de tu dispositivo)
    ESP32_ADDR = "88:13:BF:70:40:72"  # Ejemplo, usar la dirección real
    
    # Crear controlador de robot
    controller = RobotController(
        bt_addr=ESP32_ADDR,
        camera_index=0,
        resolution=(1280, 720),
        camera_height=225  # Altura de la cámara en cm
    )
    
    # Inicializar hardware
    if not controller.initialize_camera():
        print("❌ Error al inicializar la cámara. Saliendo...")
        return
    
    if not controller.connect_bluetooth():
        controller.release_camera()
        print("❌ Error al conectar por Bluetooth. Saliendo...")
        return
    
    # Ejecutar control del robot
    controller.run()
    
    # Detener y liberar recursos
    controller.stop()


if __name__ == "__main__":
    main()