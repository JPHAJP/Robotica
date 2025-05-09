#!/usr/bin/env python3
import cv2
import numpy as np
import math
import time
import socket
import struct
import os
import warnings

# Suprimir advertencias
warnings.filterwarnings("ignore", category=UserWarning)
os.environ["OPENCV_LOG_LEVEL"] = "SILENT"

class RobotController:
    def __init__(self, bt_addr, camera_index=0, resolution=(1280, 720), camera_height=300):
        """
        Inicializa el controlador del robot
        
        Args:
            bt_addr: Dirección MAC del dispositivo Bluetooth ESP32
            camera_index: Índice de la cámara
            resolution: Resolución de la cámara (ancho, alto)
            camera_height: Altura de la cámara en cm
        """
        # Configuración Bluetooth
        self.bt_addr = bt_addr
        self.bt_port = 1
        self.bt_socket = None
        
        # Configuración de cámara
        self.camera = None
        self.camera_index = camera_index
        self.resolution_x, self.resolution_y = resolution
        
        # Parámetros del robot
        self.wheel_radius = 6  # Radio de la rueda en cm
        self.wheel_distance = 26  # Distancia entre centros de las ruedas en cm
        self.center_to_wheel = 0  # Distancia del centro del robot al centro de las ruedas en cm
        self.robot_front_offset = 10  # Distancia del ArUco a la punta del robot (cm)
        self.robot_front_angle = math.pi/2  # +90 grados respecto al eje X del aruco
        
        # Configuración PWM
        self.max_pwm = 255
        self.min_pwm = 180
        self.max_speed = 10.0  # Velocidad máxima para escalar a PWM
        
        # Escala de píxeles a cm
        self.pixel_to_cm = 10.0 / camera_height
        
        # Parámetros de control
        self.k = 0.5  # Ganancia proporcional
        
        # IDs de marcadores
        self.robot_id = 0
        self.target_id = 5
        
        # Estados
        self.robot_state = {'x': 0, 'y': 0, 'theta': 0}
        self.target_state = {'x': 0, 'y': 0, 'theta': 0, 'x_axis': [0, 0], 'y_axis': [0, 0]}
        self.target = {'x': 0, 'y': 0}
        
        # Parámetros para alineación y proximidad
        self.distance_target = 5.0  # Distancia objetivo en cm
        self.distance_tolerance = self.distance_target * 0.1  # Tolerancia de 10%
        self.target_alignment_tolerance = math.radians(18)  # 10% de 180 grados
        
        # Control de impresión y estado
        self.last_print_time = 0
        self.print_interval = 0.5
        self.mission_completed = False
    
    def connect_bluetooth(self):
        """Establece conexión Bluetooth con el ESP32"""
        try:
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
        """Cierra la conexión Bluetooth"""
        if self.bt_socket:
            self.bt_socket.close()
            self.bt_socket = None
            print("🔌 Conexión Bluetooth cerrada")
    
    def initialize_camera(self):
        """Inicializa la cámara con manejo de errores simplificado"""
        # Intentar abrir la cámara directamente por índice
        for camera_index in range(3):
            print(f"Intentando abrir cámara {camera_index}...")
            try:
                capture = cv2.VideoCapture(camera_index)
                if capture.isOpened():
                    print(f"Cámara {camera_index} abierta exitosamente")
                    # Configurar propiedades
                    capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution_x)
                    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution_y)
                    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
                    capture.set(cv2.CAP_PROP_FPS, 30)
                    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                    
                    # Verificar que la cámara funcione
                    time.sleep(1)
                    ret, test_frame = capture.read()
                    if ret and test_frame is not None:
                        self.camera = capture
                        return True
            except Exception as e:
                print(f"Error al abrir cámara {camera_index}: {e}")
        
        print("Error: No se pudo abrir ninguna cámara")
        return False
    
    def release_camera(self):
        """Libera la cámara"""
        if self.camera:
            self.camera.release()
            self.camera = None
            print("📷 Cámara liberada")
    
    def adjust_image(self, img):
        """Ajusta brillo de la imagen"""
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)
        v = cv2.add(v, 10)  # Valor fijo de brillo
        v[v > 255] = 255
        v[v < 0] = 0
        final_hsv = cv2.merge((h, s, v))
        return cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)
    
    def detect_aruco(self):
        """Detecta marcadores ArUco en el frame actual"""
        if self.camera is None or not self.camera.isOpened():
            return None, None, None
        
        ret, frame = self.camera.read()
        if not ret or frame is None:
            return None, None, None
        
        frame = cv2.resize(frame, (self.resolution_x, self.resolution_y))
        frame = self.adjust_image(frame)
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
        aruco_params = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
        points, ids, rejected = detector.detectMarkers(gray)
        
        return frame, points, ids
    
    def get_marker_info(self, marker_corner):
        """Procesa información del marcador ArUco"""
        corners = marker_corner.reshape((4, 2))
        corners = [tuple(map(int, corner)) for corner in corners]
        top_left, top_right, bottom_right, bottom_left = corners
        
        # Punto medio y orientación
        mid_point = [(top_right[0] + bottom_left[0]) / 2, (top_right[1] + bottom_left[1]) / 2]
        angle = math.atan2(bottom_right[1] - bottom_left[1], bottom_right[0] - bottom_left[0])
        
        # Calcular ejes para visualización
        y_point = [(top_right[0] + bottom_right[0]) / 2, (top_right[1] + bottom_right[1]) / 2]
        x_point = [(bottom_left[0] + bottom_right[0]) / 2, (bottom_left[1] + bottom_right[1]) / 2]
        
        return {
            'corners': [top_left, top_right, bottom_right, bottom_left],
            'mid_point': mid_point,
            'angle': angle,
            'x_point': x_point,
            'y_point': y_point
        }
    
    def update_states(self, points, ids):
        """Actualiza las posiciones del robot y del objetivo"""
        robot_detected = False
        target_detected = False
        
        if points is not None and len(points) > 0 and ids is not None:
            ids = ids.flatten()
            
            for (marker_corner, marker_id) in zip(points, ids):
                marker_info = self.get_marker_info(marker_corner)
                mid_point = marker_info['mid_point']
                
                # Posición relativa al centro de la imagen
                pos_x = mid_point[0] - (self.resolution_x / 2)
                pos_y = mid_point[1] - (self.resolution_y / 2)
                
                if marker_id == self.robot_id:
                    self.robot_state['x'] = pos_x
                    self.robot_state['y'] = pos_y
                    self.robot_state['theta'] = marker_info['angle']
                    robot_detected = True
                    
                elif marker_id == self.target_id:
                    self.target_state['x'] = pos_x
                    self.target_state['y'] = pos_y
                    self.target_state['theta'] = marker_info['angle']
                    
                    # Calcular ejes del objetivo
                    length = 50
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
        """Calcula el punto objetivo para alineación con los ejes del objetivo"""
        # Vector desde el objetivo hacia el robot
        dx = self.robot_state['x'] - self.target_state['x']
        dy = self.robot_state['y'] - self.target_state['y']
        
        # Ángulos con respecto a los ejes del objetivo
        angle_to_x_axis = math.atan2(dy, dx) - self.target_state['theta']
        angle_to_y_axis = math.atan2(dy, dx) - (self.target_state['theta'] + math.pi/2)
        
        # Normalizar ángulos
        angle_to_x_axis = (angle_to_x_axis + math.pi) % (2 * math.pi) - math.pi
        angle_to_y_axis = (angle_to_y_axis + math.pi) % (2 * math.pi) - math.pi
        
        # Determinar eje más alineado
        if abs(angle_to_x_axis) < abs(angle_to_y_axis):
            # Alinear con eje X
            target_angle = self.target_state['theta']
            if abs(angle_to_x_axis) <= self.target_alignment_tolerance:
                target_angle = self.robot_state['theta']
            target_direction = [math.cos(target_angle), math.sin(target_angle)]
        else:
            # Alinear con eje Y
            target_angle = self.target_state['theta'] + math.pi/2
            if abs(angle_to_y_axis) <= self.target_alignment_tolerance:
                target_angle = self.robot_state['theta']
            target_direction = [math.cos(target_angle), math.sin(target_angle)]
        
        # Calcular punto objetivo
        self.target['x'] = self.target_state['x'] + (self.distance_target / self.pixel_to_cm) * target_direction[0]
        self.target['y'] = self.target_state['y'] + (self.distance_target / self.pixel_to_cm) * target_direction[1]
        
        return target_angle, target_direction
    
    def move_robot(self, x, y, theta, x_target, y_target):
        """Calcula velocidades para mover el robot hacia el punto objetivo"""
        # Ajustar posición considerando el offset del frente
        x_adjusted = x + self.robot_front_offset * math.cos(theta - math.pi/2) / self.pixel_to_cm
        y_adjusted = y + self.robot_front_offset * math.sin(theta - math.pi/2) / self.pixel_to_cm
        
        # Calcular error y control proporcional
        ex = x_adjusted - x_target
        ey = y_adjusted - y_target
        ux = -self.k * ex
        uy = -self.k * ey
        
        # Calcular distancia al objetivo
        distance = math.sqrt(ex**2 + ey**2) * self.pixel_to_cm  # cm
        
        # Verificar si llegamos al objetivo
        if (self.distance_target - self.distance_tolerance <= distance <= 
            self.distance_target + self.distance_tolerance):
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
        
        # Calcular velocidades
        try:
            U = np.linalg.solve(A, np.array([ux, uy]))
            V = U[0]  # Velocidad lineal
            W = U[1]  # Velocidad angular
            
            # Velocidades de ruedas
            wd = (V / self.wheel_radius) + ((self.wheel_distance * W) / (2 * self.wheel_radius))
            wi = (V / self.wheel_radius) - ((self.wheel_distance * W) / (2 * self.wheel_radius))
            
            # Convertir a PWM
            def angular_to_pwm(w):
                if abs(w) < 0.1:
                    return 0
                pwm = int(self.min_pwm + (self.max_pwm - self.min_pwm) * min(abs(w) / self.max_speed, 1.0))
                return pwm if w >= 0 else -pwm
            
            wd_pwm = angular_to_pwm(wd)
            wi_pwm = angular_to_pwm(wi)
            
            return wd_pwm, wi_pwm
            
        except np.linalg.LinAlgError:
            print("Error en la solución del sistema lineal")
            return 0, 0
    
    def send_motor_command(self, wd, wi):
        """Envía comandos de velocidad al robot vía Bluetooth"""
        if self.bt_socket:
            try:
                # Procesar valores PWM
                wd_abs = min(max(abs(wd), 0), self.max_pwm)
                wi_abs = min(max(abs(wi), 0), self.max_pwm)
                
                # Si PWM es muy bajo pero no cero, establecer a cero
                if 0 < wd_abs < self.min_pwm:
                    wd_abs = 0
                if 0 < wi_abs < self.min_pwm:
                    wi_abs = 0
                
                # Formato de paquete: 'H' + wr(lo,hi) + wl(lo,hi)
                packet = struct.pack('<c hh', b'H', 
                                    wd_abs if wd >= 0 else -wd_abs, 
                                    wi_abs if wi >= 0 else -wi_abs)
                self.bt_socket.send(packet)
                return True
            except Exception as e:
                print(f"❌ Error al enviar datos: {e}")
                return False
        else:
            return False
    
    def print_status(self):
        """Imprime información de estado periodicamente"""
        current_time = time.time()
        
        if current_time - self.last_print_time >= self.print_interval:
            self.last_print_time = current_time
            
            # Calcular distancia considerando el offset del frente
            robot_angle = self.robot_state['theta']
            front_offset_x = self.robot_front_offset * math.cos(robot_angle - math.pi/2) / self.pixel_to_cm
            front_offset_y = self.robot_front_offset * math.sin(robot_angle - math.pi/2) / self.pixel_to_cm
            
            dx_front = (self.robot_state['x'] + front_offset_x) - self.target_state['x']
            dy_front = (self.robot_state['y'] + front_offset_y) - self.target_state['y']
            distance_front = math.sqrt(dx_front**2 + dy_front**2) * self.pixel_to_cm
            
            # Diferencia angular
            angle_diff = robot_angle - self.target_state['theta']
            angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi  # Normalizar a [-pi, pi]
            
            # Imprimir estado
            print("\n" + "="*50)
            print(f"ESTADO DEL ROBOT [{time.strftime('%H:%M:%S')}]:")
            print(f"  Robot: x={self.robot_state['x']:.1f}, y={self.robot_state['y']:.1f}, θ={math.degrees(self.robot_state['theta']):.1f}°")
            print(f"  Objetivo: x={self.target_state['x']:.1f}, y={self.target_state['y']:.1f}, θ={math.degrees(self.target_state['theta']):.1f}°")
            print(f"  Distancia (frente-objetivo): {distance_front:.2f} cm (meta: {self.distance_target:.1f}±{self.distance_tolerance:.1f} cm)")
            print(f"  Diferencia angular: {math.degrees(angle_diff):.2f}°")
            print("="*50 + "\n")
    
    def draw_visualization(self, frame, robot_detected, target_detected, target_angle=None, target_direction=None):
        """Dibuja toda la visualización en un solo método consolidado"""
        # Título
        cv2.putText(frame, "Control de Robot - Alineación con Objetivo", 
                    (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
        
        # Si el robot fue detectado, dibujar su frente
        if robot_detected:
            # Convertir coordenadas relativas a absolutas
            abs_x = int(self.robot_state['x'] + self.resolution_x / 2)
            abs_y = int(self.robot_state['y'] + self.resolution_y / 2)
            
            # Calcular punto del frente
            front_length = abs(self.robot_front_offset) / self.pixel_to_cm
            front_x = int(abs_x + front_length * math.cos(self.robot_state['theta'] - math.pi/2))
            front_y = int(abs_y + front_length * math.sin(self.robot_state['theta'] - math.pi/2))
            
            # Dibujar línea y punto del frente
            cv2.line(frame, (abs_x, abs_y), (front_x, front_y), (255, 0, 255), 3)
            cv2.circle(frame, (front_x, front_y), 8, (255, 0, 255), -1)
            cv2.putText(frame, "FRENTE", (front_x + 10, front_y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2, cv2.LINE_AA)
        
        # Si ambos fueron detectados, dibujar información de alineación
        if robot_detected and target_detected and target_direction is not None:
            # Coordenadas absolutas del objetivo
            target_x = int(self.target_state['x'] + self.resolution_x / 2)
            target_y = int(self.target_state['y'] + self.resolution_y / 2)
            
            # Dibujar el eje elegido para la alineación
            line_length = 100
            end_x = int(target_x + line_length * target_direction[0])
            end_y = int(target_y + line_length * target_direction[1])
            cv2.line(frame, (target_x, target_y), (end_x, end_y), (255, 255, 0), 3)
            
            # Dibujar punto objetivo
            target_point_x = int(self.target['x'] + self.resolution_x / 2)
            target_point_y = int(self.target['y'] + self.resolution_y / 2)
            cv2.circle(frame, (target_point_x, target_point_y), 8, (0, 255, 255), -1)
            cv2.putText(frame, "Punto objetivo", (target_point_x + 10, target_point_y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)
            
            # Mostrar distancia objetivo
            cv2.putText(frame, f"Distancia objetivo: {self.distance_target} cm", 
                        (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)
    
    def run(self):
        """Ejecuta el bucle principal del controlador"""
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
                # Detectar marcadores ArUco
                frame, points, ids = self.detect_aruco()
                
                if frame is None:
                    print("❌ No se pudo obtener frame de la cámara")
                    time.sleep(0.5)
                    continue
                
                # Actualizar posiciones
                robot_detected, target_detected = self.update_states(points, ids)
                
                # Variables para visualización
                target_angle = None
                target_direction = None
                
                if robot_detected and target_detected:
                    # Calcular punto objetivo
                    target_angle, target_direction = self.calculate_alignment_target()
                    
                    # Calcular velocidades
                    wd, wi = self.move_robot(
                        self.robot_state['x'], 
                        self.robot_state['y'], 
                        self.robot_state['theta'],
                        self.target['x'], 
                        self.target['y']
                    )
                    
                    # Enviar comandos
                    self.send_motor_command(wd, wi)
                    
                    # Mostrar estado
                    self.print_status()
                else:
                    # Detener robot si no detectamos algo
                    self.send_motor_command(0, 0)
                    
                    if not robot_detected and time.time() - self.last_print_time >= self.print_interval:
                        print("⚠️ Robot (ID 0) no detectado")
                        self.last_print_time = time.time()
                    
                    if not target_detected and time.time() - self.last_print_time >= self.print_interval:
                        print("⚠️ Objetivo (ID 5) no detectado")
                        self.last_print_time = time.time()
                
                # Visualización
                if frame is not None:
                    # Mostrar información visual de marcadores detectados
                    if points is not None and len(points) > 0 and ids is not None:
                        for (marker_corner, marker_id) in zip(points, ids.flatten()):
                            marker_info = self.get_marker_info(marker_corner)
                            corners = marker_info['corners']
                            mid_point = marker_info['mid_point']
                            
                            # Dibujar cuadrado del marcador
                            for i in range(4):
                                cv2.line(frame, corners[i], corners[(i+1)%4], (0, 255, 0), 2)
                            
                            # Indicar posición relativa y ID
                            pos_x = mid_point[0] - (self.resolution_x / 2)
                            pos_y = mid_point[1] - (self.resolution_y / 2)
                            cv2.putText(frame, f"[{pos_x:.1f}, {pos_y:.1f}]", 
                                       (int(mid_point[0]), int(mid_point[1]) - 150), 
                                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 204, 204), 2, cv2.LINE_AA)
                            cv2.putText(frame, f"ID: {marker_id}", 
                                       (int(mid_point[0]) - 50, int(mid_point[1]) - 100), 
                                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 204, 204), 2, cv2.LINE_AA)
                    
                    # Dibujar visualización general
                    self.draw_visualization(frame, robot_detected, target_detected, target_angle, target_direction)
                    
                    # Mostrar frame
                    cv2.imshow('Robot Control', frame)
                
                # Salir con ESC
                if cv2.waitKey(1) & 0xFF == 27:
                    break
                
                # Pausa para reducir CPU
                time.sleep(0.01)
        
        except KeyboardInterrupt:
            print("\n⚠️ Proceso interrumpido por el usuario")
        finally:
            # Detener robot
            self.send_motor_command(0, 0)
            cv2.destroyAllWindows()
            return True
    
    def stop(self):
        """Detiene el robot y libera recursos"""
        if self.bt_socket:
            self.send_motor_command(0, 0)
            self.disconnect_bluetooth()
        
        self.release_camera()
        cv2.destroyAllWindows()
        print("✅ Robot detenido y recursos liberados")


def main():
    # Dirección MAC del ESP32
    ESP32_ADDR = "88:13:BF:70:40:72"  # Cambiar por la dirección real
    
    # Crear controlador
    controller = RobotController(
        bt_addr=ESP32_ADDR,
        camera_index=0,
        resolution=(1280, 720),
        camera_height=200
    )
    
    # Inicializar hardware
    if not controller.initialize_camera():
        print("❌ Error al inicializar la cámara. Saliendo...")
        return
    
    if not controller.connect_bluetooth():
        controller.release_camera()
        print("❌ Error al conectar por Bluetooth. Saliendo...")
        return
    
    # Ejecutar control
    controller.run()
    
    # Detener y liberar recursos
    controller.stop()


if __name__ == "__main__":
    main()