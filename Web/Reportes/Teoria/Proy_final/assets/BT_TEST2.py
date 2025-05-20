import socket
import struct
import time
import sys

class BluetoothController:
    """
    Class for controlling robot movement via Bluetooth connection to ESP32
    """
    def __init__(self, esp32_addr="88:13:BF:70:40:72", port=1, debug=True):
        """
        Initialize Bluetooth Controller
        
        Args:
            esp32_addr: MAC address of ESP32 device
            port: RFCOMM port for Bluetooth connection
            debug: Whether to print debug information
        """
        self.esp32_addr = esp32_addr
        self.port = port
        self.debug = debug
        self.sock = None
        self.connected = False
    
    def connect(self):
        """
        Establish connection to ESP32 via Bluetooth
        
        Returns:
            Boolean indicating if connection was successful
        """
        # Create RFCOMM Bluetooth socket
        self.sock = socket.socket(
            socket.AF_BLUETOOTH,
            socket.SOCK_STREAM,
            socket.BTPROTO_RFCOMM
        )
        
        if self.debug:
            print(f"Conectando a {self.esp32_addr}:{self.port} …")
            
        try:
            self.sock.connect((self.esp32_addr, self.port))
            self.connected = True
            if self.debug:
                print("✅ Conexión Bluetooth establecida.")
            return True
        except OSError as e:
            if self.debug:
                print("❌ Error al conectar:", e)
            self.connected = False
            return False
    
    def disconnect(self):
        """
        Close Bluetooth connection
        """
        if self.sock and self.connected:
            if self.debug:
                print("🔌 Cerrando conexión Bluetooth…")
            self.sock.close()
            self.connected = False
    
    def send_motor_speeds(self, right_speed, left_speed):
        """
        Send motor speed values to ESP32
        
        Args:
            right_speed: Right motor speed (-255 to 255)
            left_speed: Left motor speed (-255 to 255)
            
        Returns:
            Boolean indicating if values were sent successfully
        """
        if not self.connected:
            if self.debug:
                print("❌ No hay conexión Bluetooth establecida.")
            return False
            
        # Ensure values are within valid range
        right_speed = max(-255, min(255, right_speed))
        left_speed = max(-255, min(255, left_speed))
        
        # Intentar con formato de texto simple como los comandos J
        try:
            # Crear un comando en formato similar a los comandos J
            command = f"H {right_speed} {left_speed}\n"
            
            if self.debug:
                print(f"📤 Enviando comando: {command.strip()}")
            
            self.sock.send(command.encode('utf-8'))
            
            if self.debug:
                print(f"📤 Enviado: Motor D: {right_speed}, Motor I: {left_speed}")
            return True
        except OSError as e:
            if self.debug:
                print("❌ Error enviando datos:", e)
            self.connected = False
            return False
    
    def stop_motors(self):
        """
        Stop both motors
        
        Returns:
            Boolean indicating if stop command was sent successfully
        """
        return self.send_motor_speeds(0, 0)
    
    def move_forward(self, speed=100):
        """
        Move robot forward at specified speed
        
        Args:
            speed: Motor speed (0 to 255)
            
        Returns:
            Boolean indicating if command was sent successfully
        """
        return self.send_motor_speeds(-speed, -speed)
    
    def move_backward(self, speed=100):
        """
        Move robot backward at specified speed
        
        Args:
            speed: Motor speed (0 to 255)
            
        Returns:
            Boolean indicating if command was sent successfully
        """
        return self.send_motor_speeds(speed, speed)
    
    def turn_left(self, speed=100):
        """
        Turn robot left (counter-clockwise) at specified speed
        
        Args:
            speed: Motor speed (0 to 255)
            
        Returns:
            Boolean indicating if command was sent successfully
        """
        return self.send_motor_speeds(speed, -speed)
    
    def turn_right(self, speed=100):
        """
        Turn robot right (clockwise) at specified speed
        
        Args:
            speed: Motor speed (0 to 255)
            
        Returns:
            Boolean indicating if command was sent successfully
        """
        return self.send_motor_speeds(-speed, speed)
    
    def set_differential_drive(self, forward_speed, turn_rate):
        """
        Control robot using differential drive model
        
        Args:
            forward_speed: Forward speed (-255 to 255)
            turn_rate: Turn rate (-255 to 255), positive is right turn
            
        Returns:
            Boolean indicating if command was sent successfully
        """
        # Calculate individual motor speeds based on forward and turn components
        right_speed = forward_speed - turn_rate
        left_speed = forward_speed + turn_rate
        
        # Ensure values don't exceed limits
        right_speed = max(-255, min(255, right_speed))
        left_speed = max(-255, min(255, left_speed))
        
        return self.send_motor_speeds(right_speed, left_speed)
    
    def test_connection(self):
        """
        Test Bluetooth connection by sending a stop command
        
        Returns:
            Boolean indicating if test was successful
        """
        if not self.connected:
            success = self.connect()
            if not success:
                return False
                
        # Try to send a stop command
        return self.stop_motors()
    
    # Añadir métodos para controlar los motores articulados
    def send_articulated_command(self, command):
        """
        Send command to articulated motors
        
        Args:
            command: Command string to send (e.g., "INIT" or "M1 100")
            
        Returns:
            Boolean indicating if command was sent successfully
        """
        if not self.connected:
            if self.debug:
                print("❌ No hay conexión Bluetooth establecida.")
            return False
        
        # Format command with J prefix
        full_command = f"J {command}\n"
        
        try:
            self.sock.send(full_command.encode())
            if self.debug:
                print(f"📤 Enviado comando articulado: {command}")
            return True
        except OSError as e:
            if self.debug:
                print("❌ Error enviando datos:", e)
            self.connected = False
            return False
    
    def init_homing(self):
        """
        Initialize homing sequence for articulated motors
        
        Returns:
            Boolean indicating if command was sent successfully
        """
        return self.send_articulated_command("INIT")
    
    def move_articulated_motor(self, motor_num, steps):
        """
        Move specific articulated motor by given steps
        
        Args:
            motor_num: Motor number (1-3)
            steps: Number of steps to move (negative for reverse)
            
        Returns:
            Boolean indicating if command was sent successfully
        """
        if motor_num < 1 or motor_num > 3:
            if self.debug:
                print(f"❌ Número de motor inválido: {motor_num}. Debe ser 1-3.")
            return False
        
        command = f"M{motor_num} {steps}"
        return self.send_articulated_command(command)
    
    def move_multiple_motors(self, movements):
        """
        Move multiple articulated motors in sequence
        
        Args:
            movements: List of (motor_num, steps) tuples
            
        Returns:
            Boolean indicating if command was sent successfully
        """
        if not movements:
            return False
        
        command_parts = []
        for motor_num, steps in movements:
            if motor_num < 1 or motor_num > 3:
                if self.debug:
                    print(f"❌ Número de motor inválido: {motor_num}. Debe ser 1-3.")
                continue
            command_parts.append(f"M{motor_num} {steps}")
        
        if not command_parts:
            return False
        
        command = ", ".join(command_parts)
        return self.send_articulated_command(command)
    
    def send_trajectory_command(self, movements):
        """
        Move multiple articulated motors in sequence
        
        Args:
            movements: List of movements, where each movement is a list of 
                    tuples (motor_id, steps, velocity, acceleration)
                    
        Returns:
            Boolean indicating if command was sent successfully
        """
        if not self.connected:
            if self.debug:
                print("❌ No hay conexión Bluetooth establecida.")
            return False
            
        # Construir el comando de trayectoria completo
        command_parts = []
        
        for movement in movements:
            motor_configs = []
            for motor_id, steps, velocity, accel in movement:
                if motor_id < 1 or motor_id > 3:
                    if self.debug:
                        print(f"❌ Número de motor inválido: {motor_id}. Debe ser 1-3.")
                    continue
                motor_configs.append(f"{motor_id},{steps},{velocity},{accel}")
            
            if motor_configs:
                command_parts.append(";".join(motor_configs))
        
        if not command_parts:
            if self.debug:
                print("❌ No hay movimientos válidos para enviar.")
            return False
        
        full_command = "|".join(command_parts)
        return self.send_articulated_command(full_command)


def print_help():
    """Print help information"""
    print("\n=== COMANDOS DISPONIBLES ===")
    print("Motores de Movimiento:")
    print("  w <velocidad>      - Avanzar (velocidad: 0-255, default=100)")
    print("  s <velocidad>      - Retroceder (velocidad: 0-255, default=100)")
    print("  a <velocidad>      - Girar izquierda (velocidad: 0-255, default=100)")
    print("  d <velocidad>      - Girar derecha (velocidad: 0-255, default=100)")
    print("  v <avance> <giro>  - Control diferencial (avance: -255-255, giro: -255-255)")
    print("  x                  - Detener motores")
    print("\nMotores Articulados:")
    print("  init               - Iniciar secuencia de homing")
    print("  m <motor> <pasos>  - Mover motor articulado (motor: 1-3, pasos: negativo=atrás)")
    print("  mm <motor1> <pasos1> <motor2> <pasos2> ... - Mover múltiples motores")
    print("  traj               - Enviar trayectoria completa")
    print("  traj leer          - Leer trayectoria desde archivo movements.txt")
    print("  traj completa      - Enviar trayectoria de ejemplo completa")
    print("\nOtras Comandos:")
    print("  h, ?               - Mostrar esta ayuda")
    print("  q, quit, exit      - Salir del programa")
    print("=============================\n")


def main():
    # Dirección MAC del ESP32 - actualiza según tu dispositivo
    esp32_mac = "88:13:BF:70:40:72"
    
    print("=== Control de Robot ESP32 por Bluetooth ===")
    print(f"Conectando al ESP32 con dirección MAC: {esp32_mac}")
    
    # Crear controlador Bluetooth
    controller = BluetoothController(esp32_addr=esp32_mac, debug=True)
    
    # Intentar conectar
    if not controller.connect():
        print("❌ No se pudo establecer la conexión. Saliendo.")
        return
    
    print_help()
    
    try:
        while True:
            cmd = input("\n> ").strip().lower()
            parts = cmd.split()
            
            if not parts:
                continue
            
            # Comandos de control principal
            if parts[0] in ['q', 'quit', 'exit']:
                print("Saliendo...")
                break
            elif parts[0] in ['h', '?', 'help']:
                print_help()
            
            # Comandos para motores de movimiento
            elif parts[0] == 'w':  # Avanzar
                speed = int(parts[1]) if len(parts) > 1 else 100
                controller.move_forward(speed)
            elif parts[0] == 's':  # Retroceder
                speed = int(parts[1]) if len(parts) > 1 else 100
                controller.move_backward(speed)
            elif parts[0] == 'a':  # Girar izquierda
                speed = int(parts[1]) if len(parts) > 1 else 100
                controller.turn_left(speed)
            elif parts[0] == 'd':  # Girar derecha
                speed = int(parts[1]) if len(parts) > 1 else 100
                controller.turn_right(speed)
            elif parts[0] == 'x':  # Detener
                controller.stop_motors()
            elif parts[0] == 'v':  # Control diferencial
                if len(parts) >= 3:
                    forward = int(parts[1])
                    turn = int(parts[2])
                    controller.set_differential_drive(forward, turn)
                else:
                    print("❌ Formato correcto: v <avance> <giro>")
            
            # Comandos para motores articulados
            elif parts[0] == 'init':  # Iniciar homing
                controller.init_homing()
            elif parts[0] == 'm':  # Mover un motor articulado
                if len(parts) >= 3:
                    motor = int(parts[1])
                    steps = int(parts[2])
                    controller.move_articulated_motor(motor, steps)
                else:
                    print("❌ Formato correcto: m <motor> <pasos>")
            elif parts[0] == 'mm':  # Mover múltiples motores
                if len(parts) >= 3 and len(parts) % 2 == 1:
                    movements = []
                    for i in range(1, len(parts), 2):
                        motor = int(parts[i])
                        steps = int(parts[i+1])
                        movements.append((motor, steps))
                    controller.move_multiple_motors(movements)
                else:
                    print("❌ Formato correcto: mm <motor1> <pasos1> <motor2> <pasos2> ...")
            
            # Nuevos comandos para trayectorias
            elif parts[0] == 'traj':
                if len(parts) > 1:
                    if parts[1] == 'leer':
                        # Leer trayectoria desde archivo
                        print("Leyendo trayectoria desde archivo...")
                        movements = []
                        try:
                            with open("Web/Reportes/Teoria/Proy_final/assets/movements.txt", "r") as file:
                                content = file.read()
                                # El archivo contiene una representación de lista de Python
                                # Evaluar de manera segura el contenido como una estructura de Python
                                # Asumiendo que el archivo contiene una lista de listas de tuplas
                                movements = eval(content)
                                
                            if movements:
                                print(f"Trayectoria leída correctamente. {len(movements)} movimientos encontrados.")
                                #print(movements)
                                controller.send_trajectory_command(movements)
                            else:
                                print("❌ No se encontraron movimientos válidos en el archivo.")
                        except FileNotFoundError:
                            print("❌ Archivo de movimientos no encontrado.")

                        print("Enviando trayectoria simple...")
                        controller.send_trajectory_command(movements)
                    elif parts[1] == 'completa':
                        # Ejemplo de trayectoria completa
                        movements = [
                            [(1, 80, 2000, 2000), (2, 60, 1800, 1800), (3, -70, 2200, 2200)],
                            [(1, -40, 1000, 1000), (2, 30, 900, 900), (3, 35, 1100, 1100)],
                            [(1, -40, 1500, 1500), (2, -90, 1800, 1800), (3, 35, 2000, 2000)]
                        ]
                        print("Enviando trayectoria completa...")
                        controller.send_trajectory_command(movements)
                    else:
                        print("❌ Opción de trayectoria no reconocida")
                else:
                    # Solicitar trayectoria personalizada
                    print("Introduce la trayectoria en formato: motor_id,pasos,velocidad,aceleracion;...")
                    traj_input = input("Trayectoria> ")
                    # Parsear la entrada del usuario
                    try:
                        # Dividir por | para separar movimientos
                        movement_strings = traj_input.split('|')
                        movements = []
                        
                        for mov_str in movement_strings:
                            # Dividir por ; para separar configuraciones de motores
                            motor_configs = mov_str.split(';')
                            movement = []
                            
                            for config in motor_configs:
                                # Dividir por , para obtener los parámetros
                                params = config.split(',')
                                if len(params) == 4:
                                    motor_id = int(params[0])
                                    steps = int(params[1])
                                    velocity = float(params[2])
                                    accel = float(params[3])
                                    movement.append((motor_id, steps, velocity, accel))
                            
                            if movement:
                                movements.append(movement)
                        
                        if movements:
                            controller.send_trajectory_command(movements)
                        else:
                            print("❌ No se pudo parsear la trayectoria correctamente")
                    except Exception as e:
                        print(f"❌ Error al parsear la trayectoria: {e}")
            
            else:
                print(f"❌ Comando desconocido: {parts[0]}")
                print("Usa 'h' o '?' para ver la lista de comandos")
                
    except KeyboardInterrupt:
        print("\nPrograma interrumpido por el usuario.")
    finally:
        # Asegurarse de detener los motores y cerrar la conexión
        controller.stop_motors()
        time.sleep(0.1)  # Pequeña pausa para asegurar que se envíe el comando
        controller.disconnect()
        print("Conexión Bluetooth cerrada.")


if __name__ == "__main__":
    main()