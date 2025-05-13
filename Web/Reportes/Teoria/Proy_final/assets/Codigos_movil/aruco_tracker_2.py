import cv2
import numpy as np
import math
import time
import os
import sys
import struct
import socket

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
        
        # Pack the packet with header 'H' + right_speed + left_speed (little-endian int16)
        packet = struct.pack('<c hh', b'H', right_speed, left_speed)
        
        try:
            self.sock.send(packet)
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
        return self.send_motor_speeds(speed, speed)
    
    def move_backward(self, speed=100):
        """
        Move robot backward at specified speed
        
        Args:
            speed: Motor speed (0 to 255)
            
        Returns:
            Boolean indicating if command was sent successfully
        """
        return self.send_motor_speeds(-speed, -speed)
    
    def turn_left(self, speed=100):
        """
        Turn robot left (counter-clockwise) at specified speed
        
        Args:
            speed: Motor speed (0 to 255)
            
        Returns:
            Boolean indicating if command was sent successfully
        """
        return self.send_motor_speeds(-speed, speed)
    
    def turn_right(self, speed=100):
        """
        Turn robot right (clockwise) at specified speed
        
        Args:
            speed: Motor speed (0 to 255)
            
        Returns:
            Boolean indicating if command was sent successfully
        """
        return self.send_motor_speeds(speed, -speed)
    
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


class ImageProcessor:
    """
    Utility class for image processing operations
    """
    @staticmethod
    def change_brightness(img, value):
        """
        Modifies image brightness
        
        Args:
            img: Input image
            value: Brightness adjustment value
            
        Returns:
            Modified image with adjusted brightness
        """
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)
        v = cv2.add(v, value)
        v[v > 255] = 255
        v[v < 0] = 0
        final_hsv = cv2.merge((h, s, v))
        img = cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)
        return img


class CoordinateCalculator:
    """
    Utility class for coordinate and angle calculations
    """
    @staticmethod
    def mid_points(matrix, pt1, pt2):
        """
        Calculate midpoint between two points
        
        Args:
            matrix: Output matrix
            pt1: First point
            pt2: Second point
            
        Returns:
            Matrix with midpoint coordinates
        """
        matrix[0][0] = (pt1[0] + pt2[0]) / 2
        matrix[0][1] = (pt1[1] + pt2[1]) / 2
        return matrix
    
    @staticmethod
    def get_angle(bottom_right, bottom_left):
        """
        Get rotation angle in degrees
        
        Args:
            bottom_right: Bottom right corner coordinates
            bottom_left: Bottom left corner coordinates
            
        Returns:
            Angle in degrees
        """
        x = (bottom_right[0] - bottom_left[0])
        y = (bottom_right[1] - bottom_left[1])
        angle = math.atan2(y, x)
        angle = math.degrees(angle)
        angle *= -1

        if angle < 0:
            angle += 360
        angle = round(angle, 2)
        angle = abs(angle)
        return angle
    
    @staticmethod
    def get_anglerad(bottom_right, bottom_left):
        """
        Get rotation angle in radians
        
        Args:
            bottom_right: Bottom right corner coordinates
            bottom_left: Bottom left corner coordinates
            
        Returns:
            Angle in radians
        """
        x = (bottom_right[0] - bottom_left[0])
        y = (bottom_right[1] - bottom_left[1])
        angle = math.atan2(y, x)
        angle = round(angle, 2)
        return angle
    
    @staticmethod
    def get_coordinates(marker_corner):
        """
        Extract coordinates from marker corners
        
        Args:
            marker_corner: Corner data from ArUco detection
            
        Returns:
            Tuple of (topLeft, topRight, bottomLeft, bottomRight) coordinates
        """
        corners = marker_corner.reshape((4, 2))
        (top_left, top_right, bottom_right, bottom_left) = corners

        # Convert each of the (x, y)-coordinate pairs to integers
        top_right = (int(top_right[0]), int(top_right[1]))
        bottom_right = (int(bottom_right[0]), int(bottom_right[1]))
        bottom_left = (int(bottom_left[0]), int(bottom_left[1]))
        top_left = (int(top_left[0]), int(top_left[1]))

        return top_left, top_right, bottom_left, bottom_right


class ArucoDrawer:
    """
    Class for drawing ArUco marker information on frames
    """
    @staticmethod
    def draw_aruco(frame, top_left, top_right, bottom_left, bottom_right, mid_p, x_point, y_point, angle, marker_id, resolution_x, resolution_y):
        """
        Draw ArUco marker and information on frame
        
        Args:
            frame: Input frame
            top_left, top_right, bottom_left, bottom_right: Corner coordinates
            mid_p: Midpoint coordinates
            x_point, y_point: Axis points
            angle: Rotation angle
            marker_id: ArUco marker ID
            resolution_x, resolution_y: Frame resolution
        """
        # Draw the square
        cv2.line(frame, top_left, top_right, (0, 255, 0), 2)
        cv2.line(frame, top_right, bottom_right, (0, 255, 0), 2)
        cv2.line(frame, bottom_right, bottom_left, (0, 255, 0), 2)
        cv2.line(frame, bottom_left, top_left, (0, 255, 0), 2)

        # Draw the lines
        line_thickness = 3
        cv2.line(frame, x_point[0], mid_p[0], (0, 0, 255), thickness=line_thickness)
        cv2.line(frame, y_point[0], mid_p[0], (255, 0, 0), thickness=line_thickness)

        # Print the information text
        cv2.putText(
            frame, 
            f"[{mid_p[0][0] - (resolution_x / 2)}, {mid_p[0][1] - (resolution_y / 2)}]", 
            (mid_p[0][0], mid_p[0][1] - 150), 
            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 204, 204), 2, cv2.LINE_AA
        )
        cv2.putText(
            frame, 
            f'ID: {marker_id}', 
            (mid_p[0][0] - 50, mid_p[0][1] - 100), 
            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 204, 204), 2, cv2.LINE_AA
        )
    
    @staticmethod
    def draw_texto_titulo(frame, text, color):
        """
        Draw title text on frame
        
        Args:
            frame: Input frame
            text: Text to display
            color: Text color
        """
        cv2.putText(frame, text, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2, cv2.LINE_AA)
    
    @staticmethod
    def draw_punto(frame, text, color, pos_x, pos_y, resolution_x, resolution_y):
        """
        Draw point and information on frame
        
        Args:
            frame: Input frame
            text: Text to display
            color: Text and point color
            pos_x, pos_y: Point coordinates
            resolution_x, resolution_y: Frame resolution
        """
        cv2.putText(
            frame, 
            text, 
            (pos_x + 20, pos_y + 20), 
            cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2, cv2.LINE_AA
        )
        cv2.putText(
            frame, 
            f"{pos_x - resolution_x / 2},{pos_y - (resolution_y / 2)}", 
            (pos_x + 20, pos_y + 50), 
            cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2, cv2.LINE_AA
        )
        cv2.circle(frame, (pos_x, pos_y), 10, color, 5)


class ArucoDetector:
    """
    Main class for ArUco marker detection
    """
    def __init__(self, visual=True, width=640, height=480):
        """
        Initialize ArUco detector
        
        Args:
            visual: Whether to display visual interface
            width: Frame width resolution
            height: Frame height resolution
        """
        self.visual = visual
        self.resolution_x = width
        self.resolution_y = height
        self.camera = None
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        self.window_name = 'Camara detector qr'
        
        # Initialize matrices
        self.mid_p = np.arange(2).reshape(1, 2)
        self.y_point = np.arange(2).reshape(1, 2)
        self.x_point = np.arange(2).reshape(1, 2)
    
    def initialize_camera(self):
        """
        Initialize camera with robust error handling
        
        Returns:
            True if camera initialized successfully, False otherwise
        """
        capture = None
        
        # Try with direct device access first (often more reliable on Linux)
        for i in range(10):  # Check up to 10 possible video devices
            device_path = f"/dev/video{i}"
            try:
                if os.path.exists(device_path):
                    print(f"Found device: {device_path}, trying to open...")
                    
                    # Try MJPEG format first (often more compatible)
                    capture = cv2.VideoCapture(device_path)
                    if capture.isOpened():
                        # Force format to MJPEG for Linux
                        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                        print(f"Successfully opened {device_path}")
                        break
            except Exception as e:
                print(f"Error with {device_path}: {e}")
        
        # If direct device access didn't work, try standard indices
        if capture is None or not capture.isOpened():
            for camera_index in range(3):
                print(f"Trying to open camera {camera_index}...")
                
                # Try with VideoCapture(index)
                try:
                    capture = cv2.VideoCapture(camera_index)
                    if capture.isOpened():
                        print(f"Successfully opened camera {camera_index}")
                        # Set buffer size to minimum to reduce latency
                        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                        break
                except Exception as e:
                    print(f"Error with standard method: {e}")
                    
                # Try with V4L2
                try:
                    capture = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
                    if capture.isOpened():
                        print(f"Successfully opened camera {camera_index} with V4L2")
                        # Set buffer size to minimum to reduce latency
                        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                        break
                except Exception as e:
                    print(f"Error with V4L2: {e}")
        
        # If we couldn't open any camera
        if capture is None or not capture.isOpened():
            print("Could not open any camera. Please check your camera connection.")
            return False
            
        # Set camera properties - use lower resolution first to ensure compatibility
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution_x)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution_y)
        
        # For V4L2 compatibility
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        capture.set(cv2.CAP_PROP_FPS, 30)
        
        # Give the camera some time to initialize
        time.sleep(2)
        
        # Verify camera works by trying to read a frame
        for i in range(5):
            ret, test_frame = capture.read()
            if ret and test_frame is not None:
                print("Camera test successful - able to read frames")
                
                # Now try to set higher resolution if needed
                if self.resolution_x > 640 and self.resolution_y > 480:
                    capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution_x)
                    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution_y)
                
                self.camera = capture
                return True
            else:
                print(f"Camera test attempt {i+1}/5 failed, retrying...")
                time.sleep(1)
        
        print("Failed to read frames from camera after initialization")
        return False
    
    def get_aruco_info(self, marker_corner, marker_id):
        """
        Get ArUco marker information
        
        Args:
            marker_corner: Corner data from ArUco detection
            marker_id: ArUco marker ID
            
        Returns:
            Dictionary with coordinates, angle, and ID information
        """
        top_left, top_right, bottom_left, bottom_right = CoordinateCalculator.get_coordinates(marker_corner)

        # Calculate the angle of inclination
        angle = CoordinateCalculator.get_anglerad(bottom_right, bottom_left)

        info = {
            "coordenadas": [top_left, top_right, bottom_left, bottom_right], 
            "angulo": angle, 
            "ID": marker_id
        }

        return info
    
    def buscar_aruco(self):
        """
        Search for ArUco markers in camera frame
        
        Returns:
            Tuple of (frame, points, ids) or (None, None, None) if error
        """
        if self.camera is None or not self.camera.isOpened():
            return None, None, None
            
        ret, frame = self.camera.read()
        if not ret or frame is None:
            return None, None, None

        frame = cv2.resize(frame, (self.resolution_x, self.resolution_y))
        frame = ImageProcessor.change_brightness(frame, 10)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        points, ids, rejected = self.detector.detectMarkers(gray)

        return frame, points, ids
    
    def dibujar_aruco(self, frame, points, ids):
        """
        Draw ArUco markers on frame
        
        Args:
            frame: Input frame
            points: ArUco marker points
            ids: ArUco marker IDs
        """
        if frame is None:
            return

        if points is not None and len(points) > 0 and ids is not None:
            # flatten the ArUco IDs list
            ids = ids.flatten()
            # loop over the detected ArUCo corners
            for (marker_corner, marker_id) in zip(points, ids):
                top_left, top_right, bottom_left, bottom_right = CoordinateCalculator.get_coordinates(marker_corner)

                # Get coordinates for midpoint and lines
                CoordinateCalculator.mid_points(self.mid_p, top_right, bottom_left)
                CoordinateCalculator.mid_points(self.y_point, top_right, bottom_right)
                CoordinateCalculator.mid_points(self.x_point, bottom_left, bottom_right)

                # Calculate the angle of inclination
                angle = CoordinateCalculator.get_anglerad(bottom_right, bottom_left)

                ArucoDrawer.draw_aruco(
                    frame, top_left, top_right, bottom_left, bottom_right, 
                    self.mid_p, self.x_point, self.y_point, angle, marker_id, 
                    self.resolution_x, self.resolution_y
                )
    
    def buscar_robots(self, points, ids, robot):
        """
        Update robot positions based on ArUco markers
        
        Args:
            points: ArUco marker points
            ids: ArUco marker IDs
            robot: Dictionary to update with robot positions
            
        Returns:
            Updated robot dictionary
        """
        if points is not None and len(points) > 0 and ids is not None:
            # flatten the ArUco IDs list
            ids = ids.flatten()
            # loop over the detected ArUCo corners
            for (marker_corner, marker_id) in zip(points, ids):
                top_left, top_right, bottom_left, bottom_right = CoordinateCalculator.get_coordinates(marker_corner)

                # Get coordinates for midpoint
                CoordinateCalculator.mid_points(self.mid_p, top_right, bottom_left)
                robot[marker_id][0] = self.mid_p[0][0]
                robot[marker_id][1] = self.mid_p[0][1]
                robot[marker_id][2] = CoordinateCalculator.get_anglerad(bottom_right, bottom_left)

        return robot
    
    def preview(self):
        """
        Show camera preview without ArUco detection
        """
        if self.camera is None or not self.camera.isOpened():
            return
            
        ret, frame = self.camera.read()
        if not ret or frame is None:
            return

        frame = cv2.resize(frame, (self.resolution_x, self.resolution_y))
        frame = ImageProcessor.change_brightness(frame, 10)
        cv2.imshow(self.window_name, frame)
    
    def run_detection(self):
        """
        Run continuous ArUco detection loop
        """
        if not self.initialize_camera():
            print("Failed to initialize camera. Exiting...")
            return
        
        # Skip fewer frames initially
        frames_to_skip = 3
        for i in range(frames_to_skip):
            ret, _ = self.camera.read()  # Read and discard frames to let camera adjust
            time.sleep(0.1)
        
        print("Camera initialized. Starting detection...")
        
        while True:
            print("Waiting for ArUco markers...")
            frame, points, ids = self.buscar_aruco()
            
            if frame is None:
                print("Error: Could not read frame from camera. Checking connection...")
                time.sleep(1)
                continue
            
            try:
                if points is not None and len(points) > 0 and ids is not None:
                    # flatten the ArUco IDs list
                    ids = ids.flatten()
                    # loop over the detected ArUCo corners
                    for (marker_corner, marker_id) in zip(points, ids):
                        top_left, top_right, bottom_left, bottom_right = CoordinateCalculator.get_coordinates(marker_corner)
                        
                        # Get coordinates for midpoint and lines
                        CoordinateCalculator.mid_points(self.mid_p, top_right, bottom_left)
                        CoordinateCalculator.mid_points(self.y_point, top_right, bottom_right)
                        CoordinateCalculator.mid_points(self.x_point, bottom_left, bottom_right)
                        
                        # Calculate the angle of inclination
                        angle = CoordinateCalculator.get_angle(bottom_right, bottom_left)
                        
                        if self.visual:
                            ArucoDrawer.draw_aruco(
                                frame, top_left, top_right, bottom_left, bottom_right, 
                                self.mid_p, self.x_point, self.y_point, angle, marker_id, 
                                self.resolution_x, self.resolution_y
                            )
                        
                        print(f"ID:{marker_id}, X:{self.mid_p[0][0] - (self.resolution_x / 2)}, "
                              f"Y:{self.mid_p[0][1] - (self.resolution_y / 2)}, Th:{angle}")
                
                if self.visual:
                    cv2.putText(frame, "Press ESC to exit", (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                    cv2.imshow(self.window_name, frame)
                
            except Exception as e:
                print(f"Error processing frame: {e}")
                
            if cv2.waitKey(1) & 0xFF == 27:  # Press ESC to exit
                break
        
        self.close()
    
    def close(self):
        """
        Release camera and close windows
        """
        if self.camera is not None:
            self.camera.release()
        cv2.destroyAllWindows()


class RobotTargetTracker:
    """
    Class for tracking robot and target using ArUco markers with real-world measurements
    """
    def __init__(self, robot_id=0, target_id=1, 
                 robot_offset_px=190, target_offset_px=83, 
                 robot_offset_mm=268, target_offset_mm=138, 
                 visual=True, width=1280, height=720):
        """
        Initialize Robot and Target tracker
        
        Args:
            robot_id: ArUco ID for robot
            target_id: ArUco ID for target
            robot_offset_px: Distance in pixels to offset the point of interest for robot
            target_offset_px: Distance in pixels to offset the point of interest for target
            robot_offset_mm: Real distance in mm corresponding to robot_offset_px
            target_offset_mm: Real distance in mm corresponding to target_offset_px
            visual: Whether to display visual interface
            width: Frame width resolution
            height: Frame height resolution
        """
        self.robot_id = robot_id
        self.target_id = target_id
        self.robot_offset_px = robot_offset_px
        self.target_offset_px = target_offset_px
        
        # Calculate pixel to mm conversion factors
        self.robot_mm_per_px = robot_offset_mm / robot_offset_px
        self.target_mm_per_px = target_offset_mm / target_offset_px
        
        # Use average conversion factor for general calculations
        self.mm_per_px = (self.robot_mm_per_px + self.target_mm_per_px) / 2
        
        print(f"Conversion factors - Robot: {self.robot_mm_per_px:.4f} mm/px, Target: {self.target_mm_per_px:.4f} mm/px")
        print(f"Using average conversion factor: {self.mm_per_px:.4f} mm/px")
        
        self.detector = ArucoDetector(visual=visual, width=width, height=height)
        self.robot_poi = None  # Point of interest for robot
        self.target_poi = None  # Point of interest for target
        self.window_name = 'Robot and Target Tracker (mm)'
        
        # Add matrices for points of interest
        self.robot_poi_matrix = np.arange(2).reshape(1, 2)
        self.target_poi_matrix = np.arange(2).reshape(1, 2)
    
    def calculate_poi(self, center, angle, distance_px):
        """
        Calculate point of interest based on center, angle and distance
        
        Args:
            center: Center coordinates (x, y)
            angle: Rotation angle in radians
            distance_px: Distance to offset in pixels
            
        Returns:
            (x, y) coordinates of point of interest
        """
        # Calculate point of interest in front of the ArUco marker
        # Based on angle and distance
        x = int(center[0] + distance_px * np.cos(angle))
        y = int(center[1] + distance_px * np.sin(angle))
        return (x, y)
    
    def px_to_mm(self, px_x, px_y, center_x, center_y):
        """
        Convert pixel coordinates to mm coordinates
        
        Args:
            px_x, px_y: Coordinates in pixels
            center_x, center_y: Center coordinates in pixels
            
        Returns:
            (x, y) coordinates in mm
        """
        # Calculate relative pixel coordinates from center
        rel_px_x = px_x - center_x
        rel_px_y = px_y - center_y
        
        # Convert to mm using the average conversion factor
        mm_x = rel_px_x * self.mm_per_px
        mm_y = rel_px_y * self.mm_per_px
        
        return (mm_x, mm_y)
    
    def run(self):
        """
        Run tracking loop
        """
        if not self.detector.initialize_camera():
            print("Failed to initialize camera. Exiting...")
            return
        
        # Skip frames to let camera adjust
        frames_to_skip = 3
        for i in range(frames_to_skip):
            ret, _ = self.detector.camera.read()
            time.sleep(0.1)
        
        print("Camera initialized. Starting tracking...")
        center_x = self.detector.resolution_x // 2
        center_y = self.detector.resolution_y // 2
        
        while True:
            frame, points, ids = self.detector.buscar_aruco()
            
            if frame is None:
                print("Error: Could not read frame from camera. Checking connection...")
                time.sleep(1)
                continue
            
            try:
                if points is not None and len(points) > 0 and ids is not None:
                    # Draw all detected ArUco markers
                    self.detector.dibujar_aruco(frame, points, ids)
                    
                    # Find robot and target
                    ids = ids.flatten()
                    for (marker_corner, marker_id) in zip(points, ids):
                        top_left, top_right, bottom_left, bottom_right = CoordinateCalculator.get_coordinates(marker_corner)
                        
                        # Calculate midpoint
                        mid_p = np.zeros((1, 2))
                        CoordinateCalculator.mid_points(mid_p, top_right, bottom_left)
                        
                        # Calculate angle
                        angle = CoordinateCalculator.get_anglerad(bottom_right, bottom_left)
                        
                        # Determine if it's robot or target and apply the appropriate offset
                        if marker_id == self.robot_id:
                            self.robot_poi = self.calculate_poi((mid_p[0][0], mid_p[0][1]), angle, self.robot_offset_px)
                            # Convert to mm
                            mm_x, mm_y = self.px_to_mm(self.robot_poi[0], self.robot_poi[1], center_x, center_y)
                            # Draw robot POI with green color
                            ArucoDrawer.draw_punto(frame, "Robot POI", (0, 255, 0), 
                                                  self.robot_poi[0], self.robot_poi[1], 
                                                  self.detector.resolution_x, self.detector.resolution_y)
                            # Add mm coordinates to frame
                            cv2.putText(frame, f"mm: {mm_x:.1f}, {mm_y:.1f}", 
                                       (self.robot_poi[0] + 20, self.robot_poi[1] + 80), 
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)
                            print(f"Robot POI: X:{mm_x:.1f}mm, Y:{mm_y:.1f}mm")
                            
                        elif marker_id == self.target_id:
                            self.target_poi = self.calculate_poi((mid_p[0][0], mid_p[0][1]), angle, self.target_offset_px)
                            # Convert to mm
                            mm_x, mm_y = self.px_to_mm(self.target_poi[0], self.target_poi[1], center_x, center_y)
                            # Draw target POI with blue color
                            ArucoDrawer.draw_punto(frame, "Target POI", (255, 0, 0), 
                                                  self.target_poi[0], self.target_poi[1], 
                                                  self.detector.resolution_x, self.detector.resolution_y)
                            # Add mm coordinates to frame
                            cv2.putText(frame, f"mm: {mm_x:.1f}, {mm_y:.1f}", 
                                       (self.target_poi[0] + 20, self.target_poi[1] + 80), 
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2, cv2.LINE_AA)
                            print(f"Target POI: X:{mm_x:.1f}mm, Y:{mm_y:.1f}mm")
                
                # Draw center crosshair
                cv2.line(frame, (center_x - 20, center_y), (center_x + 20, center_y), (0, 0, 255), 2)
                cv2.line(frame, (center_x, center_y - 20), (center_x, center_y + 20), (0, 0, 255), 2)
                cv2.putText(frame, "(0,0)", (center_x + 30, center_y - 10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
                
                # Display pixel-to-mm conversion factor
                cv2.putText(frame, f"Scale: {self.mm_per_px:.4f} mm/px", (30, 60), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
                
                # Display frame
                cv2.putText(frame, "Press ESC to exit", (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                cv2.imshow(self.window_name, frame)
                
            except Exception as e:
                print(f"Error processing frame: {e}")
                
            if cv2.waitKey(1) & 0xFF == 27:  # Press ESC to exit
                break
        
        self.detector.close()


class ObstacleDetector:
    """
    Class for detecting obstacles using ArUco markers that are not the robot or target.
    These markers are treated as obstacles to be avoided by the robot.
    """
    def __init__(self, robot_id=0, target_id=1, obstacle_prefix="SK", visual=True, safety_radius=50):
        """
        Initialize Obstacle Detector
        
        Args:
            robot_id: ArUco ID for robot
            target_id: ArUco ID for target
            obstacle_prefix: Prefix to identify obstacles in visualization
            visual: Whether to display visual interface
            safety_radius: Safety radius around obstacles in pixels
        """
        self.robot_id = robot_id
        self.target_id = target_id
        self.obstacle_prefix = obstacle_prefix
        self.visual = visual
        self.safety_radius = safety_radius
        self.mm_per_px = 1.0  # Default value, should be updated from parent tracker
        
        # Storage for obstacles
        self.obstacles = {}  # Dictionary to store obstacle data: {id: {"position": (x, y), "mm_position": (mm_x, mm_y)}}
        
    def set_mm_per_px(self, mm_per_px):
        """
        Set the millimeters per pixel conversion factor
        
        Args:
            mm_per_px: Conversion factor from parent tracker
        """
        self.mm_per_px = mm_per_px
        
    def detect_obstacles(self, points, ids, frame=None, center_x=0, center_y=0):
        """
        Detect obstacles from ArUco markers
        
        Args:
            points: ArUco marker points
            ids: ArUco marker IDs
            frame: Input frame for visualization (optional)
            center_x: X coordinate of center point for mm conversion
            center_y: Y coordinate of center point for mm conversion
            
        Returns:
            Dictionary with detected obstacles
        """
        # Clear previous obstacles
        self.obstacles.clear()
        
        if points is None or ids is None:
            return self.obstacles
            
        ids = ids.flatten()
        
        # Process each detected ArUco marker
        for (marker_corner, marker_id) in zip(points, ids):
            # Skip if this is the robot or target
            if marker_id == self.robot_id or marker_id == self.target_id:
                continue
                
            # Get the marker coordinates
            top_left, top_right, bottom_left, bottom_right = CoordinateCalculator.get_coordinates(marker_corner)
            
            # Calculate midpoint (center of obstacle)
            mid_p = np.zeros((1, 2))
            CoordinateCalculator.mid_points(mid_p, top_right, bottom_left)
            obstacle_x, obstacle_y = int(mid_p[0][0]), int(mid_p[0][1])
            
            # Calculate position in mm
            mm_x, mm_y = self.px_to_mm(obstacle_x, obstacle_y, center_x, center_y)
            
            # Store obstacle data
            self.obstacles[marker_id] = {
                "position": (obstacle_x, obstacle_y),
                "mm_position": (mm_x, mm_y)
            }
            
            # Draw obstacle if visual and frame is provided
            if self.visual and frame is not None:
                self.draw_obstacle(frame, marker_id, obstacle_x, obstacle_y, mm_x, mm_y)
                
            print(f"Obstacle {self.obstacle_prefix}{marker_id}: X:{mm_x:.1f}mm, Y:{mm_y:.1f}mm")
                
        return self.obstacles
    
    def px_to_mm(self, px_x, px_y, center_x, center_y):
        """
        Convert pixel coordinates to mm coordinates
        
        Args:
            px_x, px_y: Coordinates in pixels
            center_x, center_y: Center coordinates in pixels
            
        Returns:
            (x, y) coordinates in mm
        """
        # Calculate relative pixel coordinates from center
        rel_px_x = px_x - center_x
        rel_px_y = px_y - center_y
        
        # Convert to mm using the conversion factor
        mm_x = rel_px_x * self.mm_per_px
        mm_y = rel_px_y * self.mm_per_px
        
        return (mm_x, mm_y)
    
    def draw_obstacle(self, frame, marker_id, x, y, mm_x, mm_y):
        """
        Draw obstacle on frame
        
        Args:
            frame: Input frame
            marker_id: ArUco marker ID
            x, y: Obstacle position in pixels
            mm_x, mm_y: Obstacle position in mm
        """
        # Draw obstacle marker with red color
        cv2.circle(frame, (x, y), self.safety_radius, (0, 0, 255), 2)  # Red circle with safety radius
        cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)  # Solid center point
        
        # Draw obstacle label
        cv2.putText(
            frame, 
            f"{self.obstacle_prefix}{marker_id}", 
            (x + 20, y - 10), 
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA
        )
        
        # Draw pixel coordinates
        cv2.putText(
            frame, 
            f"px: {x}, {y}", 
            (x + 20, y + 20), 
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1, cv2.LINE_AA
        )
        
        # Draw mm coordinates
        cv2.putText(
            frame, 
            f"mm: {mm_x:.1f}, {mm_y:.1f}", 
            (x + 20, y + 50), 
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1, cv2.LINE_AA
        )
    
    def get_closest_obstacle(self, position):
        """
        Find the closest obstacle to a given position
        
        Args:
            position: (x, y) position in mm
            
        Returns:
            Tuple of (obstacle_id, distance, position) for closest obstacle or None if no obstacles
        """
        if not self.obstacles:
            return None
            
        closest_id = None
        min_distance = float('inf')
        closest_position = None
        
        for obstacle_id, data in self.obstacles.items():
            obstacle_position = data["mm_position"]
            dx = position[0] - obstacle_position[0]
            dy = position[1] - obstacle_position[1]
            distance = (dx**2 + dy**2)**0.5
            
            if distance < min_distance:
                min_distance = distance
                closest_id = obstacle_id
                closest_position = obstacle_position
                
        return (closest_id, min_distance, closest_position)
    
    def is_path_clear(self, start_position, end_position, safety_radius_mm=None):
        """
        Check if path between two positions is clear of obstacles
        
        Args:
            start_position: (x, y) start position in mm
            end_position: (x, y) end position in mm
            safety_radius_mm: Safety radius in mm (if None, uses converted safety_radius)
            
        Returns:
            Boolean indicating if path is clear and list of obstacle IDs in the path if any
        """
        if not self.obstacles:
            return True, []
            
        if safety_radius_mm is None:
            safety_radius_mm = self.safety_radius * self.mm_per_px
            
        # Vector from start to end
        path_vector = (end_position[0] - start_position[0], end_position[1] - start_position[1])
        path_length = (path_vector[0]**2 + path_vector[1]**2)**0.5
        
        # Check each obstacle
        obstacles_in_path = []
        
        for obstacle_id, data in self.obstacles.items():
            obstacle_position = data["mm_position"]
            
            # Vector from start to obstacle
            to_obstacle = (obstacle_position[0] - start_position[0], obstacle_position[1] - start_position[1])
            
            # Calculate projection of obstacle onto path vector
            if path_length > 0:
                projection = (to_obstacle[0] * path_vector[0] + to_obstacle[1] * path_vector[1]) / path_length
            else:
                projection = 0
                
            # Find closest point on path to obstacle
            if projection < 0:
                closest_point = start_position
            elif projection > path_length:
                closest_point = end_position
            else:
                # Calculate point along path
                t = projection / path_length
                closest_point = (
                    start_position[0] + t * path_vector[0],
                    start_position[1] + t * path_vector[1]
                )
                
            # Distance from obstacle to path
            dx = obstacle_position[0] - closest_point[0]
            dy = obstacle_position[1] - closest_point[1]
            distance = (dx**2 + dy**2)**0.5
            
            # Check if obstacle is too close to path
            if distance < safety_radius_mm:
                obstacles_in_path.append(obstacle_id)
                
        return len(obstacles_in_path) == 0, obstacles_in_path


class RobotTargetObstacleTracker(RobotTargetTracker):
    """
    Extended tracker class that also handles obstacles
    """
    def __init__(self, robot_id=0, target_id=1, 
                 robot_offset_px=190, target_offset_px=83, 
                 robot_offset_mm=268, target_offset_mm=138,
                 safety_radius=50, visual=True, width=1280, height=720):
        
        # Initialize parent class
        super().__init__(robot_id, target_id, robot_offset_px, target_offset_px,
                         robot_offset_mm, target_offset_mm, visual, width, height)
        
        # Add obstacle detector
        self.obstacle_detector = ObstacleDetector(
            robot_id=robot_id, 
            target_id=target_id,
            safety_radius=safety_radius,
            visual=visual
        )
    
    def run(self):
        """
        Run tracking loop with obstacle detection
        """
        if not self.detector.initialize_camera():
            print("Failed to initialize camera. Exiting...")
            return
        
        # Skip frames to let camera adjust
        frames_to_skip = 3
        for i in range(frames_to_skip):
            ret, _ = self.detector.camera.read()
            time.sleep(0.1)
        
        print("Camera initialized. Starting tracking...")
        center_x = self.detector.resolution_x // 2
        center_y = self.detector.resolution_y // 2
        
        # Set mm per pixel conversion in obstacle detector
        self.obstacle_detector.set_mm_per_px(self.mm_per_px)
        
        while True:
            frame, points, ids = self.detector.buscar_aruco()
            
            if frame is None:
                print("Error: Could not read frame from camera. Checking connection...")
                time.sleep(1)
                continue
            
            try:
                # Process robot and target as usual
                if points is not None and len(points) > 0 and ids is not None:
                    # Draw all detected ArUco markers
                    self.detector.dibujar_aruco(frame, points, ids)
                    
                    # Find robot and target
                    ids_flat = ids.flatten()
                    for (marker_corner, marker_id) in zip(points, ids_flat):
                        if marker_id == self.robot_id or marker_id == self.target_id:
                            top_left, top_right, bottom_left, bottom_right = CoordinateCalculator.get_coordinates(marker_corner)
                            
                            # Calculate midpoint
                            mid_p = np.zeros((1, 2))
                            CoordinateCalculator.mid_points(mid_p, top_right, bottom_left)
                            
                            # Calculate angle
                            angle = CoordinateCalculator.get_anglerad(bottom_right, bottom_left)
                            
                            # Process robot or target
                            if marker_id == self.robot_id:
                                self.robot_poi = self.calculate_poi((mid_p[0][0], mid_p[0][1]), angle, self.robot_offset_px)
                                # Convert to mm
                                mm_x, mm_y = self.px_to_mm(self.robot_poi[0], self.robot_poi[1], center_x, center_y)
                                # Draw robot POI with green color
                                ArucoDrawer.draw_punto(frame, "Robot POI", (0, 255, 0), 
                                                      self.robot_poi[0], self.robot_poi[1], 
                                                      self.detector.resolution_x, self.detector.resolution_y)
                                # Add mm coordinates to frame
                                cv2.putText(frame, f"mm: {mm_x:.1f}, {mm_y:.1f}", 
                                           (self.robot_poi[0] + 20, self.robot_poi[1] + 80), 
                                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)
                                print(f"Robot POI: X:{mm_x:.1f}mm, Y:{mm_y:.1f}mm")
                                
                            elif marker_id == self.target_id:
                                self.target_poi = self.calculate_poi((mid_p[0][0], mid_p[0][1]), angle, self.target_offset_px)
                                # Convert to mm
                                mm_x, mm_y = self.px_to_mm(self.target_poi[0], self.target_poi[1], center_x, center_y)
                                # Draw target POI with blue color
                                ArucoDrawer.draw_punto(frame, "Target POI", (255, 0, 0), 
                                                      self.target_poi[0], self.target_poi[1], 
                                                      self.detector.resolution_x, self.detector.resolution_y)
                                # Add mm coordinates to frame
                                cv2.putText(frame, f"mm: {mm_x:.1f}, {mm_y:.1f}", 
                                           (self.target_poi[0] + 20, self.target_poi[1] + 80), 
                                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2, cv2.LINE_AA)
                                print(f"Target POI: X:{mm_x:.1f}mm, Y:{mm_y:.1f}mm")
                    
                    # Detect obstacles
                    obstacles = self.obstacle_detector.detect_obstacles(points, ids, frame, center_x, center_y)
                    
                    # If we have both robot and target and obstacles, check path
                    if self.robot_poi and self.target_poi and obstacles:
                        robot_mm = self.px_to_mm(self.robot_poi[0], self.robot_poi[1], center_x, center_y)
                        target_mm = self.px_to_mm(self.target_poi[0], self.target_poi[1], center_x, center_y)
                        
                        # Check if path is clear
                        is_clear, blocking_obstacles = self.obstacle_detector.is_path_clear(robot_mm, target_mm)
                        
                        # Display path status
                        status_text = "Path: CLEAR" if is_clear else f"Path: BLOCKED by SK{blocking_obstacles}"
                        cv2.putText(frame, status_text, (30, 90), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
                
                # Draw center crosshair
                cv2.line(frame, (center_x - 20, center_y), (center_x + 20, center_y), (0, 0, 255), 2)
                cv2.line(frame, (center_x, center_y - 20), (center_x, center_y + 20), (0, 0, 255), 2)
                cv2.putText(frame, "(0,0)", (center_x + 30, center_y - 10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
                
                # Display pixel-to-mm conversion factor
                cv2.putText(frame, f"Scale: {self.mm_per_px:.4f} mm/px", (30, 60), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
                
                # Display frame
                cv2.putText(frame, "Press ESC to exit", (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                cv2.imshow(self.window_name, frame)
                
            except Exception as e:
                print(f"Error processing frame: {e}")
                
            if cv2.waitKey(1) & 0xFF == 27:  # Press ESC to exit
                break
        
        self.detector.close()


# Example usage
if __name__ == "__main__":
    # Create and run the tracker with specified values
    tracker = RobotTargetTracker(
        robot_id=0, target_id=1, 
        robot_offset_px=190, target_offset_px=83,
        robot_offset_mm=268, target_offset_mm=138
    )
    tracker.run()

# if __name__ == "__main__":
#     # Create and run the tracker with obstacles
#     tracker = RobotTargetObstacleTracker(
#         robot_id=0, target_id=1, 
#         robot_offset_px=190, target_offset_px=83,
#         robot_offset_mm=268, target_offset_mm=138,
#         safety_radius=100  # Safety radius around obstacles in pixels
#     )
#     tracker.run()