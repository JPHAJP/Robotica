import cv2
import numpy as np
import math
import time
import os


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


# Example usage
if __name__ == "__main__":
    # Create and run the detector
    detector = ArucoDetector(visual=True, width=1280, height=720)
    detector.run_detection()