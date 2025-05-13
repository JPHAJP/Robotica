import cv2
import numpy as np
import time
from aruco_tracker import ArucoDetector, ImageProcessor, ArucoDrawer, CoordinateCalculator

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

if __name__ == "__main__":
    # Create and run the tracker with real-world measurements
    tracker = RobotTargetTracker(
        robot_id=0, target_id=1, 
        robot_offset_px=190, target_offset_px=83,
        robot_offset_mm=268, target_offset_mm=138
    )
    tracker.run()
    # 268 mm de aruco a POI robot
    # 136 mm de aruco a POI target 
    # ESTOS DATOS SON CONSTANTES SIN IMPORTAR LA ALTURA DE LA CAMARA