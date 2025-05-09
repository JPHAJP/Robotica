import cv2  # pip install opencv-python, opencv-contrib-python
import numpy as np  # pip install numpy
import math  # default pip install python-math
import time  # For adding delay between attempts
import os  # For directory operations and file checks

# pip opencv-contrib-python: descarga libreria cv2, aruco, y si es necesario, numpy

# modifica imagen, le reduce el brillo
def change_brightness(img, value):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    v = cv2.add(v, value)
    v[v > 255] = 255
    v[v < 0] = 0
    final_hsv = cv2.merge((h, s, v))
    img = cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)

    return img

# funcion que consigue punto medio del eje x y y
def mid_points(matrix, pt1, pt2):
    matrix[0][0] = (pt1[0] + pt2[0]) / 2
    matrix[0][1] = (pt1[1] + pt2[1]) / 2
    return matrix

# conseguir el angulo de rotacion de los codigos aruco
def get_angle(bottomRight, bottomLeft):
    x = (bottomRight[0] - bottomLeft[0])
    y = (bottomRight[1] - bottomLeft[1])
    angle = math.atan2(y, x)
    angle = math.degrees(angle)
    angle *= -1

    if angle < 0:
        angle += 360
    angle = round(angle, 2)
    angle = abs(angle)
    return angle

# consigue angulo de rotacion de los codigos aruco en radianes
def get_anglerad(bottomRight, bottomLeft):
    x = (bottomRight[0] - bottomLeft[0])
    y = (bottomRight[1] - bottomLeft[1])
    angle = math.atan2(y, x)
    """ 
    angle = math.degrees(angle)
    angle *= -1

    if angle < 0:
      angle += 360

    angle = abs(angle)
    angle = math.radians(angle) 
    """
    angle = round(angle, 2)
    return angle

# funcion que dibuja e imprime informacion en el frame de opencv
def draw_aruco(frame, topLeft, topRight, bottomLeft, bottomRight, MidP, X, Y, angle, markerID, resolucionx, resoluciony):
    # Se dibuja el cuadrado 
    cv2.line(frame, topLeft, topRight, (0, 255, 0), 2)
    cv2.line(frame, topRight, bottomRight, (0, 255, 0), 2)
    cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
    cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)

    # Se dibujan las lineas 
    line_thickness = 3
    cv2.line(frame, X[0], MidP[0], (0, 0, 255), thickness=line_thickness)
    cv2.line(frame, Y[0], MidP[0], (255, 0, 0), thickness=line_thickness)

    # Se imprime la información del texto 
    cv2.putText(frame, "[" + str(MidP[0][0] - (resolucionx / 2)) + " ," + str(MidP[0][1] - (resoluciony / 2)) + "]", (MidP[0][0], MidP[0][1] - 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 204, 204), 2, cv2.LINE_AA)
    cv2.putText(frame, str(f'ID: {markerID}'), (MidP[0][0] - 50, MidP[0][1] - 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 204, 204), 2, cv2.LINE_AA)
    # cv2.putText(frame, str(angle) + " grados", (MidP[0][0] - 400, MidP[0][1] - 200), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 204, 204), 2, cv2.LINE_AA )

# funcion que dibuja e imprime informacion en el frame de opencv
def draw_texto_titulo(frame, text, color):
    # Se imprime la información del texto 
    cv2.putText(frame, text, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2, cv2.LINE_AA)

def draw_punto(frame, text, color, posx, posy, resolucionx, resoluciony):
    # Se imprime la información del texto 
    cv2.putText(frame, text, (posx + 20, posy + 20), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2, cv2.LINE_AA)
    cv2.putText(frame, str(posx - resolucionx / 2) + "," + str(posy - (resoluciony / 2)), (posx + 20, posy + 50), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2, cv2.LINE_AA)
    cv2.circle(frame, (posx, posy), 10, color, 5)

# conseguimos las coordenadas del aruco y lo guadamos como pares (x y y) en variables por seccion diferente
def get_coordenates(markerCorner):
    # extract the marker corners (which are always returned in
    # top-left, top-right, bottom-right, and bottom-left order)
    corners = markerCorner.reshape((4, 2))
    (topLeft, topRight, bottomRight, bottomLeft) = corners

    # convert each of the (x, y)-coordinate pairs to integers
    topRight = (int(topRight[0]), int(topRight[1]))
    bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
    bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
    topLeft = (int(topLeft[0]), int(topLeft[1]))

    return topLeft, topRight, bottomLeft, bottomRight

# almacenamos info de coordenadas y angulo del aruco, y la almacenamos en un diccionario
def get_ArucoInfo(markerCorner, markerID):
    topLeft, topRight, bottomLeft, bottomRight = get_coordenates(markerCorner)

    # Calculamos el angulo de inclinación 
    angle = get_anglerad(bottomRight, bottomLeft)

    info = {"coordenadas": [topLeft, topRight, bottomLeft, bottomRight], "angulo": (angle), "ID": (markerID)}

    return info

def initialize_camera():
    """
    Try different methods to initialize the camera on Linux with enhanced timeout handling
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
        return None
        
    # Set camera properties - use lower resolution first to ensure compatibility
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
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
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            return capture
        else:
            print(f"Camera test attempt {i+1}/5 failed, retrying...")
            time.sleep(1)
    
    print("Failed to read frames from camera after initialization")
    return None

def buscar_Aruco(camara, resolucionx, resoluciony):
    if camara is None or not camara.isOpened():
        return None, None, None
        
    ret, frame = camara.read()
    if not ret or frame is None:
        return None, None, None

    frame = cv2.resize(frame, (resolucionx, resoluciony))  # Cambiar el tamaño de la ventana que despliega
    frame = change_brightness(frame, 10)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
    arucoParams = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)  ########
    points, ids, rejected = detector.detectMarkers(gray)

    return frame, points, ids

def dibujar_aruco(frame, points, ids, resolucionx, resoluciony):
    if frame is None:
        return

    MidP = np.arange(2).reshape(1, 2)
    Y = np.arange(2).reshape(1, 2)
    X = np.arange(2).reshape(1, 2)
    # window_name = 'Camara detector qr' #Nombre de la ventana

    if points is not None and len(points) > 0 and ids is not None:
        # flatten the ArUco IDs list
        ids = ids.flatten()
        # loop over the detected ArUCo corners
        for (markerCorner, markerID) in zip(points, ids):
            topLeft, topRight, bottomLeft, bottomRight = get_coordenates(markerCorner)

            # Obtenemos coordenadas para punto medio y lineas
            mid_points(MidP, topRight, bottomLeft)
            mid_points(Y, topRight, bottomRight)
            mid_points(X, bottomLeft, bottomRight)

            # Calculamos el angulo de inclinación 
            angle = get_anglerad(bottomRight, bottomLeft)

            draw_aruco(frame, topLeft, topRight, bottomLeft, bottomRight, MidP, X, Y, angle, markerID, resolucionx, resoluciony)

    # cv2.imshow(window_name, frame) #Despliega la ventana 

def buscar_robots(points, ids, robot):
    MidP = np.arange(2).reshape(1, 2)
    if points is not None and len(points) > 0 and ids is not None:
        # flatten the ArUco IDs list
        ids = ids.flatten()
        # loop over the detected ArUCo corners
        for (markerCorner, markerID) in zip(points, ids):
            topLeft, topRight, bottomLeft, bottomRight = get_coordenates(markerCorner)

            # Obtenemos coordenadas para punto medio
            mid_points(MidP, topRight, bottomLeft)
            robot[markerID][0] = MidP[0][0]
            robot[markerID][1] = MidP[0][1]
            robot[markerID][2] = get_anglerad(bottomRight, bottomLeft)

    return robot

def preview(camara, resolucionx, resoluciony):
    if camara is None or not camara.isOpened():
        return
        
    ret, frame = camara.read()
    if not ret or frame is None:
        return

    frame = cv2.resize(frame, (resolucionx, resoluciony))  # Cambiar el tamaño de la ventana que despliega
    frame = change_brightness(frame, 10)
    window_name = 'Camara detector qr'  # Nombre de la ventana
    cv2.imshow(window_name, frame)  # Despliega la ventana 


##################################################################################################################
##################################################################################################################
##################################################################################################################
##################################################################################################################
##CODIGO EJEMPLO##
# si es True, se visualizara la camara y su interfaz, si es falsa ejecutará la camara sin mostrar nada
visual = True

import os  # Add this import at the top of the file

if __name__ == "__main__":
    # Initialize camera with improved method
    capture = initialize_camera()
    
    if capture is None:
        print("Failed to initialize camera. Exiting...")
        exit()
    
    window_name = 'Camara detector qr'  # Nombre de la ventana
    
    # Start with lower resolution for compatibility
    resolucionx = 640
    resoluciony = 480
    
    # If the camera allows setting to HD resolution, try it
    if capture.get(cv2.CAP_PROP_FRAME_WIDTH) == 1280 and capture.get(cv2.CAP_PROP_FRAME_HEIGHT) == 720:
        resolucionx = 1280
        resoluciony = 720
        print("Using HD resolution (1280x720)")
    else:
        print(f"Using resolution: {int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))}")
    
    points = np.arange(8).reshape(4, 2)
    MidP = np.arange(2).reshape(1, 2)
    Y = np.arange(2).reshape(1, 2)
    X = np.arange(2).reshape(1, 2)
    
    # Skip fewer frames initially
    frames_to_skip = 3
    for i in range(frames_to_skip):
        ret, _ = capture.read()  # Read and discard frames to let camera adjust
        time.sleep(0.1)
    
    print("Camera initialized. Starting detection...")
    
    while True:
        print("Esperando a que detecte un aruco")
        ret, frame = capture.read()
        
        # Check if the frame was successfully read
        if not ret or frame is None:
            print("Error: No se pudo leer el frame de la cámara. Verificando la conexión...")
            time.sleep(1)  # Wait a second before trying again
            continue  # Skip the rest of this loop iteration
        
        # Process the frame since ret is True
        try:
            # Check for NULL or empty frame before processing
            if frame is None or frame.size == 0:
                print("Warning: Empty frame received. Skipping...")
                time.sleep(0.1)
                continue
                
            # Print frame info for debugging
            print(f"Frame shape: {frame.shape}, dtype: {frame.dtype}")
                
            # Handle frame resizing carefully
            current_height, current_width = frame.shape[:2]
            if current_width != resolucionx or current_height != resoluciony:
                frame = cv2.resize(frame, (resolucionx, resoluciony))
                
            frame = change_brightness(frame, 10)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
            arucoParams = cv2.aruco.DetectorParameters()
            detector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)
            points, ids, rejected = detector.detectMarkers(gray)
            
            if points is not None and len(points) > 0 and ids is not None:
                # flatten the ArUco IDs list
                ids = ids.flatten()
                # loop over the detected ArUCo corners
                for (markerCorner, markerID) in zip(points, ids):
                    topLeft, topRight, bottomLeft, bottomRight = get_coordenates(markerCorner)
                    
                    # Obtenemos coordenadas para punto medio y lineas
                    mid_points(MidP, topRight, bottomLeft)
                    mid_points(Y, topRight, bottomRight)
                    mid_points(X, bottomLeft, bottomRight)
                    
                    # Calculamos el angulo de inclinación 
                    angle = get_angle(bottomRight, bottomLeft)
                    
                    if visual:
                        draw_aruco(frame, topLeft, topRight, bottomLeft, bottomRight, MidP, X, Y, angle, markerID, resolucionx, resoluciony)
                    
                    print("ID:" + str(markerID) + ", X:" + str(MidP[0][0] - (resolucionx / 2)) + ", Y:" + str(MidP[0][1] - (resoluciony / 2)) + ", Th:" + str(angle))
            
            if visual:
                cv2.putText(frame, "Press ESC to exit", (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                cv2.imshow(window_name, frame)  # Despliega la ventana 
            
        except Exception as e:
            print(f"Error processing frame: {e}")
            
        if cv2.waitKey(1) & 0xFF == 27:  # Presiona ESC para salir
            break
    
    capture.release()
    cv2.destroyAllWindows()