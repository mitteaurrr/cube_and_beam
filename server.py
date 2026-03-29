import cv2
import numpy as np
import socket
import sys
import time
import math

HOST = '169.254.123.188'
PORT = 30000
LARGEUR_REELLE_MM = 300
RES_X = 640
RES_Y = 480
RATIO = LARGEUR_REELLE_MM / RES_X

CIBLE_X_MM = 0.0
KP = 0.4
KD = 0.05
ANGLE_REPOS_DEG = 180.0
LIMITE_DEG = 10.0

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

try:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((HOST, PORT))
    s.listen(1)
    print("Attente connexion robot...")
    conn, addr = s.accept()
    conn.setblocking(False)
    print(f"Connecte: {addr}")
except Exception as e:
    print(e)
    sys.exit()

cam = cv2.VideoCapture(0, cv2.CAP_V4L2)
cam.set(cv2.CAP_PROP_FRAME_WIDTH, RES_X)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, RES_Y)
time.sleep(1.0)

erreur_prec = 0.0
theta6_rad = math.radians(ANGLE_REPOS_DEG)
last_time = time.time()

try:
    while True:
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time
        start_time = current_time
        
        ret, frame = cam.read()
        if not ret:
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = detector.detectMarkers(gray)

        x_mm = 0.0

        if ids is not None and len(ids) > 0:
            c = corners[0][0]
            center_x = np.mean(c[:, 0])
            center_y = np.mean(c[:, 1])

            x_mm = (center_x - (RES_X / 2)) * RATIO
            
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            cv2.circle(frame, (int(center_x), int(center_y)), 5, (0, 0, 255), -1)
            
            texte = f"X: {x_mm:.1f} mm"
            cv2.putText(frame, texte, (int(center_x) + 10, int(center_y) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            erreur = CIBLE_X_MM - x_mm
            
            if dt <= 0.0:
                dt = 0.001
                
            derivee = (erreur - erreur_prec) / dt 
            
            commande_deg = (erreur * KP) + (derivee * KD)
            erreur_prec = erreur

            if commande_deg > LIMITE_DEG:
                commande_deg = LIMITE_DEG
            elif commande_deg < -LIMITE_DEG:
                commande_deg = -LIMITE_DEG

            angle_absolu_deg = ANGLE_REPOS_DEG - commande_deg
            
            theta6_rad = math.radians(angle_absolu_deg)

        else:
            theta6_rad = math.radians(ANGLE_REPOS_DEG)
            erreur_prec = 0.0

        message = f"({theta6_rad:.4f})\n"
        print(message)
        
        try:
            conn.sendall(message.encode('ascii'))
        except BlockingIOError:
            pass
        except BrokenPipeError:
            break
        
        cv2.imshow("Camera", frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        process_time = time.time() - start_time
        if process_time < 0.04:
            time.sleep(0.04 - process_time)

except KeyboardInterrupt:
    pass

finally:
    conn.close()
    cam.release()
    cv2.destroyAllWindows()