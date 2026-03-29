import cv2
import numpy as np
import socket
import sys
import time
import math

HOST = '169.254.123.188' #adresse IP du robot
PORT = 30000 # port de communication réseau
LARGEUR_REELLE_MM = 300 # taille réelle en millimètres de la zone visible
RES_X = 640 # résolution horizontale de la caméra en pixels
RES_Y = 480 # résolution verticale de la caméra en pixels
RATIO = LARGEUR_REELLE_MM / RES_X # coefficient pour convertir les pixels en millimètres

CIBLE_X_MM = 0.0 # La position de consigne où le cube doit aller (0.0 = le centre)
KP = 0.4 # Gain Proportionnel : définit la force de la correction (réaction forte si loin, faible si proche)
KD = 0.05 # Gain Dérivé : freine le mouvement à l'approche de la cible pour éviter de dépasser
ANGLE_REPOS_DEG = 180.0 # L'angle physique où la règle est parfaitement à l'horizontale
LIMITE_DEG = 10.0 # La sécurité maximale : le robot ne s'inclinera jamais de plus de 10 degrés

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50) # Charge le dictionnaire des marqueurs Aruco
parameters = cv2.aruco.DetectorParameters() # Charge les paramètres par défaut du détecteur
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters) # Initialise le détecteur d'Aruco

try: # Essaie d'exécuter ce bloc de code pour le réseau
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # Crée le socket TCP/IP
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) # Permet de relancer le programme sans erreur de port bloqué
    s.bind((HOST, PORT)) # Lie le socket à l'adresse IP et au port définis
    s.listen(1) # Met le serveur en écoute d'une connexion entrante (le robot)
    print("Attente connexion robot...") # Affiche un message dans la console
    conn, addr = s.accept() # Accepte la connexion du robot quand il se connecte
    conn.setblocking(False) # Rend le socket non-bloquant pour ne pas figer le programme Python
    print(f"Connecte: {addr}") # Affiche l'adresse IP du robot connecté
except Exception as e: # Si une erreur réseau survient
    print(e) # Affiche l'erreur dans la console
    sys.exit() # Arrête complètement le programme Python

cam = cv2.VideoCapture(0, cv2.CAP_V4L2) # Ouvre le flux vidéo de la webcam branchée en USB
cam.set(cv2.CAP_PROP_FRAME_WIDTH, RES_X) # Force la largeur de l'image de la caméra
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, RES_Y) # Force la hauteur de l'image de la caméra
time.sleep(1.0) # Fait une pause de 1 seconde pour laisser la caméra s'allumer

erreur_prec = 0.0 # Initialise la mémoire de l'erreur précédente à 0 pour la dérivée
theta6_rad = math.radians(ANGLE_REPOS_DEG) # Convertit l'angle de repos (180°) en radians (approx 3.1416)

try: # Démarre le bloc principal qui tournera à l'infini
    while True: # Boucle infinie
        start_time = time.time() # Enregistre l'heure exacte au début de ce cycle
        
        ret, frame = cam.read() # Lit une image depuis la caméra
        if not ret: # Si la lecture de l'image a échoué
            continue # Saute le reste de la boucle et recommence au début

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # Convertit l'image en noir et blanc pour faciliter la détection
        corners, ids, rejected = detector.detectMarkers(gray) # Cherche les marqueurs Aruco dans l'image

        x_mm = 0.0 # Initialise la coordonnée X du cube à 0 par défaut

        if ids is not None and len(ids) > 0: # Si au moins un marqueur Aruco est détecté
            c = corners[0][0] # Récupère les 4 coins du premier marqueur trouvé
            center_x = np.mean(c[:, 0]) # Calcule la position X du centre du marqueur en pixels
            center_y = np.mean(c[:, 1]) # Calcule la position Y du centre du marqueur en pixels

            x_mm = (center_x - (RES_X / 2)) * RATIO # Convertit la position X en millimètres par rapport au centre de l'image
            
            cv2.aruco.drawDetectedMarkers(frame, corners, ids) # Dessine un carré vert autour du marqueur sur l'écran
            cv2.circle(frame, (int(center_x), int(center_y)), 5, (0, 0, 255), -1) # Dessine un point rouge au centre du marqueur
            
            texte = f"X: {x_mm:.1f} mm" # Prépare le texte affichant la position X avec 1 décimale
            cv2.putText(frame, texte, (int(center_x) + 10, int(center_y) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2) # Affiche le texte sur l'image

            erreur = CIBLE_X_MM - x_mm # Calcule l'écart entre la position voulue et la position réelle (+25 ou -10 par exemple)
            derivee = erreur - erreur_prec # Calcule la vitesse de déplacement de l'erreur
            commande_deg = (erreur * KP) + (derivee * KD) # Calcule l'angle d'inclinaison nécessaire pour corriger l'erreur
            erreur_prec = erreur # Sauvegarde l'erreur actuelle pour le prochain tour de boucle

            if commande_deg > LIMITE_DEG: # Si l'angle demandé est plus grand que la limite autorisée (ex: > 10)
                commande_deg = LIMITE_DEG # Force l'angle à la valeur limite de sécurité (+10)
            elif commande_deg < -LIMITE_DEG: # Si l'angle demandé est plus petit que la limite négative (ex: < -10)
                commande_deg = -LIMITE_DEG # Force l'angle à la valeur limite de sécurité négative (-10)

            angle_absolu_deg = ANGLE_REPOS_DEG - commande_deg # Calcule l'angle final du robot (le '-' assure que le robot tourne du bon côté)
            
            theta6_rad = math.radians(angle_absolu_deg) # Convertit l'angle final de degrés vers radians pour l'envoyer au robot

        else: # Si le marqueur Aruco n'est plus visible (cube tombé ou caché)
            theta6_rad = math.radians(ANGLE_REPOS_DEG) # Force le robot à se remettre parfaitement à l'horizontale
            erreur_prec = 0.0 # Réinitialise la mémoire du correcteur

        message = f"({theta6_rad:.4f})\n" # Formate le message réseau avec l'angle en radians et 4 décimales
        print(message) # Affiche la donnée envoyée dans la console Python pour vérifier le bon fonctionnement
        
        try: # Essaie d'envoyer la donnée au robot
            conn.sendall(message.encode('ascii')) # Envoie physiquement le message converti en texte ASCII dans le câble Ethernet
        except BlockingIOError: # Si le réseau est momentanément saturé (non-bloquant)
            pass # Ne fait rien et continue
        except BrokenPipeError: # Si le robot s'est déconnecté brutalement
            break # Casse la boucle infinie pour arrêter le programme
        
        cv2.imshow("Camera", frame) # Affiche la fenêtre avec le retour vidéo de la webcam
        
        if cv2.waitKey(1) & 0xFF == ord('q'): # Attend 1 milliseconde, si la touche 'q' est pressée
            break # Casse la boucle infinie pour fermer la fenêtre

        process_time = time.time() - start_time # Calcule le temps qu'a pris le programme pour faire tout ce cycle
        if process_time < 0.04: # Si le cycle a pris moins de 0.04 seconde (pour viser 25 images par seconde)
            time.sleep(0.04 - process_time) # Fait une pause pour attendre la fin des 0.04 secondes et ne pas noyer le robot de données

except KeyboardInterrupt: # Si l'utilisateur fait CTRL+C dans la console
    pass # Ne fait rien, passe directement au bloc 'finally'

finally: # Code exécuté obligatoirement à la fin, quoi qu'il arrive (erreur ou arrêt manuel)
    conn.close() # Coupe proprement la connexion réseau avec le robot
    cam.release() # Libère l'accès à la webcam pour les autres programmes
    cv2.destroyAllWindows() # Ferme la fenêtre d'affichage vidéo