import socket
import sys
import time
import math

HOST = '169.254.123.188'
PORT = 30000

# --- PARAMETRES DU SIGNAL SINUSOIDAL ---
ANGLE_REPOS_DEG = 180.0
AMPLITUDE_DEG = 3.0       # L'angle maximal de basculement (+- 3 degrés)
PERIODE_SEC = 5.0         # Le temps en secondes pour faire une oscillation complète
# ---------------------------------------

try:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((HOST, PORT))
    s.listen(1)
    print("Attente connexion robot (Test Sinusoidal)...")
    conn, addr = s.accept()
    conn.setblocking(False)
    print(f"Connecte: {addr}")
except Exception as e:
    print(e)
    sys.exit()

print(f"Generation sinusoide : Amplitude {AMPLITUDE_DEG} deg, Periode {PERIODE_SEC} s")
start_time = time.time()

try:
    while True:
        # 1. Calcul du temps écoulé depuis le début du test
        t = time.time() - start_time
        
        # 2. Equation mathématique de la sinusoïde
        omega = (2 * math.pi) / PERIODE_SEC
        oscillation_deg = AMPLITUDE_DEG * math.sin(omega * t)
        
        # 3. Calcul de l'angle final absolu et conversion
        angle_absolu_deg = ANGLE_REPOS_DEG + oscillation_deg
        theta6_rad = math.radians(angle_absolu_deg)

        # 4. Ecoute du réseau et envoi au robot
        try:
            requete = conn.recv(1024)
            if requete:
                message = f"({theta6_rad:.4f})\n"
                conn.sendall(message.encode('ascii'))
                
                # Affichage dans la console pour monitorer l'évolution
                print(f"T: {t:.1f}s | Angle commande: {angle_absolu_deg:.2f} deg")
        except BlockingIOError:
            pass
        except BrokenPipeError:
            break
            
        # Courte pause pour soulager le processeur de la Raspberry Pi
        time.sleep(0.01)

except KeyboardInterrupt:
    pass

finally:
    conn.close()
    print("Fin du test.")