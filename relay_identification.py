"""
=============================================================================
relay_identification.py
=============================================================================
Identification de la période ultime (T_u) et du gain ultime (K_u) d'un
système ball-and-beam via la méthode du relais d'Åström-Hägglund (1984).

Projet  : Ball and Beam sur UR3e
Auteur  : Polytech Angers - SAGI
Matériel: Robot UR3e | Webcam | Marqueur ArUco | Socket TCP/IP

Architecture :
  - Commande relais ±d_deg autour de la consigne x_cible
  - Enregistrement automatique des franchissements de consigne
  - Calcul de T_u, K_u, omega_u après N_CYCLES_MIN cycles stables
  - Génération des gains PID (abaques Ziegler-Nichols)
  - Affichage graphique de la trajectoire et du rapport final

Usage :
  1. Lancer le script Polyscope côté robot (écoute sur PORT 30000)
  2. Lancer ce script Python côté PC
  3. Lire le rapport imprimé en console à la fin de l'identification

=============================================================================
"""

import socket
import cv2
import cv2.aruco as aruco
import numpy as np
import time
import math
import sys
import matplotlib.pyplot as plt

# =============================================================================
# PARAMÈTRES — À MODIFIER SELON VOTRE INSTALLATION
# =============================================================================

# --- Réseau ---
ROBOT_IP   = "169.254.123.188"
ROBOT_PORT = 30000

# --- Caméra ---
CAM_INDEX  = 0          # Index de la webcam (0 = webcam par défaut)
RES_X      = 640        # Résolution horizontale en pixels
RES_Y      = 480        # Résolution verticale en pixels
CHAMP_CM   = 110.0      # Champ de vision physique mesuré en cm
RATIO      = CHAMP_CM / RES_X  # cm / pixel

# --- Géométrie ArUco ---
ARUCO_DICT = aruco.DICT_4X4_50

# --- Offset gravitaire ---
# Angle de repos mesuré expérimentalement (robot parfaitement horizontal)
ANGLE_REPOS_DEG = 181.039

# --- Paramètres du relais ---
D_DEG         = 5.0     # Amplitude de basculement du relais en degrés (±d)
                        # Doit être < LIMITE_DEG pour respecter la sécurité
LIMITE_DEG    = 10.0    # Limite absolue de sécurité (ne jamais dépasser)

# --- Consigne de position ---
X_CIBLE_CM    = 0.0     # Position cible autour de laquelle osciller (0 = centre)

# --- Critères d'arrêt ---
N_CYCLES_MIN  = 5       # Nombre minimum de cycles complets avant calcul final
DUREE_MAX_S   = 120.0   # Durée maximale de l'expérience en secondes (sécurité)

# --- Cadence ---
DT            = 0.04    # Période d'échantillonnage en secondes (25 Hz)

# =============================================================================
# FONCTIONS UTILITAIRES
# =============================================================================

def deg_to_rad(deg: float) -> float:
    return deg * math.pi / 180.0


def rad_to_deg(rad: float) -> float:
    return rad * 180.0 / math.pi


def build_servoj_command(angle_deg: float) -> str:
    """
    Construit la commande URScript servoj() à envoyer au robot.
    Seul l'axe 6 (Wrist 3 / index 5) est modifié.

    Le robot attend les angles en RADIANS.
    """
    angle_rad = deg_to_rad(angle_deg)
    # Format URScript : servoj([j0, j1, j2, j3, j4, j5], t, lookahead_time, gain)
    # Les 5 premiers axes sont figés à 0 pour cet exemple.
    # À adapter selon votre configuration home du robot.
    cmd = (
        f"servoj([0, 0, 0, 0, 0, {angle_rad:.6f}], "
        f"t={DT}, lookahead_time=0.2, gain=50)\n"
    )
    return cmd.encode("utf-8")


def detect_aruco_position(frame, aruco_dict_obj, parameters) -> float | None:
    """
    Détecte un marqueur ArUco dans l'image et retourne sa position X en cm.
    Retourne None si aucun marqueur n'est détecté.
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict_obj, parameters=parameters)

    if ids is not None and len(ids) > 0:
        # Centroïde du premier marqueur détecté
        c = corners[0][0]
        cx_pixel = float(np.mean(c[:, 0]))
        # Conversion pixel → cm, centré sur le milieu de l'image
        cx_cm = (cx_pixel - RES_X / 2.0) * RATIO
        return cx_cm
    return None


def compute_ziegler_nichols(Ku: float, Tu: float) -> dict:
    """
    Calcule les gains PID selon les abaques de Ziegler-Nichols.
    Retourne un dictionnaire avec les 4 variantes classiques.
    """
    return {
        "P": {
            "Kp": 0.50 * Ku,
            "Ki": 0.0,
            "Kd": 0.0,
        },
        "PD": {
            "Kp": 0.80 * Ku,
            "Ki": 0.0,
            "Kd": 0.80 * Ku * 0.125 * Tu,
        },
        "PID_agressif": {
            "Kp": 0.60 * Ku,
            "Ki": 0.60 * Ku / (0.50 * Tu),
            "Kd": 0.60 * Ku * 0.125 * Tu,
        },
        "PID_conservateur": {
            "Kp": 0.33 * Ku,
            "Ki": 0.33 * Ku / (0.50 * Tu),
            "Kd": 0.33 * Ku * 0.33 * Tu,
        },
    }


def print_rapport(Tu: float, Ku: float, amplitudes: list, gains: dict) -> None:
    """
    Affiche le rapport d'identification complet dans la console.
    """
    omega_u = 2.0 * math.pi / Tu
    sep = "=" * 60

    print("\n")
    print(sep)
    print("   RAPPORT D'IDENTIFICATION — MÉTHODE DU RELAIS")
    print(sep)
    print(f"  Amplitude relais (d)     : ±{D_DEG:.1f}°  (±{deg_to_rad(D_DEG):.4f} rad)")
    print(f"  Amplitude oscillations   : {np.mean(amplitudes):.2f} cm  "
          f"(moy. sur {len(amplitudes)} cycles)")
    print(sep)
    print(f"  Période ultime  T_u      : {Tu:.4f}  s")
    print(f"  Pulsation ultime ω_u     : {omega_u:.4f}  rad/s")
    print(f"  Gain ultime     K_u      : {Ku:.4f}  (rad/cm)")
    print(sep)
    print("  GAINS PID (abaques Ziegler-Nichols)")
    print(sep)
    for nom, g in gains.items():
        print(f"  [{nom}]")
        print(f"    Kp = {g['Kp']:.6f}")
        print(f"    Ki = {g['Ki']:.6f}")
        print(f"    Kd = {g['Kd']:.6f}")
        print()
    print(sep)
    print("  RECOMMANDATION : Commencer avec PID_conservateur")
    print("  puis affiner vers PID_agressif si la réponse est trop lente.")
    print(sep)


def plot_resultats(timestamps_x: list, positions_x: list,
                   timestamps_u: list, commandes_u: list,
                   crossings: list, Tu: float) -> None:
    """
    Affiche deux graphiques :
      1. Position du cube au fil du temps avec marquage des franchissements
      2. Commande relais (angle) au fil du temps
    """
    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    fig.suptitle(
        f"Identification par relais — T_u = {Tu:.4f} s  |  "
        f"ω_u = {2*math.pi/Tu:.4f} rad/s",
        fontsize=13, fontweight="bold"
    )

    # --- Graphique 1 : Position ---
    ax1 = axes[0]
    ax1.plot(timestamps_x, positions_x, color="steelblue", linewidth=1.5,
             label="Position cube (cm)")
    ax1.axhline(y=X_CIBLE_CM, color="red", linestyle="--",
                linewidth=1.0, label=f"Consigne = {X_CIBLE_CM} cm")
    for t_cross in crossings:
        ax1.axvline(x=t_cross, color="orange", linestyle=":", linewidth=0.8)
    ax1.set_ylabel("Position (cm)")
    ax1.legend(loc="upper right")
    ax1.grid(True, linestyle="--", alpha=0.5)
    ax1.set_title("Position du cube")

    # --- Graphique 2 : Commande ---
    ax2 = axes[1]
    ax2.step(timestamps_u, commandes_u, color="darkorange", linewidth=1.5,
             where="post", label="Commande relais (°)")
    ax2.axhline(y=0, color="gray", linestyle="--", linewidth=0.8)
    ax2.set_ylabel("Angle relais (°)")
    ax2.set_xlabel("Temps (s)")
    ax2.legend(loc="upper right")
    ax2.grid(True, linestyle="--", alpha=0.5)
    ax2.set_title("Commande relais envoyée au robot")

    plt.tight_layout()
    plt.savefig("relay_identification_graph.png", dpi=150)
    print("  Graphique sauvegardé : relay_identification_graph.png")
    plt.show()


# =============================================================================
# PROGRAMME PRINCIPAL
# =============================================================================

def main():
    print("=" * 60)
    print("  relay_identification.py — Démarrage")
    print(f"  Cible        : {X_CIBLE_CM} cm")
    print(f"  Relais       : ±{D_DEG}°")
    print(f"  Cycles min   : {N_CYCLES_MIN}")
    print(f"  Durée max    : {DUREE_MAX_S} s")
    print("=" * 60)

    # --- Vérification de sécurité ---
    if D_DEG >= LIMITE_DEG:
        print(f"[ERREUR] D_DEG ({D_DEG}°) >= LIMITE_DEG ({LIMITE_DEG}°). Abandon.")
        sys.exit(1)

    # --- Initialisation caméra ---
    print("[INFO] Ouverture de la caméra...")
    cap = cv2.VideoCapture(CAM_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  RES_X)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, RES_Y)
    if not cap.isOpened():
        print("[ERREUR] Impossible d'ouvrir la caméra.")
        sys.exit(1)

    aruco_dict_obj = aruco.getPredefinedDictionary(ARUCO_DICT)
    aruco_params   = aruco.DetectorParameters()

    # --- Connexion socket robot ---
    print(f"[INFO] Connexion au robot {ROBOT_IP}:{ROBOT_PORT}...")
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((ROBOT_IP, ROBOT_PORT))
        sock.setblocking(False)
        print("[INFO] Connexion établie.")
    except Exception as e:
        print(f"[ERREUR] Connexion impossible : {e}")
        cap.release()
        sys.exit(1)

    # --- Variables d'état ---
    erreur_precedente  = None   # Erreur au cycle précédent (pour détection franchissement)
    commande_actuelle  = +D_DEG # Commande relais courante (en degrés relatifs)

    # Listes d'enregistrement
    timestamps_x  = []          # Temps de chaque mesure (s)
    positions_x   = []          # Position mesurée (cm)
    timestamps_u  = []          # Temps de chaque commande (s)
    commandes_u   = []          # Commande envoyée (°)
    crossings     = []          # Timestamps des franchissements de consigne
    amplitudes    = []          # Demi-amplitudes crête mesurées par cycle

    # Suivi des extrema pour calcul de l'amplitude
    pos_max_courant = -np.inf
    pos_min_courant = +np.inf
    en_montee       = True      # Sens courant du déplacement du cube

    t_debut = time.time()
    cycles_complets = 0
    Tu = None
    Ku = None

    print("[INFO] Identification en cours... (Ctrl+C pour interrompre)")
    print(f"       Attente de {N_CYCLES_MIN} cycles complets.\n")

    try:
        while True:
            t_cycle_start = time.time()

            # --- Acquisition image ---
            ret, frame = cap.read()
            if not ret:
                print("[WARN] Frame non lue, cycle ignoré.")
                continue

            # --- Détection ArUco ---
            position_cm = detect_aruco_position(frame, aruco_dict_obj, aruco_params)

            if position_cm is None:
                # Pas de marqueur détecté : on envoie la dernière commande connue
                cv2.imshow("Relay Identification", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
                # Busy-wait pour respecter DT
                while (time.time() - t_cycle_start) < DT:
                    pass
                continue

            erreur_actuelle = X_CIBLE_CM - position_cm

            # --- Enregistrement position ---
            t_now = time.time() - t_debut
            timestamps_x.append(t_now)
            positions_x.append(position_cm)

            # --- Détection franchissement de consigne ---
            if erreur_precedente is not None:
                signe_change = (erreur_precedente * erreur_actuelle) < 0

                if signe_change:
                    crossings.append(t_now)
                    n_cross = len(crossings)

                    # Un cycle complet = 2 franchissements successifs
                    if n_cross >= 2:
                        T_demi = crossings[-1] - crossings[-2]  # demi-période
                        T_cycle = 2.0 * T_demi                  # période complète
                        cycles_complets = (n_cross - 1) // 2 + 1

                        # Amplitude du demi-cycle écoulé
                        if en_montee:
                            amplitude = abs(pos_max_courant - X_CIBLE_CM)
                        else:
                            amplitude = abs(pos_min_courant - X_CIBLE_CM)
                        amplitudes.append(amplitude)

                        print(
                            f"  Cycle {cycles_complets:3d} | "
                            f"T_demi = {T_demi:.3f} s → T_cycle ≈ {T_cycle:.3f} s | "
                            f"Amplitude = {amplitude:.2f} cm"
                        )

                        # Reset extrema pour le prochain demi-cycle
                        pos_max_courant = -np.inf
                        pos_min_courant = +np.inf
                        en_montee = not en_montee

                    # Inversion du relais au franchissement
                    commande_actuelle = -commande_actuelle

            # --- Suivi des extrema de position ---
            pos_max_courant = max(pos_max_courant, position_cm)
            pos_min_courant = min(pos_min_courant, position_cm)

            # --- Calcul de la commande à envoyer ---
            angle_absolu = ANGLE_REPOS_DEG + commande_actuelle
            # Sécurité : clamp absolu
            angle_absolu = max(
                ANGLE_REPOS_DEG - LIMITE_DEG,
                min(ANGLE_REPOS_DEG + LIMITE_DEG, angle_absolu)
            )

            # --- Envoi commande robot ---
            try:
                sock.send(build_servoj_command(angle_absolu))
            except BlockingIOError:
                pass  # Buffer plein, on ignore et on envoie au prochain cycle

            timestamps_u.append(t_now)
            commandes_u.append(commande_actuelle)

            # --- Affichage caméra ---
            cv2.putText(
                frame,
                f"Pos: {position_cm:.1f} cm | Err: {erreur_actuelle:.1f} cm | "
                f"Cmd: {commande_actuelle:+.1f}° | Cycles: {cycles_complets}",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 0), 2
            )
            cv2.imshow("Relay Identification", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                print("[INFO] Arrêt demandé par l'utilisateur.")
                break

            erreur_precedente = erreur_actuelle

            # --- Condition d'arrêt normale ---
            if cycles_complets >= N_CYCLES_MIN and len(crossings) >= 2 * N_CYCLES_MIN:
                print(f"\n[INFO] {N_CYCLES_MIN} cycles atteints. Fin de l'identification.")
                break

            # --- Condition d'arrêt sécurité temporelle ---
            if (time.time() - t_debut) > DUREE_MAX_S:
                print(f"\n[WARN] Durée maximale ({DUREE_MAX_S} s) atteinte. Arrêt forcé.")
                break

            # --- Busy-wait précis pour respecter DT ---
            while (time.time() - t_cycle_start) < DT:
                pass

    except KeyboardInterrupt:
        print("\n[INFO] Interruption clavier.")

    finally:
        # --- Remise à zéro du robot ---
        print("[INFO] Remise à zéro du robot (angle de repos)...")
        try:
            sock.send(build_servoj_command(ANGLE_REPOS_DEG))
            time.sleep(0.5)
        except Exception:
            pass
        sock.close()
        cap.release()
        cv2.destroyAllWindows()

    # ==========================================================================
    # POST-TRAITEMENT
    # ==========================================================================

    if len(crossings) < 4:
        print("[ERREUR] Pas assez de cycles enregistrés pour calculer T_u.")
        print(f"         Franchissements détectés : {len(crossings)}")
        print("         Vérifiez : caméra, amplitude D_DEG, position initiale.")
        sys.exit(1)

    # --- Calcul de T_u (moyenne sur tous les intervalles entre franchissements) ---
    intervalles = []
    for i in range(1, len(crossings)):
        intervalles.append(crossings[i] - crossings[i - 1])

    # T_u = 2 × demi-période moyenne
    Tu = 2.0 * np.mean(intervalles)
    ecart_type = 2.0 * np.std(intervalles)
    omega_u = 2.0 * math.pi / Tu

    # --- Calcul de K_u via la formule du relais ---
    # K_u = (4 × d) / (π × a)
    # d = amplitude de la commande relais en RADIANS
    # a = amplitude moyenne des oscillations du cube en cm
    d_rad   = deg_to_rad(D_DEG)
    a_moyen = np.mean(amplitudes) if amplitudes else 1.0  # sécurité division /0
    Ku      = (4.0 * d_rad) / (math.pi * a_moyen)

    # --- Génération des gains PID ---
    gains = compute_ziegler_nichols(Ku, Tu)

    # --- Rapport console ---
    print(f"\n  Ecart-type sur T_u   : ±{ecart_type:.4f} s  "
          f"({'stable' if ecart_type < 0.1 * Tu else 'variable — à investiguer'})")
    print_rapport(Tu, Ku, amplitudes, gains)

    # --- Graphique ---
    plot_resultats(
        timestamps_x, positions_x,
        timestamps_u, commandes_u,
        crossings, Tu
    )


if __name__ == "__main__":
    main()