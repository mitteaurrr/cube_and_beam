import math

# =========================================================
# 1. VOS RELEVÉS EXPÉRIMENTAUX (Essai sinusoïdal)
# =========================================================
X_MAX_MM = 30.0         # Amplitude maximale mesurée sur le rail (mm)
AMPLITUDE_DEG = 3.0     # Amplitude de l'angle envoyée au robot (degrés)
PERIODE_SEC = 5.0       # Période d'oscillation envoyée au robot (secondes)

# =========================================================
# 2. VOS PERFORMANCES DÉSIRÉES (Théorie de l'automatique)
# =========================================================
ZETA = 1.0              # Coefficient d'amortissement (1.0 = critique, pas de dépassement)
OMEGA_N = 3.0           # Pulsation propre (vitesse de réaction du système, en rad/s)

# --- CALCUL DE K_sys ---
amplitude_rad = math.radians(AMPLITUDE_DEG)
omega_test = (2 * math.pi) / PERIODE_SEC
k_sys = (X_MAX_MM * (omega_test ** 2)) / amplitude_rad

print("--- RÉSULTATS DE L'IDENTIFICATION ---")
print(f"Constante du système (K_sys) : {k_sys:.2f} mm/(rad.s^2)")

# --- CALCUL DE KP et KD ---
# Formules issues de l'identification : s^2 + K_sys*Kd*s + K_sys*Kp = s^2 + 2*Zeta*Omega_n*s + Omega_n^2
kp = (OMEGA_N ** 2) / k_sys
kd = (2 * ZETA * OMEGA_N) / k_sys

print("\n--- GAINS À RENTRER DANS server.py ---")
print(f"KP = {kp:.4f}")
print(f"KD = {kd:.4f}")