# Before start 

q_init = get_actual_joint_positions()
q_init[5] = 3.1416
movej(q_init, a=0.5, v=0.5)
socket_open("169.254.123.188", 30000, "socket_pi")

# Robot program (Loop always)

donnees = socket_read_ascii_float(1, "socket_pi")
If donnees[0] == 1:
    q_cible = get_actual_joint_positions()
    q_cible[5] = donnees[1]
    servoj(q_cible, t=0.04, lookahead_time=0.1, gain=300)