import math

last_time = 0
curr_time= 0
RPM = 300
radius = 2
dia = 2*radius


def get_delta_theta(t):
    curr_time = t
    dt = curr_time - last_time
    dtheta = dt*RPM/60
    last_time = curr_time
    return dtheta


def find_Vb(t):
    A = [[-1/(2*dia), 1/(2*dia)],[0.5, 0.5],[0, 0]]
    B = [1, 1]
    C = [0, 0, 0]
    for x in range(len(A)):
        for y in range(len(x)):
            C[x] = A[x][y]*B[y]

    C = radius*get_delta_theta(t)*C
    return C


def find_delta_q(t, phi):
    delta_qb = [0, 0, 0]
    delta_q  = [0, 0, 0]
    mat = [[1, 0, 0],[0, cos(phi), -sin(phi)],[0, sin(phi), cos(phi)]]

    Vb = find_Vb(t)  #Vb = [w vx vy]
    

    if Vb[0] != 0:
        delta_qb[0] = Vb[0]
        delta_qb[1] = (Vb[1]*sin(Vb[0]) + Vb[2]*(cos(Vb[0]) - 1))/Vb[0]
        delta_qb[2] = (Vb[2]*sin(Vb[0]) + Vb[1]*(1 - cos(Vb[0])))/Vb[0]
    elif Vb[0] == 0:
        delta_qb[0] = 0
        delta_qb[1] = Vb[1]
        delta_qb[2] = Vb[2]

    for x in range(len(mat)):
        for y in range(len(x)):
            delta_q[x] = mat[x][y] * delta_qb[y]

    return delta_q


def get_distance_n_angle(G, C):
    tmp_theta = math.degrees(math.atan2(G[1]-C[1],G[0]-C[0]))
     
    distance = math.sqrt( math.pow(G[0]-C[0], 2) + math.pow(G[1]-C[1], 2) )
    distance = distance if (tmp_theta >= C[2]-90 and tmp_theta < C[2]+90) else (-distance)

    tmp_theta = tmp_theta if (tmp_theta >= C[2]-90 and tmp_theta < C[2]+90) else (tmp_theta+180)   
    theta = tmp_theta - C[2]
    
    return [distance, theta]

































