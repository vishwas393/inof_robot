import math
import numpy as np

def R_to_wt(R):
    if ((R == np.identity(3)).any()):
        theta =0
        w = [0, 0, 0]
    elif (np.trace(R) == -1):
        theta = 180
        if (R[2][2] != -1):
            w = (1/(2*(1+R[2][2])))*([R[0][2], R[1][2], 1+R[2][2]])
        elif (R[1][1] != -1):
            w = (1/(2*(1+R[1][1])))*([R[0][1], 1+R[1][1], R[2][1]])
        elif (R[0][0] != -1):
            w = (1/(2*(1+R[0][0])))*([1+R[0][0], R[1][0], 1+R[2][0]])
    else:
        theta = math.acos(0.5*(np.trace(R)-1))
        theta = math.degrees(theta)
        w = (1/(2*math.sin(math.radians(theta))))*(R - np.transpose(R))
    
    return [theta, [w[2][1], w[0][2], w[1][0]]]

if __name__ == '__main__':
    R = np.array([[0.866, -0.25, 0.433],[0.25, 0.967, 0.058],[-0.433, 0.058, 0.899]])
    w = R_to_wt(R)
    print(w)



