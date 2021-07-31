import cv2
import numpy as np
import math

fr = 15;

def obj_robot_collision_border(arr):
    shape = arr.shape
    print("obj_robot_collision_border : " + str(shape))
    for i in range(shape[0]-1):
        for j in range(shape[1]):
            if (arr[i][j] - arr[i+1][j]) == 1:
                for k in range(15):
                    if i-k >= 0:
                        arr[i-k][j] = 0
    
    for i in range(shape[0]-1):
        for j in range(shape[1]):
            if (arr[shape[0]-1 -i][j] - arr[shape[0]-1-i-1][j]) == 1:
                for k in range(15):
                    if shape[0]-1-i+k < shape[0]:
                        arr[shape[0]-1-(i-k)][j] = 0

    
    for j in range(shape[1]-1):
        for i in range(shape[0]):
            if(arr[i][j] - arr[i][j+1]) == 1:
                for k in range(15):
                    if j-k > 0:
                        arr[i][j-k] = 0


    for j in range(shape[1]-1):
        for i in range(shape[0]):
            if(arr[i][shape[1]-1-j-1] - arr[i][shape[1]-1-j]) == -1:
                for k in range(15):
                    if shape[1]-1-j+k < shape[1]:
                        arr[i][shape[1]-1-j+k] = 0


    return arr


def array_shrinking(arr):
    shape = arr.shape
    print("array_shrinking : Recved size: " + str(shape))
    subarr = np.zeros([int(shape[0]/fr), int(shape[1]/fr)], dtype=int)
    for i in range(int(shape[0]/fr)):
        for j in range(int(shape[1]/fr)):
            tmp = arr[fr*i : fr*(i+1) , fr*j : fr*(j+1) ]
            #subarr[i,j] = sum(sum(tmp))
            subarr[i,j] = 1 if sum(sum(tmp)) == fr*fr else 0

    return subarr
    


def convert_img_to_array():
   
    f = open("/home/vishwas/Desktop/Office_Map/office_map_5cm.txt", 'w')


    image = cv2.imread("/home/vishwas/Desktop/Office_Map/combined_office_map.png", 0)
    image = image / 255

    A = image.shape
    print("Original size" + str(A))

    print(math.ceil(A[0]/fr))
    print(int(A[0]/fr))


    f.write("{{")
    for i in range(A[0]):
        for j in range(A[1]):
            f.write("%d, " % image[i][j])
        f.write("}, \n {")

    f.write("} \n\n\n\n")
    

    dflt_img = obj_robot_collision_border(image)


    if (math.ceil(A[0]/fr) != int(A[0]/fr)):
        row = ( A[0] + ( fr - (A[0]%fr) ) ) 
    else:
        row = A[0]



    if (math.ceil(A[1]/fr) != int(A[1]/fr)):
        col = ( A[1] + ( fr - (A[1]%fr) ) ) 
    else:
        col = A[1]
    


    grew_img = np.zeros([row, col], dtype=int)

    for i in range(A[0]):
        for j in range(A[1]):
            grew_img[i, j] = dflt_img[i][j]


    img = array_shrinking(grew_img)

    A = img.shape
    print("Modified size" + str(A))
    

    f.write("{{")
    for i in range(A[0]):
        for j in range(A[1]):
            f.write("%d, " % img[i][j])
        f.write("}, \n {")

    f.write("}")

if __name__ == "__main__":
    convert_img_to_array()
