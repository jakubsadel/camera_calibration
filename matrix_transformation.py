import numpy as np
from math import cos, sin, radians



def transformation(quaternion, translation):

    # Extract the values from quaternion
    q0 = quaternion[0]
    q1 = quaternion[1]
    q2 = quaternion[2]
    q3 = quaternion[3]
     
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
     
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
    # 3x3 rotation matrix
    # rotation_matrix = np.array([[r00, r01, r02],
    #                        [r10, r11, r12],
    #                        [r20, r21, r22]])


    dX = translation[0]
    dY = translation[1]
    dZ = translation[2]

    # translation_matrix = np.array([[1, 0, 0, dX],
    #                            [0, 1, 0, dY],
    #                            [0, 0, 1, dZ],
    #                            [0, 0, 0, 1]])   
                            
    transfromation_matrix = np.array([[r00, r01, r02, dX],
                           [r10, r11, r12, dY],
                           [r20, r21, r22, dZ],
                           [0, 0, 0, 1]])
   

    return transfromation_matrix

