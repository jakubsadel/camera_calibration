import numpy as np


def projection(filename):
    camera_matrix = np.loadtxt(filename,  delimiter=',')
    z = np.zeros((3,1))
    projection_matrix = np.concatenate((camera_matrix, z), axis=1)
    return projection_matrix

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


cf2fl_quaternion =  (0.0036421704571694136, 0.0016839834861457348, 0.003173565724864602,0.9999868869781494)
cf2fl_translation = (-0.0005275279399938881, 0.014794671908020973, 0.00016263406723737717)

fl2mf_quaternion =  (0.0, 0.0, 0.0, 1.0)
fl2mf_translation = (0.015, 0.0, -0.03)

osl2oss_quaternion =  (0, 0, 1, 0)
osl2oss_translation = (0.0, 0.0, 0.036180000000000004)

oss2lmb_quaternion =  (0, 0, 0, 1)
oss2lmb_translation = (0.0, 0.0, 0.03635)


lmb2mfb_quaternion =  (0, 0, 0, 1)
lmb2mfb_translation = (-0.615, 0.0, 0.27)

mfb2mf_quaternion =  (0, 0, 0, 1)
mfb2mf_translation = (0.0, 0.0, 0.015)

cf2fl_tm = transformation(cf2fl_quaternion, cf2fl_translation)

fl2mf_tm = transformation(fl2mf_quaternion, fl2mf_translation)

osl2oss_tm = transformation(osl2oss_quaternion, osl2oss_translation)

oss2lmb_tm = transformation(oss2lmb_quaternion, oss2lmb_translation)

lmb2mfb_tm = transformation(lmb2mfb_quaternion, lmb2mfb_translation)

mfb2mf_tm = transformation(mfb2mf_quaternion, mfb2mf_translation)



cf2mf_tm = cf2fl_tm @ fl2mf_tm
inv_cf2mf_tm = np.linalg.inv(cf2mf_tm)
lidar2camera_tm = inv_cf2mf_tm @ osl2oss_tm @ oss2lmb_tm @ lmb2mfb_tm @ mfb2mf_tm


print("Camera to base: \n", cf2mf_tm)

print("Base to camera: \n", inv_cf2mf_tm)

print("Lidar to camera: \n", lidar2camera_tm)

projection_matrix = projection("camera_matrix.txt")

print("Projection matrix: \n", projection_matrix)

xd = projection_matrix@ lidar2camera_tm


print("Final matrix: \n", xd)
print("Final matrix: \n", xd.shape)


point = np.array([22,33,44, 1]).T

projectedpoint = xd@point


print("Point: \n", projectedpoint/projectedpoint[2])

