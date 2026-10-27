# -*- coding: utf-8 -*-
import math
import numpy as np
from scipy.spatial.transform import Rotation as R

def angle_between_vectors(v1, v2):
    """计算两个向量之间的夹角，返回角度"""
    dot_product = np.dot(v1, v2)
    norm_v1 = np.linalg.norm(v1)
    norm_v2 = np.linalg.norm(v2)
    cos_theta = dot_product / (norm_v1 * norm_v2)
    angle = np.arccos(np.clip(cos_theta, -1.0, 1.0))  # 计算夹角
    return np.degrees(angle)  # 转换为角度



def quaternion_to_rotation_matrix(q):
    """
    四元数转旋转矩阵
    q: 四元数 [x, y, z, w]
    返回值: 3x3 旋转矩阵
    """
    rot_matrix = np.array(
        [[1.0 - 2 * (q[1] * q[1] + q[2] * q[2]), 2 * (q[0] * q[1] - q[3] * q[2]), 2 * (q[3] * q[1] + q[0] * q[2])],
         [2 * (q[0] * q[1] + q[3] * q[2]), 1.0 - 2 * (q[0] * q[0] + q[2] * q[2]), 2 * (q[1] * q[2] - q[3] * q[0])],
         [2 * (q[0] * q[2] - q[3] * q[1]), 2 * (q[1] * q[2] + q[3] * q[0]), 1.0 - 2 * (q[0] * q[0] + q[1] * q[1])]],
        dtype=np.float64)
    return rot_matrix

def rotation_matrix_to_quaternion(rotMat):
    """
    旋转矩阵 转 四元数 xyzw
    rotMat: 3x3 旋转矩阵
    返回值: 四元数 [x, y, z, w]
    """
    # r3 = R.from_matrix(rotMat)
    # qua = r3.as_quat()
    # return qua
    R = rotMat
    trace = np.trace(R)
    
    # 计算四元数的 w
    qw = 0.5 * np.sqrt(1 + trace)
    
    # 计算四元数的 x, y, z
    qx = (R[2, 1] - R[1, 2]) / (4 * qw)
    qy = (R[0, 2] - R[2, 0]) / (4 * qw)
    qz = (R[1, 0] - R[0, 1]) / (4 * qw)
    
    return np.array([qx, qy, qz, qw])  # [x, y, z, w] 格式


def eulerAnglesToRotationMatrix(theta):
    """
    欧拉角转旋转矩阵
    theta: [r, p, y]
    """
    R_x = np.array([[1,         0,                  0                   ],
                    [0,         math.cos(theta[0]), -math.sin(theta[0]) ],
                    [0,         math.sin(theta[0]), math.cos(theta[0])  ]
                    ])
                
    R_y = np.array([[math.cos(theta[1]),    0,      math.sin(theta[1])  ],
                    [0,                     1,      0                   ],
                    [-math.sin(theta[1]),   0,      math.cos(theta[1])  ]
                    ])
                
    R_z = np.array([[math.cos(theta[2]),    -math.sin(theta[2]),    0],
                    [math.sin(theta[2]),    math.cos(theta[2]),     0],
                    [0,                     0,                      1]
                    ])
                                
    R = np.dot(R_z, np.dot( R_y, R_x ))

    return R


def rotationMatrixToEulerAngles(RotM):
    """
    旋转矩阵转欧拉角
    RotateM: 旋转矩阵 ndarray shape=(3,3)
    """
    sciangle_0, sciangle_1, sciangle_2 = R.from_matrix(RotM).as_euler('xyz')

    return np.array([sciangle_0, sciangle_1, sciangle_2])

def getTransformMat(pos, rot):
    """
    将输入的pos和rot转换成 转换矩阵 格式

    input:
        pos: [ndarray, (3,), np.float]
            xyz坐标
        rot: [ndarray, (3,3), np.float]
            旋转矩阵

    return:
        T: [ndarray, (4,4), np.float]
            转换矩阵
    """
    T = np.zeros((4, 4), dtype=float)
    T[:3, :3] = rot
    T[:, 3] = np.r_[np.array(pos), np.array([1,])]
    return T

def getTransformMat1(offset, rotate):
    """
    将平移向量和旋转矩阵合并为变换矩阵
    offset: (x, y, z)
    rotate: 旋转矩阵 (9,)
    """
    mat = np.array([
        [rotate[0], rotate[1], rotate[2], offset[0]], 
        [rotate[3], rotate[4], rotate[5], offset[1]], 
        [rotate[6], rotate[7], rotate[8], offset[2]],
        [0, 0, 0, 1.] 
    ])
    return mat


def distancePt(pt1, pt2):
    """
    计算两点之间的欧氏距离
    pt: [row, col] 或 [x, y]
    return: float
    """
    return ((pt1[0] - pt2[0]) ** 2 + (pt1[1] - pt2[1]) ** 2) ** 0.5

def distancePt3d(pt1, pt2):
    """
    计算两点之间的欧氏距离
    pt: [x, y, z]
    return: float
    """
    return ((pt1[0] - pt2[0]) ** 2 + (pt1[1] - pt2[1]) ** 2 + (pt1[2] - pt2[2]) ** 2) ** 0.5


def calcAngleOfPts(pt1, pt2):
    """
    计算从pt1到pt2的逆时针夹角 [0, 2pi)
    
    pt: [x, y] 二维坐标系中的坐标，不是图像坐标系的坐标
    
    return: 弧度
    """
    dy = pt2[1] - pt1[1]
    dx = pt2[0] - pt1[0]
    return (math.atan2(dy, dx) + 2*math.pi) % (2*math.pi)
    

def radians_TO_angle(radians):
    """
    弧度转角度
    """
    return 180 * radians / math.pi

def angle_TO_radians(angle):
    """
    角度转弧度
    """
    return math.pi * angle / 180





if __name__ == "__main__":
    qua = np.array([0, 0, 0, 1])
    r = quaternion_to_rotation_matrix(qua)
    q = rotation_matrix_to_quaternion(r)
    print(q)
