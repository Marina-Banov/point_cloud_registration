import numpy as np
from geometry_msgs.msg import Vector3


def euler_from_quaternion(q):
    angles = Vector3()

    sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
    angles.x = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (q.w * q.y - q.z * q.x)
    angles.y = np.arcsin(sinp)

    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    angles.z = np.arctan2(siny_cosp, cosy_cosp)

    return angles

