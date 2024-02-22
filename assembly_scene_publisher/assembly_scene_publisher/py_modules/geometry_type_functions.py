import sympy as sp
from geometry_msgs.msg import Vector3, Quaternion
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
import numpy as np
from typing import Union
from scipy.spatial.transform import Rotation as R

SCALE_FACTOR = 100.0
def vector3_to_matrix1x4(vector:Vector3)->sp.Matrix:
    return sp.Matrix([[vector.x], [vector.y], [vector.z], [1.0]])

def vector3_to_matrix1x3(vector:Vector3)->sp.Matrix:
    return sp.Matrix([[vector.x], [vector.y], [vector.z]])

def point3D_to_vector3(point:Point)->Vector3:
    return Vector3(x=float(point.x), y=float(point.y), z=float(point.z))

def check_and_return_quaternion(object_to_check,logger=None):
    """This function checks if a given quaternion is valid. This is the case if x=0, y=0, z=0, w=0. 
    In this case the function will set the quaterion to x=0, y=0, z=0, w=1.
    This function accepts quaternions as well as poses as input. According to its input the equivaltent type is returned. """

    if isinstance(object_to_check,Quaternion):
        tmp_q = object_to_check
    elif isinstance(object_to_check,Pose):
        tmp_q = object_to_check.orientation

    # This case will probably never happen because the node would crash at some point
    else:
        if logger is not None:
            logger.error("Fatal error")
        return object_to_check
    
    if tmp_q.x==0 and tmp_q.y==0 and tmp_q.z==0 and tmp_q.w==0:
        tmp_q.x=0.0
        tmp_q.y=0.0
        tmp_q.z=0.0
        tmp_q.w=1.0
        if logger is not None:
            logger.info("Assuming Quaternion: x=0.0, y=0.0, z=0.0, w=1.0!")
    
    if isinstance(object_to_check,Quaternion):
        return tmp_q
    elif isinstance(object_to_check,Pose):
        object_to_check.orientation = tmp_q
        return object_to_check
    
def get_point_from_ros_obj(position: Union[Vector3, Point]) -> sp.Point3D:
    
    if isinstance(position, Vector3):
        position:Vector3
        #point = sp.Point3D(position.x, position.y, position.z)
        #point = sp.Point3D(position.x, position.y, position.z, evaluate=False)
        point = sp.Point3D(position.x*SCALE_FACTOR, position.y*SCALE_FACTOR, position.z*SCALE_FACTOR, evaluate=False)

    elif isinstance(position, Point):
        position:Point
        #point = sp.Point3D(position.x, position.y, position.z, evaluate=False)
        point = sp.Point3D(position.x*SCALE_FACTOR, position.y*SCALE_FACTOR, position.z*SCALE_FACTOR, evaluate=False)
        #point = sp.Point3D(position.x, position.y, position.z)

    else:
        raise ValueError("Invalid input type in method 'get_point_from_ros_obj'!")
    
    return point

def get_transform_matrix_from_tf(tf: Union[Pose, TransformStamped])-> sp.Matrix:
    if isinstance(tf, Pose):
        tf: Pose
        r:sp.Matrix = quaternion_to_rotation_matrix(tf.orientation)
        t = sp.Matrix([tf.position.x, tf.position.y, tf.position.z]) 
    elif isinstance(tf, TransformStamped):
        tf: TransformStamped
        r:sp.Matrix = quaternion_to_rotation_matrix(tf.transform.rotation)
        t = sp.Matrix([tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z])         
    else:
        return None
    transform_matrix = sp.eye(4)

    transform_matrix[:3, :3] = r
    transform_matrix[:3, 3] = t
    return transform_matrix

def get_rotation_matrix_from_tf(tf: Union[Pose, TransformStamped])-> sp.Matrix:
    if isinstance(tf, Pose):
        tf: Pose
        r:sp.Matrix = quaternion_to_rotation_matrix(tf.orientation)
    elif isinstance(tf, TransformStamped):
        tf: TransformStamped
        r:sp.Matrix = quaternion_to_rotation_matrix(tf.transform.rotation)
    else:
        return None
    roation_matrix = sp.eye(3)

    roation_matrix[:3, :3] = r
    return roation_matrix

def transform_matrix_to_pose(transform_matrix:sp.Matrix, logger = None)-> Pose:
    # Extract the rotation matrix and translation vector from the transformation matrix
    rotation_matrix = transform_matrix[:3, :3]
    translation_vector = transform_matrix[:3, 3]

    # Convert the rotation matrix to a quaternion
    quaternion = rotation_matrix_to_quaternion(rotation_matrix)

    if logger is not None:
        logger.debug(f"Quaternion: {quaternion}")

    # Create a Pose message
    pose_msg = Pose(
        position=Point(x=float(translation_vector[0]), y=float(translation_vector[1]), z=float(translation_vector[2])),
        orientation=Quaternion(x=float(quaternion[1]), y=float(quaternion[2]), z=float(quaternion[3]), w=float(quaternion[0])))
    return pose_msg 

def rotation_matrix_to_quaternion(Rot_mat)-> sp.Matrix:
    r = R.from_matrix(Rot_mat)
    return sp.Matrix(r.as_quat())

def quaternion_to_rotation_matrix(quaternion:Quaternion)-> sp.Matrix:
    w, x, y, z = quaternion.w, quaternion.x, quaternion.y, quaternion.z
    r = sp.Matrix([
        [1 - 2*y**2 - 2*z**2, 2*x*y - 2*w*z, 2*x*z + 2*w*y],
        [2*x*y + 2*w*z, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*w*x],
        [2*x*z - 2*w*y, 2*y*z + 2*w*x, 1 - 2*x**2 - 2*y**2]
    ])
    return r

def euler_to_quaternion(roll:float, pitch:float, yaw:float)-> Quaternion:
    """
    Convert Euler angles to quaternion.

    Parameters:
    - roll: Rotation angle around the x-axis (in radians)
    - pitch: Rotation angle around the y-axis (in radians)
    - yaw: Rotation angle around the z-axis (in radians)

    Returns:
    - Quaternion
    """
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    result = Quaternion()
    result.w = w
    result.x = x
    result.y = y
    result.z = z
    return result

def quaternion_to_euler(quaternion:Quaternion)-> tuple:
    """
    Convert quaternion to Euler angles.

    Parameters:
    - quaternion: Quaternion

    Returns:
    - tuple(roll, pitch, yaw)
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    t0 = 2 * (w * x + y * z)
    t1 = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(t0, t1)

    t2 = 2 * (w * y - z * x)
    t2 = 1 if t2 > 1 else t2
    t2 = -1 if t2 < -1 else t2
    pitch = np.arcsin(t2)

    t3 = 2 * (w * z + x * y)
    t4 = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(t3, t4)

    return (roll, pitch, yaw)

def euler_to_matrix(angles:list):
    Rz = sp.Matrix([[sp.cos(angles[2]), -1*sp.sin(angles[2]), 0],
                [sp.sin(angles[2]), sp.cos(angles[2]), 0],
                [0, 0, 1]])

    Ry = sp.Matrix([[sp.cos(angles[1]), 0, sp.sin(angles[1])],
                [0, 1, 0],
                [-1*sp.sin(angles[1]), 0, sp.cos(angles[1])]])

    Rx = sp.Matrix([[1, 0, 0],
                [0, sp.cos(angles[0]), -1*sp.sin(angles[0])],
                [0, sp.sin(angles[0]), sp.cos(angles[0])]])

    return Rz * Ry * Rx