import sympy as sp
from geometry_msgs.msg import Vector3, Quaternion
from geometry_msgs.msg import TransformStamped, Transform
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

def get_normal_vectors_from_quaternion(quaternion:Quaternion)->list[Vector3]:
    """
    Get the normal vectors of the coordinate system defined by the quaternion.
    The normal vectors are the columns of the rotation matrix defined by the quaternion.
    
    Args:
        quaternion: A geometry_msgs.msg.Quaternion object.
        
    Returns:
        A list containing the normal vectors of the coordinate system.
    """
    # Extract the rotation matrix from the quaternion
    rotation_matrix = quaternion_to_rotation_matrix(quaternion)
    
    # Extract the normal vectors from the rotation matrix
    normal_vectors = []
    for i in range(3):
        normal_vectors.append(Vector3(x=float(rotation_matrix[0, i]), y=float(rotation_matrix[1, i]), z=(float(rotation_matrix[2, i]))))
        
    return normal_vectors

def get_normal_vectors_from_rotation_matrix(rotation_matrix:sp.Matrix)->list[Vector3]:
    """
    Get the normal vectors of the coordinate system defined by the rotation matrix.
    The normal vectors are the columns of the rotation matrix.
    
    Args:
        rotation_matrix: A 3x3 sympy.Matrix representing the rotation matrix.
        
    Returns:
        A list containing the normal vectors of the coordinate system.
    """
    # Extract the normal vectors from the rotation matrix
    normal_vectors = []
    for i in range(3):
        normal_vectors.append(Vector3(x=float(rotation_matrix[0, i]), y=float(rotation_matrix[1, i]), z=float(rotation_matrix[2, i])))
        
    return normal_vectors

def get_euler_angles_from_roatation_matrix(rotation_matrix:sp.Matrix)->list[float]:
    """
    Get the Euler angles from the rotation matrix.
    
    Args:
        rotation_matrix: A 3x3 sympy.Matrix representing the rotation matrix.
        
    Returns:
        A list containing the Euler angles (roll, pitch, yaw).
    """
    # Extract the Euler angles from the rotation matrix
    roll = sp.atan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
    pitch = sp.atan2(-rotation_matrix[2, 0], sp.sqrt(rotation_matrix[2, 1]**2 + rotation_matrix[2, 2]**2))
    yaw = sp.atan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
    
    return [roll, pitch, yaw]


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

def get_transform_matrix_from_tf(tf: Union[Pose, TransformStamped, Transform])-> sp.Matrix:
    if isinstance(tf, Pose):
        tf: Pose
        r:sp.Matrix = quaternion_to_rotation_matrix(tf.orientation)
        t = sp.Matrix([tf.position.x, tf.position.y, tf.position.z]) 
    elif isinstance(tf, TransformStamped):
        tf: TransformStamped
        r:sp.Matrix = quaternion_to_rotation_matrix(tf.transform.rotation)
        t = sp.Matrix([tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z])   
    elif isinstance(tf, Transform):
        tf: Transform
        r:sp.Matrix = quaternion_to_rotation_matrix(tf.rotation)
        t = sp.Matrix([tf.translation.x, tf.translation.y, tf.translation.z])      
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
        orientation=Quaternion(x=float(quaternion.x), y=float(quaternion.y), z=float(quaternion.z), w=float(quaternion.w)))
    return pose_msg 

def transform_matrix_to_transform(transform_matrix:sp.Matrix, logger = None)-> Transform:
    # Extract the rotation matrix and translation vector from the transformation matrix
    rotation_matrix = transform_matrix[:3, :3]
    translation_vector = transform_matrix[:3, 3]

    # Convert the rotation matrix to a quaternion
    quaternion = rotation_matrix_to_quaternion(rotation_matrix)

    if logger is not None:
        logger.debug(f"Quaternion: {quaternion}")

    # Create a Transform message
    transform_msg = Transform(
        translation=Vector3(x=float(translation_vector[0]), y=float(translation_vector[1]), z=float(translation_vector[2])),
        rotation=Quaternion(x=float(quaternion.x), y=float(quaternion.y), z=float(quaternion.z), w=float(quaternion.w)))
    return transform_msg

# def rotation_matrix_to_quaternion(Rot_mat)-> sp.Matrix:
#     r = R.from_matrix(Rot_mat)
#     return sp.Matrix(r.as_quat())

# def rotation_matrix_to_quaternion_q(Rot_mat)-> Quaternion:
#     r = R.from_matrix(Rot_mat)
#     return Quaternion(x=r.as_quat()[1], y=r.as_quat()[2], z=r.as_quat()[3], w=r.as_quat()[0])

# def rotation_matrix_to_quaternion(rotation_matrix:sp.Matrix)-> Quaternion:
#     """
#     Convert a 3x3 rotation matrix to a quaternion.
    
#     Args:
#         rotation_matrix: A 3x3 sympy.Matrix representing the rotation matrix.
        
#     Returns:
#         A sympy.Matrix representing the quaternion (w, x, y, z).
#     """
#     # Extract elements of the rotation matrix
#     r = rotation_matrix
#     r00, r01, r02 = r[0, 0], r[0, 1], r[0, 2]
#     r10, r11, r12 = r[1, 0], r[1, 1], r[1, 2]
#     r20, r21, r22 = r[2, 0], r[2, 1], r[2, 2]
    
#     # Calculate the trace of the rotation matrix
#     trace = r00 + r11 + r22
    
#     if trace > 0:
#         s = sp.sqrt(trace + 1.0) * 2
#         w = 0.25 * s
#         x = (r21 - r12) / s
#         y = (r02 - r20) / s
#         z = (r10 - r01) / s
#     elif (r00 > r11) and (r00 > r22):
#         s = sp.sqrt(1.0 + r00 - r11 - r22) * 2
#         w = (r21 - r12) / s
#         x = 0.25 * s
#         y = (r01 + r10) / s
#         z = (r02 + r20) / s
#     elif r11 > r22:
#         s = sp.sqrt(1.0 + r11 - r00 - r22) * 2
#         w = (r02 - r20) / s
#         x = (r01 + r10) / s
#         y = 0.25 * s
#         z = (r12 + r21) / s
#     else:
#         s = sp.sqrt(1.0 + r22 - r00 - r11) * 2
#         w = (r10 - r01) / s
#         x = (r02 + r20) / s
#         y = (r12 + r21) / s
#         z = 0.25 * s
    
#     # Return the quaternion as a sympy Matrix
#     return Quaternion(x=float(x), y=float(y), z=float(z), w=float(w))

def rotation_matrix_to_quaternion(rotation_matrix: sp.Matrix) -> Quaternion:
    np_matrix = np.array(rotation_matrix.tolist()).astype(np.float64)
    r = R.from_matrix(np_matrix)
    x, y, z, w = r.as_quat()  # returns in [x, y, z, w] order
    return Quaternion(x=x, y=y, z=z, w=w)

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

def get_relative_transform_for_transforms(from_frame: TransformStamped,
                           to_frame: TransformStamped,
                           logger=None) -> Transform:
    """
    Compute the transform that brings coordinates from `from_frame` to `to_frame`.
    
    Args:
        from_frame: TransformStamped of the source frame.
        to_frame: TransformStamped of the target frame.
        logger: Optional logger for error reporting.
    
    Returns:
        Transform: The relative transform from `from_frame` to `to_frame`, or None on error.
    """
    # Convert to transformation matrices
    from_mat = get_transform_matrix_from_tf(from_frame)
    to_mat = get_transform_matrix_from_tf(to_frame)

    if from_mat is None or to_mat is None:
        if logger:
            logger.error("Invalid input in 'get_relative_transform'.")
        return None

    # Compute relative transform: to_mat⁻¹ * from_mat
    relative_mat = to_mat.inv() * from_mat
    return transform_matrix_to_transform(relative_mat)

def get_relative_transform_for_transforms_calibration(base_transform: TransformStamped,
                                                    additional_transform: TransformStamped,
                                                    logger=None) -> Transform:
    """
    Compute the transform that brings coordinates from `from_frame` to `to_frame`.
    
    Args:
        from_frame: TransformStamped of the source frame.
        to_frame: TransformStamped of the target frame.
        logger: Optional logger for error reporting.
    
    Returns:
        Transform: The relative transform from `from_frame` to `to_frame`, or None on error.
    """
    # Convert to transformation matrices
    base_mat = get_transform_matrix_from_tf(base_transform)
    additional_mat = get_transform_matrix_from_tf(additional_transform)

    if base_mat is None or additional_mat is None:
        if logger:
            logger.error("Invalid input in 'get_relative_transform'.")
        return None

    # Compute relative transform: to_mat⁻¹ * from_mat
    relative_mat = base_mat * additional_mat
    return transform_matrix_to_transform(relative_mat)
