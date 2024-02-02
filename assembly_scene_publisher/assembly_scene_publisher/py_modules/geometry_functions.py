import sympy as sp
from geometry_msgs.msg import Vector3, Quaternion
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
import numpy as np
from typing import Union


def get_point_of_plane_intersection(plane1: sp.Plane, plane2: sp.Plane, plane3: sp.Plane) -> sp.Point3D:
    line = plane1.intersection(plane2)
    # Get the first point of intersection, should also be the only one
    #inter:sp.Point3D = plane3.intersection(line[0])

    try:
        plane3.intersection(line[0])[0]
    except Exception as e:
        raise ValueError(f"Given planes (1.{plane1}, 2.{plane2}, 3.{plane3}) do not have a single point of intersection. Invalid plane selection!")

    inter:sp.Point3D = plane3.intersection(line[0])[0]

    if not isinstance(inter, sp.Point3D):
        raise ValueError(f"Given planes (1.{plane1}, 2.{plane2}, 3.{plane3}) do not have a single point of intersection. Invalid plane selection!")
    
    # Value Error if not a point

    return inter


def get_euler_rotation_matrix(alpha, beta, gamma):
    rotation_z = sp.Matrix([
        [sp.cos(alpha), -sp.sin(alpha), 0],
        [sp.sin(alpha), sp.cos(alpha), 0],
        [0, 0, 1]
    ])

    rotation_y = sp.Matrix([
        [sp.cos(beta), 0, sp.sin(beta)],
        [0, 1, 0],
        [-sp.sin(beta), 0, sp.cos(beta)]
    ])

    rotation_x = sp.Matrix([
        [1, 0, 0],
        [0, sp.cos(gamma), -sp.sin(gamma)],
        [0, sp.sin(gamma), sp.cos(gamma)]
    ])

    rotation_matrix = rotation_z * rotation_y * rotation_x
    return rotation_matrix

def quaternion_multiply(q0:Quaternion, q1:Quaternion)->Quaternion:
    """
    Multiplies two quaternions.

    Input
    :param q0: 
    :param q1: 

    Output
    :return: Quaternion

    """

    #q0.w = -q0.w

    # Extract the values from q0
    w0 = q0.w
    x0 = q0.x
    y0 = q0.y
    z0 = q0.z

    # Extract the values from q1
    w1 = q1.w
    x1 = q1.x
    y1 = q1.y
    z1 = q1.z

    # Computer the product of the two quaternions, term by term
    q0q1_w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1
    q0q1_x = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1
    q0q1_y = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1
    q0q1_z = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1

    result = Quaternion()
    result.x = q0q1_x
    result.y = q0q1_y
    result.z = q0q1_z
    result.w = q0q1_w

    return result


def matrix_multiply_vector(matrix:sp.Matrix, vector:sp.Matrix)->Vector3:
    result = matrix * vector
    return Vector3(x=float(result[0]), y=float(result[1]), z=float(result[2]))

def norm_vec_direction(norm_vector_1: sp.Matrix, norm_vector_2: Union[sp.Matrix,Vector3], logger = None) -> int:
    """
    Checks if the direction of the two vectors is the same or not.
    Parameters:
    - norm_vector_1: sp.Matrix
    - norm_vector_2: sp.Matrix or Vector3
    Returns:
    - 1 if direction is the same
    - -1 if direction is not the same
    """
    if isinstance(norm_vector_2, Vector3):
        norm_vector_2 = sp.Matrix([norm_vector_2.x, norm_vector_2.y, norm_vector_2.z])

    angle_rad = calc_angle_between_vectors(norm_vector_1, norm_vector_2)

    if logger is not None:
        logger.error(f"Angle between the two vectors [rad]: {angle_rad}")

    if angle_rad > sp.pi/2 or angle_rad < -sp.pi/2:
        return -1
    else:
        return 1
    

def calc_angle_between_vectors(vector_1: sp.Matrix, vector_2: sp.Matrix) -> float:
    """
    Calculates the angle between two vectors in radians.
    Parameters:
    - vector_1: sp.Matrix
    - vector_2: sp.Matrix
    Returns:
    - angle_radians: float
    """
    dot_product = sp.DotProduct(vector_1, vector_2).doit()
    #magnitude_vec_1 = sp.sqrt(sp.DotProduct(vector_1, vector_1)).doit()
    #magnitude_vec_2 = sp.sqrt(sp.DotProduct(vector_2, vector_2)).doit()
    magnitude_vec_1 = vector_1.norm()
    magnitude_vec_2 = vector_2.norm()
    # Calculate the cosine of the angle
    cosine_theta = dot_product / (magnitude_vec_1 * magnitude_vec_2)
    
    # if not -1 <= cosine_theta <= 1:
    #     #return float(np.pi)  # Return a default value (e.g., pi) or handle as needed
    #     return 0.0

    # Calculate the angle in radians
    angle_radians = sp.re (sp.acos(cosine_theta))

    return float(angle_radians.evalf())