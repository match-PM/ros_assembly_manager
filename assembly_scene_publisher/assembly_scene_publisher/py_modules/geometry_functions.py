from copy import deepcopy
import sympy as sp
from geometry_msgs.msg import Vector3, Quaternion
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
import numpy as np
from typing import Union
from scipy.spatial.transform import Rotation
from assembly_scene_publisher.py_modules.geometry_type_functions import rotation_matrix_to_quaternion, quaternion_to_rotation_matrix
from rclpy.impl.rcutils_logger import RcutilsLogger 
import math

from scipy.spatial.transform import Rotation as R

class NumPlane():
    """
    A class to represent a plane in 3D space using the equation Ax + By + Cz + D = 0.
    """
    def __init__(self, A:float, B:float, C:float, D:float):
        self.A = A
        self.B = B
        self.C = C
        self.D = D

def num_plane_from_points(point_1: Point,
                          point_2: Point,
                          point_3: Point) -> NumPlane:
    """
    Computes the coefficients of a plane defined by three points in 3D space.

    Args:
        points (list of Point): List of ROS2 Point objects containing x, y, z coordinates.
    Returns:
        NumPlane: Coefficients of the plane in the form Ax + By + Cz + D = 0.
    """
    # Extract positions as numpy arrays
    points = np.array([[point_1.x, point_1.y, point_1.z],
                       [point_2.x, point_2.y, point_2.z],
                       [point_3.x, point_3.y, point_3.z]])
    
    # Create vectors from the first point to the other two points
    v1, v2 = points[1] - points[0], points[2] - points[0]
    
    # Compute the normal vector using the cross product
    normal_vector = np.cross(v1, v2)
    
    # Coefficients A, B, C
    A, B, C = normal_vector
    
    # Compute D using one of the points
    D = -np.dot(normal_vector, points[0])

    return NumPlane(A, B, C, D)

def num_plane_from_axis_and_point(axis_point_1: Point, 
                                  axis_point_2: Point,
                                  point_on_plane: Point) -> NumPlane:
    """
    Computes the coefficients of a plane given a perpendicular axis (normal vector) 
    defined by two points and a point on the plane.

    Args:
        axis_point_1 (Point): First point defining the axis (normal vector).
        axis_point_2 (Point): Second point defining the axis.
        point_on_plane (Point): Point lying on the plane.

    Returns:
        NumPlane: Coefficients of the plane in the form Ax + By + Cz + D = 0.
    """
    # Convert inputs to numpy arrays
    normal_vector = np.array([
        axis_point_2.x - axis_point_1.x,
        axis_point_2.y - axis_point_1.y,
        axis_point_2.z - axis_point_1.z
    ])
    point = np.array([point_on_plane.x, point_on_plane.y, point_on_plane.z])

    A, B, C = normal_vector
    D = -np.dot(normal_vector, point)  # correct D for Ax + By + Cz + D = 0

    return NumPlane(A, B, C, D)


def get_point_of_plane_intersection_num(plane1:NumPlane, plane2:NumPlane, plane3:NumPlane) -> Vector3:
    A = np.array([[plane1.A, plane1.B, plane1.C],
                  [plane2.A, plane2.B, plane2.C],
                  [plane3.A, plane3.B, plane3.C]])
    b = np.array([-plane1.D, -plane2.D, -plane3.D])
    # Solve the system of equations
    try:
        point = np.linalg.solve(A, b)
    except np.linalg.LinAlgError:
        raise ValueError("The planes do not intersect at a single point.")
    # Return the point as a sympy Point3D object
    return Vector3(x=point[0], y=point[1], z=point[2])

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


def compute_eigenvectors_and_centroid_old(poses: list[Pose], 
                                      logger: RcutilsLogger = None) -> tuple:
    """
    Computes the eigenvectors of the given poses and the centroid of the positions.

    Args:
        poses (list of Pose): List of ROS2 Pose objects containing position and orientation.

    Returns:
        tuple: (quaternion, centroid)
    """
    positions = []
    rotation_matrices = []
    
    if len(poses) < 2:
        raise ValueError("At least 2 poses are required to compute eigenvectors and centroid.")
    
    # Extract positions and rotation matrices
    for pose in poses:
        positions.append([pose.position.x, pose.position.y, pose.position.z])
        quaternion = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        rotation_matrices.append(Rotation.from_quat(quaternion).as_matrix())
    
    positions = np.array(positions)
    
    centroid = np.mean(positions, axis=0)  # Compute centroid
    
    positions_centered = positions - centroid  # Shift points to mean
    
    centroid_vector = Vector3()
    centroid_vector.x = centroid[0]
    centroid_vector.y = centroid[1]
    centroid_vector.z = centroid[2]
    
    # Compute covariance matrix from positions
    covariance_matrix = np.cov(positions_centered.T)
    
    # Compute eigenvalues and eigenvectors
    eigenvalues, eigenvectors = np.linalg.eig(covariance_matrix)
    
     # Sort eigenvectors by ascending eigenvalues (smallest eigenvalue first)
    sorted_indices = np.argsort(eigenvalues)  # Smallest eigenvalue first
    eigenvectors = eigenvectors[:, sorted_indices]
    eigenvalues = eigenvalues[sorted_indices]

    # Ensure the smallest eigenvector (now in first column) is the Z-axis
    rotation_matrix = np.zeros((3, 3))
    rotation_matrix[:, 2] = eigenvectors[:, 0]  # Smallest eigenvector → Z-axis

    # Use the other two eigenvectors for X and Y axes
    rotation_matrix[:, 0] = eigenvectors[:, 1]  # Second smallest eigenvector → X-axis
    rotation_matrix[:, 1] = eigenvectors[:, 2]  # Largest eigenvector → Y-axis

    # Ensure a right-handed coordinate system
    if np.linalg.det(rotation_matrix) < 0:
        rotation_matrix[:, 1] *= -1  # Flip the Y-axis to maintain right-handedness
    
    quat = rotation_matrix_to_quaternion(rotation_matrix)
    #get rotation_matrix as quaternion 
    
    return quat, centroid_vector

# def compute_eigenvectors_and_centroid(poses: list[Pose], 
#                                       logger: RcutilsLogger = None) -> tuple:
#     """
#     Computes the eigenvectors of the given poses and the centroid of the positions.

#     Args:
#         poses (list of Pose): List of ROS2 Pose objects containing position and orientation.

#     Returns:
#         tuple: (quaternion, centroid)
#     """
#     positions = []
#     rotation_matrices = []
    
#     if len(poses) < 2:
#         raise ValueError("At least 2 poses are required to compute eigenvectors and centroid.")
    
#     # Extract positions and rotation matrices
#     for pose in poses:
#         positions.append([pose.position.x, pose.position.y, pose.position.z])
#         quaternion = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
#         rotation_matrices.append(Rotation.from_quat(quaternion).as_matrix())
    
#     positions = np.array(positions)
    
#     centroid = np.mean(positions, axis=0)  # Compute centroid
    
#     positions_centered = positions - centroid  # Shift points to mean
    
#     centroid_vector = Vector3()
#     centroid_vector.x = centroid[0]
#     centroid_vector.y = centroid[1]
#     centroid_vector.z = centroid[2]
    
#     # Compute covariance matrix from positions
#     covariance_matrix = np.cov(positions_centered.T)
    
#     # Compute eigenvalues and eigenvectors
#     eigenvalues, eigenvectors = np.linalg.eig(covariance_matrix)
    
#      # Sort eigenvectors by ascending eigenvalues (smallest eigenvalue first)
#     sorted_indices = np.argsort(eigenvalues)  # Smallest eigenvalue first
#     eigenvectors = eigenvectors[:, sorted_indices]
#     eigenvalues = eigenvalues[sorted_indices]

#     biggest_vector = eigenvectors[:, 2]
#     medium_vector = eigenvectors[:, 1]
#     smallest_vector = eigenvectors[:, 0]

#     if biggest_vector[0]<0:
#         biggest_vector = -biggest_vector

#     if smallest_vector[2]<0:
#         smallest_vector = -smallest_vector
    
#     # Ensure the smallest eigenvector (now in first column) is the Z-axis
#     rotation_matrix = np.zeros((3, 3))
#     rotation_matrix[:, 2] = smallest_vector  # Smallest eigenvector → Z-axis

#     # Use the other two eigenvectors for X and Y axes
#     rotation_matrix[:, 0] = biggest_vector  # Largest eigenvector → X-axis
#     rotation_matrix[:, 1] = medium_vector  # Medium eigenvector → Y-axis

#     logger.warn(f"X-axis: {biggest_vector}")
#     logger.warn(f"Y-axis: {medium_vector}")
#     logger.warn(f"Z-axis: {smallest_vector}")

#     # Ensure a right-handed coordinate system
#     if np.linalg.det(rotation_matrix) < 0:
#         rotation_matrix[:, 0] *= -1  # Flip the X-axis to maintain right-handedness
    
#     quat = rotation_matrix_to_quaternion(rotation_matrix)
#     #get rotation_matrix as quaternion 
    
#     return quat, centroid_vector


def compute_eigenvectors_and_centroid(poses: list[Pose], logger=None) -> tuple[Quaternion, Vector3]:
    """
    Computes the orientation (as a quaternion) and the centroid (as a Vector3) from a list of poses,
    ensuring Z-axis points upwards and X-axis points forward.

    Args:
        poses (list of Pose): List of ROS2 Pose objects.
        logger (optional): ROS2 logger.

    Returns:
        tuple:
            - Quaternion: Orientation based on eigenvectors.
            - Vector3: Centroid of the pose positions.
    """
    if len(poses) < 2:
        raise ValueError("At least 2 poses are required.")

    # Step 1: Extract positions
    positions = np.array([[p.position.x, p.position.y, p.position.z] for p in poses])
    centroid = np.mean(positions, axis=0)
    positions_centered = positions - centroid

    # Step 2: Prepare Vector3 centroid
    centroid_vector = Vector3()
    centroid_vector.x, centroid_vector.y, centroid_vector.z = centroid

    # Step 3: PCA - compute covariance and eigenvectors
    cov_matrix = np.cov(positions_centered.T)
    eigenvalues, eigenvectors = np.linalg.eig(cov_matrix)

    # Sort by ascending eigenvalue
    sorted_indices = np.argsort(eigenvalues)
    eigenvectors = eigenvectors[:, sorted_indices]

    # Step 4: Construct axes
    smallest_vector = eigenvectors[:, 0]  # Z-axis
    biggest_vector = eigenvectors[:, 2]   # X candidate

    # Normalize Z and ensure it points up
    z_axis = smallest_vector / np.linalg.norm(smallest_vector)
    if z_axis[2] < 0:
        z_axis = -z_axis

    # Orthogonalize X-axis to Z
    x_candidate = biggest_vector / np.linalg.norm(biggest_vector)
    x_axis = x_candidate - np.dot(x_candidate, z_axis) * z_axis
    x_axis = x_axis / np.linalg.norm(x_axis)

    # Ensure X points forward
    if x_axis[0] < 0:
        x_axis = -x_axis

    # Compute Y-axis via cross product
    y_axis = np.cross(z_axis, x_axis)

    # Build rotation matrix
    rotation_matrix = np.column_stack((x_axis, y_axis, z_axis))

    # Optional: check determinant
    # if logger:
    #     logger.warning(f"X-axis: {x_axis}")
    #     logger.warning(f"Y-axis: {y_axis}")
    #     logger.warning(f"Z-axis: {z_axis}")
    #     logger.warning(f"Rotation matrix determinant: {np.linalg.det(rotation_matrix):.6f}")

    # Convert to quaternion
    quat_array = Rotation.from_matrix(rotation_matrix).as_quat()
    quat_msg = Quaternion()
    quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w = quat_array

    return quat_msg, centroid_vector


def get_transformed_pose(initial_pose:Pose, offset:Vector3)->Pose:
    """
    Computes a new Pose given an initial Pose and an (x, y, z) offset in the frame's local direction.

    Args:
        initial_pose (Pose): The original pose (position + orientation).
        offset (Vector3): (x, y, z) offset in the local frame of initial_pose.

    Returns:
        Pose: The new pose with the offset applied in the correct direction.
    """
    # Extract the initial position
    pos = np.array([initial_pose.position.x, initial_pose.position.y, initial_pose.position.z])

    rotation_matrix = quaternion_to_rotation_matrix(initial_pose.orientation)
    
    offset_tuple = (offset.x, offset.y, offset.z)
    # Transform the offset from local to global frame
    global_offset = rotation_matrix @ np.array(offset_tuple)

    # Compute new position
    new_pos = pos + global_offset

    # Create a new Pose with the new position (keeping the same orientation)
    new_pose = Pose()
    new_pose.position.x = float(new_pos[0])
    new_pose.position.y = float(new_pos[1])
    new_pose.position.z = float(new_pos[2])
    new_pose.orientation = initial_pose.orientation  # Keep the same orientation

    return new_pose


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

def norm_vec_direction(norm_vector_1: sp.Matrix, 
                       norm_vector_2: Union[sp.Matrix,Vector3], 
                       logger:RcutilsLogger = None) -> int:
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
        logger.debug(f"Debug-Info: Angle between the two vectors [rad]: {angle_rad}")

    if angle_rad > sp.pi/2 or angle_rad < -sp.pi/2:
        logger.debug(f"Return -1")
        return -1
    else:
        logger.debug(f"Return 1")
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

def normalize_vector3(vector: Vector3) -> Vector3:
    """Normalizes a Vector3 object."""
    magnitude = math.sqrt(vector.x ** 2 + vector.y ** 2 + vector.z ** 2)
    
    if magnitude == 0:
        raise ValueError("Cannot normalize a zero vector")
    
    normalized_vector = Vector3()
    normalized_vector.x = vector.x / magnitude
    normalized_vector.y = vector.y / magnitude
    normalized_vector.z = vector.z / magnitude

    return normalized_vector



def points_valid_for_plane(frames: list[Pose]) -> bool:

    if len(frames) < 3:
        raise ValueError("At least 3 points are required to define a plane.")

    points = [(pose.position.x, pose.position.y, pose.position.z) for pose in frames]

    # Create vectors from the first point to all other points
    p1 = np.array(points[0])
    vectors = [np.array(p) - p1 for p in points[1:]]

    # Compute the cross product of each pair of vectors
    for i in range(len(vectors) - 1):
        for j in range(i + 1, len(vectors)):
            cross_product = np.cross(vectors[i], vectors[j])
            if np.linalg.norm(cross_product) > 0.0001:
                return True
            else:
                raise ValueError("Points are collinear. Cannot define a plane.")
    return False

def project_pose_on_plane(frame: Pose, 
                        plane: sp.Plane, 
                        logger: RcutilsLogger = None) -> Pose:
    """
    Projects a 3D point onto a given plane.

    :param frame: A Pose object from geometry_msgs.msg.Pose with position (x, y, z).
    :param plane: A sympy Plane object.
    :return: A new Pose object with projected coordinates.
    """
    # make copy
    frame = deepcopy(frame)
    
    # Extract normal vector from the plane
    normal_vector = np.array([float(coord.evalf()) for coord in plane.normal_vector])

    # Extract original position from ROS2 Pose message
    original_position = np.array([frame.position.x, frame.position.y, frame.position.z])

    # Get a known point on the plane
    point_on_plane = np.array([float(coord.evalf()) for coord in plane.p1])

    # Compute the projection scalar t
    vector_to_plane = original_position - point_on_plane
    t = np.dot(vector_to_plane, normal_vector) / np.dot(normal_vector, normal_vector)

    # Compute the new projected position
    new_position = original_position - t * normal_vector

    # Create a new Pose object for the result
    projected_pose = Pose()
    projected_pose.position.x = new_position[0]
    projected_pose.position.y = new_position[1]
    projected_pose.position.z = new_position[2]

    # Keep the original orientation unchanged
    projected_pose.orientation = frame.orientation
    
    if logger is not None:
        if abs(t)>0.000001:
            logger.error(f"DEBUG: POINT MOVED: Distance point moved: {(t):.6f}")

    return projected_pose

def create_3D_plane(frames: list[Pose])-> tuple:
    """
    Computes the best-fit plane for a set of 3D points using least squares.
    """

    # Check if points are collinear
    points_positions = [(pose.position.x, pose.position.y, pose.position.z) for pose in frames]

    if not points_valid_for_plane(frames):
        print("Points are collinear. Cannot define a plane.")
        return None, None, None
    
    # Step 1: Compute center of mass
    center_coordinates = [sum(coords) / len(points_positions) for coords in zip(*points_positions)]
    center_of_mass = sp.Point3D(*center_coordinates)

    # Step 2: Convert points into a numpy array and center them
    A = np.array(points_positions) - np.array(center_coordinates)

    # Step 3: Use least squares to fit a plane
    # Set up the system Ax = b, where A contains x, y, z coordinates and b is a constant
    X = A[:, 0]
    Y = A[:, 1]
    Z = A[:, 2]
    A_matrix = np.vstack([X, Y, np.ones(len(X))]).T

    # Solve for the best-fit plane coefficients using least squares
    C, residuals, rank, s = np.linalg.lstsq(A_matrix, Z, rcond=None)
    A, B, D = C[0], C[1], 1
    normal_vector = np.array([A, B, 1]) #remember 1 was negative, changes direction of normal vector relativ to z axis
    normal_vector = normal_vector / np.linalg.norm(normal_vector)  # Normalize the normal vector

    print(f"Normal vector: {normal_vector}")

    # The plane equation is Ax + By + Cz + D = 0
    ref_plane = sp.Plane(sp.Point3D(*center_coordinates), normal_vector.tolist())

    return ref_plane, center_of_mass, normal_vector.tolist()


def are_points_collinear(points: list[Pose]) -> bool:
    """
    Checks if a set of 3D points are collinear by computing the rank of their covariance matrix.
    """
    if len(points) < 3:
        return True  # Less than 3 points are always collinear

    # Extract positions
    points_array = np.array([(pose.position.x, pose.position.y, pose.position.z) for pose in points])

    # Compute centered positions
    centered = points_array - np.mean(points_array, axis=0)

    # Compute SVD
    _, singular_values, _ = np.linalg.svd(centered)

    # If two singular values are near zero, points are collinear
    return np.isclose(singular_values[1], 0) and np.isclose(singular_values[2], 0)


def create_3D_plane_2(frames: list[Pose], 
                        plane_offset: float = 0.0,
                        logger: RcutilsLogger = None) -> sp.Plane:
    """
    Computes the best-fit plane for a set of 3D points using least squares and applies an optional offset.
    
    :param frames: List of Pose objects (ROS2 Pose messages).
    :param plane_offset: Distance to shift the plane along its normal vector.
    :param logger: ROS2 logger for debugging.
    :return: (plane)
    """
    if len(frames) < 3:
        print("Not enough points to define a plane.")
        return None, None, None

    # Check collinearity
    if are_points_collinear(frames):
        print("Points are collinear. Cannot define a plane.")
        return None, None, None

    # Step 1: Extract points and compute centroid
    points_positions = np.array([(pose.position.x, pose.position.y, pose.position.z) for pose in frames])
    center_coordinates = np.mean(points_positions, axis=0)
    center_of_mass = sp.Point3D(*center_coordinates)

    # Step 2: Perform Singular Value Decomposition (SVD) to find the normal vector
    centered_points = points_positions - center_coordinates
    _, _, vh = np.linalg.svd(centered_points)

    # The normal vector to the best-fit plane is the last row of V^T
    normal_vector = vh[-1]

    # Normalize the normal vector
    normal_vector /= np.linalg.norm(normal_vector)

    print(f"Normal Vector: {normal_vector}")
    if normal_vector[2]<0:
        norm_vector_dir = -normal_vector
    else:
        norm_vector_dir = normal_vector

    # Step 3: Apply the plane offset
    offset_vector = norm_vector_dir * plane_offset
    offset_center_coordinates = center_coordinates + offset_vector
    offset_center_of_mass = sp.Point3D(*offset_center_coordinates)

    # Step 4: Define the plane using SymPy
    ref_plane = sp.Plane(offset_center_of_mass, normal_vector.tolist())

    return ref_plane

def rotate_point(frame:Pose, plane, target_axis, logger: RcutilsLogger = None) -> Pose:
    """
    Rotates a given point so that its target axis aligns with the normal of a plane.
    
    :param frame: Pose object containing position and orientation.
    :param plane: Plane object with a normal vector.
    :param target_axis: Axis to align ("X", "Y", or "Z").
    :param logger: ROS2 logger for debugging.
    :return: Updated Pose object with new orientation.
    """
    
    frame = deepcopy(frame)
    
    # Convert plane normal vector to float (ensuring no symbolic values)
    normal_vector = np.array([float(sp.N(coord)) for coord in plane.normal_vector], dtype=float)

    if logger is not None:
        logger.warn(f"Normal Vector: {normal_vector}")

    # Set direction of normal vector to point upwards. 
    if normal_vector[2] < 0:
        normal_vector = -normal_vector
    
    # calculate the angle between all of the coordianate axis of the frame and the normal vector
    angles = angles_to_normal_vector(normal_vector, frame.orientation)

    if logger is not None:
        logger.warn(f"Angles to Normal Vector: {angles}")

    target_axis = target_axis.upper()
    if target_axis not in ["X", "Y", "Z"]:
        raise ValueError("Invalid axis. Choose 'X', 'Y', or 'Z'.")

    # Select the rotation angle based on the target axis
    angle_rad = angles[{"X": 0, "Y": 1, "Z": 2}[target_axis]]

    if np.isclose(angle_rad, 0, atol=1e-6):
        return frame  # No need to rotate

    axis_index = {"X": 0, "Y": 1, "Z": 2}[target_axis]

    # Extract and ensure quaternion is numerical
    quaternion = np.array([float(frame.orientation.x), float(frame.orientation.y), 
                           float(frame.orientation.z), float(frame.orientation.w)], dtype=float)

    rotation_matrix = R.from_quat(quaternion).as_matrix()

    # Get the local axis corresponding to the target axis
    local_axis = rotation_matrix[:, axis_index]

    # Compute rotation axis, this vector is orthogonal to both local_axis and normal_vector
    rotation_axis = np.cross(local_axis, normal_vector)

    # Handle degenerate cases where local_axis is parallel to normal_vector
    if np.allclose(rotation_axis, 0, atol=1e-6):
        logger.warning("Local axis is already aligned with the normal vector. No rotation needed.")
        return frame  # No rotation needed

    # Normalize rotation axis
    rotation_axis /= np.linalg.norm(rotation_axis)

    # Create rotation quaternion - create a rotation matrix
    rotation_quaternion = R.from_rotvec(angle_rad * rotation_axis).as_quat()

    # Normalize quaternion
    rotation_quaternion /= np.linalg.norm(rotation_quaternion)

    # Apply rotation
    new_quaternion = R.from_quat(rotation_quaternion) * R.from_quat(quaternion)
    new_quat_values = new_quaternion.as_quat()

    # Update frame orientation
    frame.orientation.x, frame.orientation.y, frame.orientation.z, frame.orientation.w = new_quat_values

    return frame

def signed_angle_between(a, b, ref_axis):
    """
    Compute the signed angle (in radians) between two vectors a and b,
    using ref_axis to define the sign direction (right-hand rule).

    :param a: First vector (unit or raw)
    :param b: Second vector (unit or raw)
    :param ref_axis: Axis to define positive rotation direction (unit or raw)
    :return: Signed angle in radians
    """
    a = a / np.linalg.norm(a)
    b = b / np.linalg.norm(b)
    ref_axis = ref_axis / np.linalg.norm(ref_axis)

    dot = np.clip(np.dot(a, b), -1.0, 1.0)
    angle = np.arccos(dot)

    cross = np.cross(a, b)
    sign = np.sign(np.dot(cross, ref_axis))

    return angle * sign

def angles_to_normal_vector(normal_vector, quaternion):
    """
    Computes the angles between the coordinate axes of a given quaternion and a plane's normal vector.
    """
    # Ensure quaternion is numerical
    quaternion_t = np.array([float(quaternion.x), float(quaternion.y), 
                             float(quaternion.z), float(quaternion.w)], dtype=float)

    # Ensure normal_vector is a proper float array
    normal_vector = np.array([float(sp.N(coord)) for coord in normal_vector], dtype=float)

    # Rotation from quaternion
    rotation_matrix = R.from_quat(quaternion_t).as_matrix()

    angles = []
    for i in range(3):
        local_axis = rotation_matrix[:, i]

        # Compute dot product (ensuring numerical)
        dot_product = np.dot(local_axis, normal_vector)

        # Clip to valid range and compute angle
        cos_theta = np.clip(dot_product, -1.0, 1.0)
        angle = np.arccos(cos_theta)

        angles.append(angle)

    return angles

if __name__ == "__main__":

    # Test the function
    # Define the planes
    pose_3 = Pose()
    pose_3.position.x = -0.00155326
    pose_3.position.y = -0.0216407
    pose_3.position.z = 0.00233789
    
    pose_2 = Pose()
    pose_2.position.x = -0.00224831
    pose_2.position.y = -0.00156493
    pose_2.position.z = 0.00163683


    pose_1 = Pose() 
    pose_1.position.x = -0.0321483
    pose_1.position.y = -0.00231629
    pose_1.position.z = 0.00166307

    pose_4 = Pose() 
    pose_4.position.x = -0.0316481
    pose_4.position.y = -0.0221979
    pose_4.position.z = 0.00235735
            
    pose_list = [pose_1, 
                 pose_2, 
                 pose_3,
                #pose_4
                ]

    plane, centre, vectors = create_3D_plane_2(pose_list)

    pose_1.position.z =  pose_1.position.z + 0.1

    result_pose = project_pose_on_plane(pose_1, plane)

    # print(f"Vector man: {norm_vector}")
    # #quad, centroid = compute_eigenvectors_and_centroid([pose_1,pose_2])
    # quad, centroid = compute_eigenvectors_and_centroid([pose_1,pose_2,pose_3,pose_4])
    # print(quad)
    # print(centroid)
    
