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


def compute_eigenvectors_and_centroid(poses: list[Pose], 
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

def compute_eigenvectors_and_centroid_old(poses):
    """
    Computes the best-fit plane for given poses and returns its rotation as a quaternion and the centroid.

    Args:
        poses (list of Pose): List of ROS2 Pose objects.

    Returns:
        tuple: (quaternion, centroid_vector)
    """
    positions = []

    if len(poses) < 2:
        raise ValueError("At least 2 poses are required to compute eigenvectors and centroid.")

    # Extract positions
    for pose in poses:
        positions.append([pose.position.x, pose.position.y, pose.position.z])

    positions = np.array(positions)
    centroid = np.mean(positions, axis=0)

    # Special handling for two points
    if len(poses) == 2:
        p1, p2 = positions
        direction = p2 - p1  # Vector along the line
        arbitrary = np.array([1, 0, 0])  # Arbitrary vector
        
        # If direction is along x-axis, pick y-axis
        if np.allclose(direction, [1, 0, 0]):
            arbitrary = np.array([0, 1, 0])

        # Compute a normal via cross product
        normal = np.cross(direction, arbitrary)
        normal = normal / np.linalg.norm(normal)  # Normalize
    else:
        # General case: Use SVD to find the best plane
        shifted_points = positions - centroid
        _, _, vh = np.linalg.svd(shifted_points)
        normal = vh[-1]  # Smallest singular value corresponds to normal

    # Convert normal vector to quaternion representation
    rotation_matrix = np.eye(3)
    rotation_matrix[:, 2] = normal  # Set normal as Z-axis direction
    rotation_matrix[:, 1] = np.cross(normal, [1, 0, 0])  # X cross normal → Y-axis
    rotation_matrix[:, 1] /= np.linalg.norm(rotation_matrix[:, 1])
    rotation_matrix[:, 0] = np.cross(rotation_matrix[:, 1], normal)  # Ensure right-handed system

    #quat = Rotation.from_matrix(rotation_matrix).as_quat()
    quat = rotation_matrix_to_quaternion(rotation_matrix)

    quad_2 = Quaternion()
    quad_2.x = quat[1]
    quad_2.y = quat[2]
    quad_2.z = quat[3]
    quad_2.w = quat[0]
    
    centroid_vector = Vector3()
    centroid_vector.x = centroid[0]
    centroid_vector.y = centroid[1]
    centroid_vector.z = centroid[2]

    return quad_2, centroid_vector

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


import numpy as np

def perform_pca(points, n_components=None):
    """
    Perform Principal Component Analysis (PCA) on a set of 2D or 3D points.

    Args:
        points (list of tuples/lists): A list of (x, y) or (x, y, z) coordinates.
        n_components (int, optional): Number of principal components to keep (default: all).

    Returns:
        eigenvalues (numpy array): Eigenvalues sorted in descending order.
        eigenvectors (numpy array): Corresponding eigenvectors.
        transformed_points (numpy array): Points transformed into the PCA space.
    """
    points = np.array(points)  # Convert to NumPy array

    # Ensure at least two points
    if points.shape[0] < 2:
        raise ValueError("At least two points are required for PCA.")

    # Step 1: Compute mean and center the data
    mean = np.mean(points, axis=0)
    centered_points = points - mean

    # Step 2: Compute covariance matrix
    covariance_matrix = np.cov(centered_points.T)

    # Step 3: Compute eigenvalues & eigenvectors
    eigenvalues, eigenvectors = np.linalg.eig(covariance_matrix)

    # Step 4: Sort eigenvalues & eigenvectors in descending order
    sorted_indices = np.argsort(eigenvalues)[::-1]
    eigenvalues = eigenvalues[sorted_indices]
    eigenvectors = eigenvectors[:, sorted_indices]

    # Step 5: Project data onto principal components
    if n_components is not None:
        eigenvectors = eigenvectors[:, :n_components]  # Keep only the top components
    transformed_points = np.dot(centered_points, eigenvectors)

    return eigenvalues, eigenvectors, transformed_points

# Example usage with 3D points
points = [(1, 2, 3), (4, 5, 6), (7, 8, 9), (2, 1, 3)]
eigenvalues, eigenvectors, transformed_points = perform_pca(points)

print("Eigenvalues:", eigenvalues)
print("Eigenvectors:\n", eigenvectors)
print("Transformed Points:\n", transformed_points)


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
        
    vector = Vector3()
    vector.x = pose_2.position.x - pose_1.position.x
    vector.y = pose_2.position.y - pose_1.position.y
    vector.z = pose_2.position.z - pose_1.position.z
    
    norm_vector = normalize_vector3(vector)
        
    print(f"Vector man: {norm_vector}")
    #quad, centroid = compute_eigenvectors_and_centroid([pose_1,pose_2])
    quad, centroid = compute_eigenvectors_and_centroid([pose_1,pose_2,pose_3,pose_4])
    print(quad)
    print(centroid)
    
    
    import subprocess

    def get_all_ros2_executables():
        result = subprocess.run(["ros2", "pkg", "executables"], capture_output=True, text=True)
        executables = result.stdout.strip().split("\n")
        for line in executables:
            if line:
                package, executable = line.split()
                print(f"Package: {package}, Executable: {executable}")

    get_all_ros2_executables()