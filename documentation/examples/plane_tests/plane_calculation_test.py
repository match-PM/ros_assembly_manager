
from typing import List
from assembly_scene_publisher.py_modules.geometry_functions import create_3D_plane_2, project_pose_on_plane, rotate_point

from scipy.spatial.transform import Rotation as R
from assembly_manager_interfaces.msg import RefFrame
# import pose
from geometry_msgs.msg import Pose, Vector3, Quaternion
import random

INITIAL_FRAME = Pose()

class EulerAngles:
    def __init__(self, roll: float, pitch: float, yaw: float):
        self.roll = (roll)
        self.pitch = (pitch)
        self.yaw = (yaw)

class FrameSet():
    def __init__(self, x_value:float, y_value:float, z_values:List[float]):
        self.x_value = x_value
        self.y_value = y_value
        self.z_values = z_values

    def to_poses(self, multiplier: float = 1) -> List[Pose]:
        frames = []
        
        corners = [
            ( self.x_value,  self.y_value),
            (-self.x_value,  self.y_value),
            (-self.x_value, -self.y_value),
            ( self.x_value, -self.y_value)
        ]
        for index, z in enumerate(self.z_values):
            frame = Pose()
            corner_index = index % 4
            x_base, y_base = corners[corner_index]

            # Add a small random offset
            delta = 0.000002  # maximum offset in m
            frame.position.x = x_base * multiplier + random.uniform(-delta, delta)
            frame.position.y = y_base * multiplier + random.uniform(-delta, delta)
            frame.position.z = z * multiplier

            frames.append(frame)
        return frames

class FrameSetCollection():
    def __init__(self, frame_sets: List[FrameSet]=[], multiplier: float = 1):
        self.frame_sets = frame_sets
        self.euler_angles: List[EulerAngles] = []
        self.multiplier = multiplier

    def compute_euler_angles(self):
        for frame_set in self.frame_sets:

            frame_list = frame_set.to_poses(self.multiplier)
            plane = create_3D_plane_2(frame_list)
            new_pose_initial_frame = project_pose_on_plane(INITIAL_FRAME, plane)
            new_pose_initial_frame_rot = rotate_point(new_pose_initial_frame, plane, target_axis='Z')

            q = new_pose_initial_frame_rot.orientation
            # Create a Rotation object
            r = R.from_quat([q.x, q.y, q.z, q.w])  # Note: [x, y, z, w]
            # Convert to Euler angles (roll, pitch, yaw)
            roll, pitch, yaw = r.as_euler('xyz', degrees=True)  # Set degrees=True if you want degrees
            self.euler_angles.append(EulerAngles(roll, pitch, yaw))

    def append_frame_set(self, frame_set: FrameSet):
        self.frame_sets.append(frame_set)

    def print_euler_angles(self):
        round_ind=3
        for index, angles in enumerate(self.euler_angles):
            print(f"Frame {index}: Roll: {round(angles.roll, round_ind)}, Pitch: {round(angles.pitch, round_ind)}, Yaw: {round(angles.yaw, round_ind)}")

    def print_frame_sets(self):
        for set_index, frame_set in enumerate(self.frame_sets):
            print(f"Frame Set {set_index}:")
            poses = frame_set.to_poses(multiplier=self.multiplier)
            for i, pose in enumerate(poses):
                print(f"  Pose {i}: x={pose.position.x:.3f}, y={pose.position.y:.3f}, z={pose.position.z:.3f}")
            print("-" * 40)

if __name__ == "__main__":
    my_collection = FrameSetCollection(multiplier=100000)  # Convert to mm

    frame_set_1 = FrameSet(x_value=0.02, y_value=0.02, z_values=[0.02, 0.02, 0.00, 0.0])
    frame_set_2 = FrameSet(x_value=0.02, y_value=0.01, z_values=[0.1, 0.02, 0.01, 0.02])

    my_collection.append_frame_set(frame_set_1)
    my_collection.append_frame_set(frame_set_2)
    
    # ----------------------
    # Example 2: Tilted plane 45 deg around X
    # Expected: roll ~45 deg, pitch ~0 deg, yaw ~0 deg
    frame_set_tilt_x = FrameSet(
        x_value=0.02,
        y_value=0.02,
        z_values=[0.0, 0.0, 0.000002, 0.000002]  # z increases along +x direction
    )
   
    # ----------------------
    # Example 3: Tilted plane 30 deg around Y
    # Expected: roll ~0 deg, pitch ~30 deg, yaw ~0 deg
    frame_set_tilt_y = FrameSet(
        x_value=0.02,
        y_value=0.02,
        z_values=[0.0, 0.02, 0.02, 0.0]  # z increases along +y direction
    )

    # ----------------------
    my_collection.append_frame_set(frame_set_tilt_x)
    my_collection.append_frame_set(frame_set_tilt_y)
    
    my_collection.print_frame_sets()
    my_collection.compute_euler_angles()
    my_collection.print_euler_angles()
    print(INITIAL_FRAME)
