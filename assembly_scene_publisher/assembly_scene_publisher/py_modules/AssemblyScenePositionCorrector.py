from assembly_scene_publisher.py_modules.AssemblySceneAnalyzer import AssemblySceneAnalyzer
from assembly_scene_publisher.py_modules.scene_errors import RefFrameNotFoundError
from rclpy.node import Node
import assembly_manager_interfaces.msg as ami_msg
import assembly_manager_interfaces.srv as ami_srv
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
import time
from copy import deepcopy
from geometry_msgs.msg import Transform, Vector3, Quaternion, Pose
from assembly_scene_publisher.py_modules.geometry_functions import multiply_ros_transforms, inverse_ros_transform

import numpy as np
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Pose, Transform


class AssemblyScenePositionCorrector:
    def __init__(self, node: Node, 
                 get_scene_callback,
                 update_scene_with_constraints_callback,
                 assembly_scene_topic: str = '/assembly_manager/scene'):
        
        self.node = node
        self.get_scene_callback = get_scene_callback
        self.update_scene_with_constraints_callback = update_scene_with_constraints_callback
        # create subscriber to listen for scene updates and correct positions when a new scene is received
        self.scene = None  # This will hold the current scene for comparison
        self.scene_subscription = node.create_subscription(
            ami_msg.ObjectScene,
            assembly_scene_topic,
            self.scene_callback,
            10,
            callback_group=ReentrantCallbackGroup()
        )
        
        # Analyzer for the external scene received from the topic, will be updated in the callback
        self.external_scene_analyzer = AssemblySceneAnalyzer(None, node.get_logger())  # Initialize with None, will set the current scene in the callback
        
        self.internal_scene = ami_msg.ObjectScene()  # This will hold the original scene
        
        # Analyzer for the internal scene, will be updated in the callback
        self.internal_scene_analyzer = AssemblySceneAnalyzer(self.internal_scene, node.get_logger())  # Analyzer for corrected scene
        self.true_scene = None

        # Analyzer for the true scene, will be set when needed
        self.true_scene_analyzer = AssemblySceneAnalyzer(None, node.get_logger())  # Analyzer for the true scene, will be set when needed
        self.logger = node.get_logger()
        self.counter = 0
        self._lock = False
        # tolerances for frame comparisons (configurable)
        self.position_tolerance = 1e-5
        self.orientation_tolerance = 1e-5

    def is_locked(self):
        return self._lock
    
    def set_lock(self, value: bool):
        self._lock = value
    
    def scene_callback(self, msg: ami_msg.ObjectScene):
        
        if self.is_locked():
            #self.logger.warn("Scene callback is currently locked, skipping scene correction.")
            return
        
        start = time.perf_counter()
        
        # get the current scene from the callback
        self.external_scene_analyzer.set_scene(msg)  # Update the analyzer with the new scene

        # iterate through components in the received scene
        for component in msg.objects_in_scene:
            component: ami_msg.Object

            if not self.internal_scene_analyzer.check_component_exists(component.obj_name):
                # store a deep copy of the incoming object to avoid shared-message mutation
                new_component = deepcopy(component)
                new_component.ref_frames = []  # Clear frames to be added separately to avoid shared-state issues
                self.internal_scene.objects_in_scene.append(new_component)
                #self.logger.debug(f"Component {component.obj_name} missing in internal scene, adding it.")
            else:
                existing_component = self.internal_scene_analyzer.get_component_by_name(component.obj_name)
                # copy properties to avoid aliasing with the incoming message
                existing_component.obj_pose = deepcopy(component.obj_pose)  # Update pose of the existing component

            # iterate through frames in the component of the received scene
            for frame in component.ref_frames:
                frame: ami_msg.RefFrame

                if self.internal_scene_analyzer.check_frames_exist_in_scene(frame_names=[frame.frame_name]):
                    _, frame_from_internal_scene = self.internal_scene_analyzer.get_frame_from_scene(frame.frame_name)
                    # copy properties/constraints to avoid aliasing with the incoming message
                    frame_from_internal_scene.properties = deepcopy(frame.properties)  # Update properties of the existing frame in the internal scene
                    frame_from_internal_scene.constraints = deepcopy(frame.constraints)  # Update constraints of the existing frame in the internal scene
                    continue
                
                if self.external_scene_analyzer.is_frame_constrained(frame_name=frame.frame_name, except_in_plane = True):
                    continue
                
                else:
                    internal_component = self.internal_scene_analyzer.get_component_by_name(component.obj_name)
                    if internal_component is None:
                        #self.logger.debug(f"internal component {component.obj_name} not found when adding frame {frame.frame_name}")
                        continue
                    
                    # append a deep copy of the frame to avoid shared-state with incoming message
                    internal_component.ref_frames.append(deepcopy(frame))
                    #self.logger.warn(f"Frame {frame.frame_name} missing in internal scene, adding it.")

        duration_ms = (time.perf_counter() - start) * 1000
        self.counter += 1
        self.logger.debug(f"Scene correction check completed. Total corrections made: {self.counter}, Duration: {duration_ms:.2f} ms")


        # delete later
        for obj in self.internal_scene.objects_in_scene:
            obj:ami_msg.Object
            for frame in obj.ref_frames:
                frame: ami_msg.RefFrame
                self.logger.debug(
                    f"Internal Scene Frame: {obj.obj_name}, "
                    f"Frame: {frame.frame_name}, "
                    f"Properties: {self.internal_scene_analyzer.get_property_types_for_frame(frame.frame_name)}"
                )

    def itentify_relevant_frames(
        self,
        component_name: str,
        reference_frame: str = "",
    ) -> list[ami_msg.RefFrame]:
        # Identify frames that are relevant for the position correction of the component
        relevant_frames = []
        component = self.internal_scene_analyzer.get_component_by_name(component_name)  # Ensure the component exists in the internal scene

        if reference_frame and not any(
            frame.frame_name == reference_frame for frame in component.ref_frames
        ):
            raise RefFrameNotFoundError(reference_frame)

        for frame in component.ref_frames:
            frame: ami_msg.RefFrame

            if reference_frame:
                if frame.frame_name == reference_frame:
                    relevant_frames.append(frame)
                continue

            if self.check_frames_equal(frame.frame_name):
                continue
            
            if frame.constraints.centroid.is_active:
                self.logger.warn(f"Frame {frame.frame_name} is constrained by centroid, skipping it for position correction.")
                continue

            if frame.properties.vision_frame_properties.is_vision_frame:
                self.logger.warn(f"Frame {frame.frame_name} is a vision frame, skipping it for position correction.")
                continue

            relevant_frames.append(frame)

        return relevant_frames
    

    def update_true_scene(self):
        # Update the true scene analyzer with the corrected internal scene
        self.true_scene = self.get_scene_callback()
        self.true_scene_analyzer.set_scene(self.true_scene)

    @staticmethod
    def _format_pose(pose_or_transform) -> str:
        position = getattr(pose_or_transform, "position", None)
        if position is None:
            position = pose_or_transform.translation

        orientation = getattr(pose_or_transform, "orientation", None)
        if orientation is None:
            orientation = pose_or_transform.rotation

        return (
            "position_um="
            f"({position.x * 1e6:.3f}, {position.y * 1e6:.3f}, "
            f"{position.z * 1e6:.3f}), "
            "orientation="
            f"({orientation.x:.9f}, {orientation.y:.9f}, "
            f"{orientation.z:.9f}, {orientation.w:.9f})"
        )

    @staticmethod
    def _pose_delta(current_pose: Pose, target_pose_or_transform) -> tuple[float, float]:
        current_position = np.array([
            current_pose.position.x,
            current_pose.position.y,
            current_pose.position.z,
        ])
        target_position_msg = getattr(target_pose_or_transform, "position", None)
        if target_position_msg is None:
            target_position_msg = target_pose_or_transform.translation
        target_position = np.array([
            target_position_msg.x,
            target_position_msg.y,
            target_position_msg.z,
        ])
        translation_delta = float(np.linalg.norm(target_position - current_position))

        current_quaternion = np.array([
            current_pose.orientation.x,
            current_pose.orientation.y,
            current_pose.orientation.z,
            current_pose.orientation.w,
        ])
        target_orientation = getattr(target_pose_or_transform, "orientation", None)
        if target_orientation is None:
            target_orientation = target_pose_or_transform.rotation
        target_quaternion = np.array([
            target_orientation.x,
            target_orientation.y,
            target_orientation.z,
            target_orientation.w,
        ])

        current_norm = np.linalg.norm(current_quaternion)
        target_norm = np.linalg.norm(target_quaternion)
        if current_norm == 0.0 or target_norm == 0.0:
            return translation_delta, float("inf")

        quaternion_dot = abs(np.dot(
            current_quaternion / current_norm,
            target_quaternion / target_norm,
        ))
        rotation_delta = float(2.0 * np.arccos(np.clip(quaternion_dot, 0.0, 1.0)))
        return translation_delta, rotation_delta

    def transform_component(
        self,
        component_name: str,
        transform: Transform,
    ):
        
        # o - origin
        # B - component
        # k - key frame
        # transformation = o_T_b_1
        
        # b_T_0_1
        inv_transform = inverse_ros_transform(transform, output_type=Transform)

        # is equal to the internal component pose, which is the original pose before correction
        true_component = self.true_scene_analyzer.get_component_by_name(component_name)
        
        o_T_b_0 = deepcopy(true_component.obj_pose)

        for frame in true_component.ref_frames:
            frame: ami_msg.RefFrame
            
            o_T_f_0 = multiply_ros_transforms(o_T_b_0, frame.pose, output_type=Transform)

            b_T_f_1 = multiply_ros_transforms(inv_transform, o_T_f_0, output_type=Pose)

            frame.pose = b_T_f_1

        true_component.obj_pose.position.x = transform.translation.x
        true_component.obj_pose.position.y = transform.translation.y
        true_component.obj_pose.position.z = transform.translation.z
        true_component.obj_pose.orientation = transform.rotation

    def correct_component_position(
        self,
        component_name: str,
        reference_frame: str = "",
    ) -> bool:
        self.set_lock(True)
        try:
            self.update_true_scene()

            relevant_frames = self.itentify_relevant_frames(
                component_name,
                reference_frame,
            )
            relevant_frames_str = [frame.frame_name for frame in relevant_frames]

            if len(relevant_frames) == 0:
                frame_suffix = (
                    f" using reference frame {reference_frame}"
                    if reference_frame
                    else ""
                )
                self.logger.warning(
                    f"No relevant frames found for component {component_name}"
                    f"{frame_suffix}, skipping position correction."
                )
                return True

            transform = self.calculate_transform(relevant_frames)

            current_component = self.true_scene_analyzer.get_component_by_name(
                component_name
            )
            current_component_pose = deepcopy(current_component.obj_pose)
            frame_world_poses_before = {
                frame.frame_name: multiply_ros_transforms(
                    current_component_pose,
                    frame.pose,
                    output_type=Pose,
                )
                for frame in current_component.ref_frames
            }
            translation_delta, rotation_delta = self._pose_delta(
                current_component_pose,
                transform,
            )

            self.logger.warning(
                f"Relevant frames for component {component_name}: {relevant_frames_str}"
            )
            self.logger.warning(
                f"Current pose for component {component_name}: "
                f"{self._format_pose(current_component_pose)}"
            )
            self.logger.warning(
                f"Calculated target pose for component {component_name}: "
                f"{self._format_pose(transform)}"
            )
            self.logger.warning(
                f"Calculated movement for component {component_name}: "
                f"translation={translation_delta * 1e6:.3f} um, "
                f"rotation={np.degrees(rotation_delta):.9f} deg"
            )

            if (
                translation_delta <= self.position_tolerance
                and rotation_delta <= self.orientation_tolerance
            ):
                self.logger.warning(
                    f"Component {component_name} has not been moved because the "
                    "calculated target pose matches its current pose within "
                    f"tolerance (position={self.position_tolerance * 1e6:.3f} um, "
                    f"orientation={np.degrees(self.orientation_tolerance):.9f} deg)."
                )

            self.transform_component(
                component_name,
                transform,
            )

            frame_poses_to_preserve = {
                frame.frame_name: deepcopy(frame.pose)
                for frame in current_component.ref_frames
            }

            self.clear_internal_scene()
        finally:
            self.set_lock(False)

        self.update_scene_with_constraints_callback(
            frame_poses_to_preserve=frame_poses_to_preserve
        )

        self.true_scene = self.get_scene_callback()
        self.true_scene_analyzer.set_scene(self.true_scene)
        applied_component = self.true_scene_analyzer.get_component_by_name(
            component_name
        )
        applied_translation_delta, applied_rotation_delta = self._pose_delta(
            current_component_pose,
            applied_component.obj_pose,
        )
        target_translation_error, target_rotation_error = self._pose_delta(
            applied_component.obj_pose,
            transform,
        )

        self.logger.warning(
            f"Applied scene pose for component {component_name}: "
            f"{self._format_pose(applied_component.obj_pose)}"
        )

        for frame in applied_component.ref_frames:
            previous_world_pose = frame_world_poses_before.get(frame.frame_name)
            if previous_world_pose is None:
                continue

            applied_world_pose = multiply_ros_transforms(
                applied_component.obj_pose,
                frame.pose,
                output_type=Pose,
            )
            frame_translation_drift, frame_rotation_drift = self._pose_delta(
                previous_world_pose,
                applied_world_pose,
            )
            if (
                frame_translation_drift > self.position_tolerance
                or frame_rotation_drift > self.orientation_tolerance
            ):
                self.logger.warning(
                    f"World pose changed for frame {frame.frame_name} while "
                    f"correcting component {component_name}: "
                    f"translation drift={frame_translation_drift * 1e6:.3f} um, "
                    f"rotation drift={np.degrees(frame_rotation_drift):.9f} deg."
                )

        if (
            translation_delta > self.position_tolerance
            or rotation_delta > self.orientation_tolerance
        ) and (
            applied_translation_delta <= self.position_tolerance
            and applied_rotation_delta <= self.orientation_tolerance
        ):
            self.logger.warning(
                f"Component {component_name} has not been moved in the scene, "
                "although the calculated target differs from its previous pose."
            )
        elif (
            target_translation_error > self.position_tolerance
            or target_rotation_error > self.orientation_tolerance
        ):
            self.logger.warning(
                f"Applied pose for component {component_name} does not match the "
                "calculated target: "
                f"translation error={target_translation_error * 1e6:.3f} um, "
                f"rotation error={np.degrees(target_rotation_error):.9f} deg."
            )
        return True
    
    def calculate_transform(self, frame_list: list[ami_msg.RefFrame]) -> Transform:
        # Placeholder for transform calculation logic based on the differences between the frames in the internal and true scenes
        # This function should compute the necessary transform to correct the position of the component based on the discrepancies in the frames
        # For now, we return an identity transform as a placeholder
        
        internal_component_name = self.internal_scene_analyzer.get_component_for_frame_name(frame_list[0].frame_name)
        internal_component = self.internal_scene_analyzer.get_component_by_name(internal_component_name)
        
        # 0_T_b_0
        internal_component_pose = internal_component.obj_pose

        if len(frame_list) == 1:
            # b_T_k_0
            b_T_k_0 = self.internal_scene_analyzer.get_frame_from_scene(frame_list[0].frame_name)[1].pose

            # b_T_k_1
            b_T_k_1 = self.true_scene_analyzer.get_frame_from_scene(frame_list[0].frame_name)[1].pose
            
            o_T_k_1 = multiply_ros_transforms(internal_component_pose,
                                              b_T_k_1,
                                              output_type=Pose)
            
            k_T_b_0 = inverse_ros_transform(b_T_k_0, output_type=Transform)
            # Compute the transform needed to correct the position
            # transform = multiply_ros_transforms(k_T_b_0, 
            #                                     b_T_k_1)

            o_T_b_1 = multiply_ros_transforms(o_T_k_1, 
                                                k_T_b_0,
                                                output_type=Transform)
            self.logger.warning(
                f"Single-frame correction input for {frame_list[0].frame_name}: "
                f"stored frame pose [{self._format_pose(b_T_k_0)}], "
                f"measured frame pose [{self._format_pose(b_T_k_1)}], "
                f"base component pose [{self._format_pose(internal_component_pose)}]"
            )
            return o_T_b_1

        if len(frame_list) == 2:
            frame_1_initial_pose = self.internal_scene_analyzer.get_frame_from_scene(frame_list[0].frame_name)[1].pose
            frame_2_initial_pose = self.internal_scene_analyzer.get_frame_from_scene(frame_list[1].frame_name)[1].pose

            frame_1_current_pose = self.true_scene_analyzer.get_frame_from_scene(frame_list[0].frame_name)[1].pose
            frame_2_current_pose = self.true_scene_analyzer.get_frame_from_scene(frame_list[1].frame_name)[1].pose

            transform = best_fit_transform_two_poses(p1=frame_1_initial_pose,
                                                     p2 = frame_2_initial_pose,
                                                     p3 = frame_1_current_pose,
                                                     p4 = frame_2_current_pose)

            return transform 
        
        if len(frame_list) > 2:
            
            initial_pose_list = []
            current_pose_list = []
            for frame in frame_list:
                frame:ami_msg.RefFrame
                _pose_initial = self.internal_scene_analyzer.get_frame_from_scene(frame.frame_name)[1].pose
                _pose_current = self.true_scene_analyzer.get_frame_from_scene(frame.frame_name)[1].pose
                initial_pose_list.append(_pose_initial)
                current_pose_list.append(_pose_current)
            
            transform  = best_fit_transform_from_poses(initial_poses=initial_pose_list,
                                                       measured_poses=current_pose_list)
            
            return transform
        
    
    def clear_internal_scene(self):
        self.internal_scene = ami_msg.ObjectScene()
        self.internal_scene_analyzer.set_scene(self.internal_scene)


    def check_frames_equal(self, frame_name: str) -> bool:
        """
        This function compares the position and orientation of a specific frame in the true scene and the internal scene to check if they match within a certain tolerance.
        
        :param self: Description
        :param frame_name: Description
        :type frame_name: str
        :return: Description
        :rtype: bool
        """
        frame_true = self.true_scene_analyzer.get_frame_from_scene(frame_name)
        frame_internal = self.internal_scene_analyzer.get_frame_from_scene(frame_name)

        if frame_true is None or frame_internal is None:
            self.logger.debug(f"Frame {frame_name} missing in one of the scenes when checking equality")
            return False

        true_frame = frame_true[1]
        internal_frame = frame_internal[1]

        true_position = true_frame.pose.position
        internal_position = internal_frame.pose.position

        position_match = (abs(true_position.x - internal_position.x) < self.position_tolerance and
                          abs(true_position.y - internal_position.y) < self.position_tolerance and
                          abs(true_position.z - internal_position.z) < self.position_tolerance)
        
        true_orientation = true_frame.pose.orientation
        internal_orientation = internal_frame.pose.orientation
        orientation_match = (abs(true_orientation.x - internal_orientation.x) < self.orientation_tolerance and
                             abs(true_orientation.y - internal_orientation.y) < self.orientation_tolerance and
                             abs(true_orientation.z - internal_orientation.z) < self.orientation_tolerance and
                             abs(true_orientation.w - internal_orientation.w) < self.orientation_tolerance)

        self.logger.debug(f"Checking frame {frame_name}: Position match: {position_match}, Orientation match: {orientation_match}")

        return position_match and orientation_match
    

def best_fit_transform_two_poses(
    p1: Pose,
    p2: Pose,
    q1: Pose,
    q2: Pose
    ) -> Transform:
    """
    Compute rigid transform (ROS Transform) that aligns
    initial poses p1, p2 to measured poses q1, q2.

    Uses positions only.
    """

    # --- Extract positions ---
    P1 = np.array([p1.position.x, p1.position.y, p1.position.z])
    P2 = np.array([p2.position.x, p2.position.y, p2.position.z])
    Q1 = np.array([q1.position.x, q1.position.y, q1.position.z])
    Q2 = np.array([q2.position.x, q2.position.y, q2.position.z])

    # --- Direction vectors ---
    v_p = P2 - P1
    v_q = Q2 - Q1

    norm_p = np.linalg.norm(v_p)
    norm_q = np.linalg.norm(v_q)

    if norm_p < 1e-8 or norm_q < 1e-8:
        raise ValueError("Points must not be identical")

    v_p_hat = v_p / norm_p
    v_q_hat = v_q / norm_q

    # --- Compute rotation ---
    cross = np.cross(v_p_hat, v_q_hat)
    dot = np.dot(v_p_hat, v_q_hat)
    dot = np.clip(dot, -1.0, 1.0)

    cross_norm = np.linalg.norm(cross)

    if cross_norm < 1e-8:
        # Parallel case
        if dot > 0:
            rot = R.identity()
        else:
            # 180° rotation around any perpendicular axis
            axis = np.array([1.0, 0.0, 0.0])
            if abs(v_p_hat[0]) > 0.9:
                axis = np.array([0.0, 1.0, 0.0])
            axis = axis - axis.dot(v_p_hat) * v_p_hat
            axis /= np.linalg.norm(axis)
            rot = R.from_rotvec(np.pi * axis)
    else:
        axis = cross / cross_norm
        angle = np.arccos(dot)
        rot = R.from_rotvec(axis * angle)

    # --- Compute translation ---
    t = Q1 - rot.apply(P1)

    # --- Convert to ROS Transform ---
    transform = Transform()

    transform.translation.x = float(t[0])
    transform.translation.y = float(t[1])
    transform.translation.z = float(t[2])

    qx, qy, qz, qw = rot.as_quat()

    transform.rotation.x = float(qx)
    transform.rotation.y = float(qy)
    transform.rotation.z = float(qz)
    transform.rotation.w = float(qw)

    return transform



def best_fit_transform_from_poses(
    initial_poses: list[Pose],
    measured_poses: list[Pose]
) -> Transform:
    """
    Compute least-squares rigid transform aligning initial_poses to measured_poses.

    Requires at least 3 pose pairs.
    Uses Kabsch algorithm (SVD).
    Returns ROS Transform.
    """

    if len(initial_poses) != len(measured_poses):
        raise ValueError("Pose lists must have same length")

    if len(initial_poses) < 3:
        raise ValueError("At least 3 poses required")

    # --- Extract position arrays ---
    P = np.array([
        [p.position.x, p.position.y, p.position.z]
        for p in initial_poses
    ])

    Q = np.array([
        [p.position.x, p.position.y, p.position.z]
        for p in measured_poses
    ])

    # --- Compute centroids ---
    centroid_P = np.mean(P, axis=0)
    centroid_Q = np.mean(Q, axis=0)

    # --- Center points ---
    P_centered = P - centroid_P
    Q_centered = Q - centroid_Q

    # --- Covariance matrix ---
    H = P_centered.T @ Q_centered

    # --- SVD ---
    U, S, Vt = np.linalg.svd(H)

    # --- Compute rotation ---
    R_mat = Vt.T @ U.T

    # --- Reflection correction ---
    if np.linalg.det(R_mat) < 0:
        Vt[2, :] *= -1
        R_mat = Vt.T @ U.T

    rot = R.from_matrix(R_mat)

    # --- Compute translation ---
    t = centroid_Q - R_mat @ centroid_P

    # --- Convert to ROS Transform ---
    transform = Transform()

    transform.translation.x = float(t[0])
    transform.translation.y = float(t[1])
    transform.translation.z = float(t[2])

    qx, qy, qz, qw = rot.as_quat()

    transform.rotation.x = float(qx)
    transform.rotation.y = float(qy)
    transform.rotation.z = float(qz)
    transform.rotation.w = float(qw)

    return transform
