"""Associate existing scene frames and calculate frame-match transforms."""

import numpy as np
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation

from assembly_scene_publisher.py_modules.frame_properties import scene_frame_name
from assembly_scene_publisher.py_modules.scene_errors import (
    AssemblyTransformationError,
)
from assembly_scene_publisher.py_modules.tf_functions import (
    get_transform_for_frame_in_world,
)


class FrameMatchHandler:
    """Use existing frames without generating or changing their poses."""

    def __init__(self, analyzer, tf_buffer):
        self._analyzer = analyzer
        self._tf_buffer = tf_buffer

    def get_frames(self, instruction):
        match = instruction.frame_match
        if match.assembly_frame.component == match.target_frame.component:
            raise AssemblyTransformationError(
                'FrameMatch endpoints must belong to different components.')
        self._analyzer.get_component_by_name(match.assembly_frame.component)
        self._analyzer.get_component_by_name(match.target_frame.component)
        return (self._resolve_frame(match.assembly_frame),
                self._resolve_frame(match.target_frame))

    def _resolve_frame(self, reference):
        if not reference.frame or not reference.component:
            raise AssemblyTransformationError(
                'FrameMatch references require component and frame names.')
        full_name = scene_frame_name(reference.component, reference.frame)
        frame = self._analyzer.get_ref_frame_by_name(full_name)
        owner = self._analyzer.get_component_for_frame_name(full_name)
        if owner != reference.component:
            raise AssemblyTransformationError(
                f'Frame {full_name} does not belong to {reference.component}.')
        return frame

    def attach(self, instruction):
        """Associate the two validated frames, retaining their other roles."""
        assembly, target = self.get_frames(instruction)
        # Validate both sides before mutating either frame.
        for frame, other in ((assembly, target), (target, assembly)):
            associated = frame.properties.assembly_frame_properties.associated_frame
            if associated and associated != other.frame_name:
                raise AssemblyTransformationError(
                    f'Frame {frame.frame_name} is already associated with '
                    f'{associated}.')
        for frame, other, is_assembly in (
                (assembly, target, True), (target, assembly, False)):
            properties = frame.properties.assembly_frame_properties
            properties.is_assembly_frame = is_assembly
            properties.is_target_frame = not is_assembly
            properties.associated_frame = other.frame_name
            properties.associated_component = (
                instruction.frame_match.target_frame.component if is_assembly
                else instruction.frame_match.assembly_frame.component)

    def calculate(self, instruction):
        """Return the world-space rigid displacement mapping assembly to target.

        delta = world_T_target @ inverse(world_T_assembly).
        """
        assembly, target = self.get_frames(instruction)
        assembly_world = self._world_matrix(assembly)
        target_world = self._world_matrix(target)
        delta = target_world @ np.linalg.inv(assembly_world)
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = map(
            float, delta[:3, 3])
        quaternion = Rotation.from_matrix(delta[:3, :3]).as_quat()
        (pose.orientation.x, pose.orientation.y,
         pose.orientation.z, pose.orientation.w) = map(float, quaternion)
        return pose

    def _world_matrix(self, frame):
        parent = get_transform_for_frame_in_world(
            frame.parent_frame, self._tf_buffer).transform
        return self._matrix(parent.translation, parent.rotation) @ \
            self._matrix(frame.pose.position, frame.pose.orientation)

    @staticmethod
    def _matrix(position, orientation):
        matrix = np.eye(4)
        matrix[:3, :3] = Rotation.from_quat([
            orientation.x, orientation.y, orientation.z, orientation.w,
        ]).as_matrix()
        matrix[:3, 3] = [position.x, position.y, position.z]
        return matrix
