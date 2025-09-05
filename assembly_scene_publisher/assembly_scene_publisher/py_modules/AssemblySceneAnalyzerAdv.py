from assembly_scene_publisher.py_modules.AssemblySceneAnalyzer import AssemblySceneAnalyzer
from assembly_manager_interfaces.msg import ObjectScene
from rclpy.impl.rcutils_logger import RcutilsLogger
import assembly_manager_interfaces.msg as ami_msg
from geometry_msgs.msg import Pose, Transform

from assembly_scene_publisher.py_modules.scene_errors import (RefAxisNotFoundError, 
                                                              RefFrameNotFoundError, 
                                                              RefPlaneNotFoundError, 
                                                              ComponentNotFoundError,
                                                              GrippingFrameNotFoundError)

from copy import copy
from typing import Union
import time


PM_ROBOT_GRIPPER_FRAME = 'PM_Robot_Tool_TCP'
PM_ROBOT_GONIO_LEFT_FRAME_INDICATOR = 'Gonio_Left_Part'
PM_ROBOT_GONIO_RIGHT_FRAME_INDICATOR = 'Gonio_Right_Part'
GRIPPING_FRAME_IDENTIFICATORS = ['Grip', 'grip']

class AssemblySceneAnalyzerAdv(AssemblySceneAnalyzer):
    def __init__(self, scene_data: ObjectScene, logger: RcutilsLogger = None):
        super().__init__(scene_data, logger)


    def get_gripped_component(self)-> str:
        for obj in self._get_scene().objects_in_scene:
            obj:ami_msg.Object
            if obj.parent_frame == PM_ROBOT_GRIPPER_FRAME:
                return obj.obj_name
        return None
    

    def get_assembly_target_frame_gripped_component(self)->list[str]:
        """
        Get the assembly and target frames for the gripped component. 

        Returns:
            list[str]: List of assembly and target frames for the gripped component.
        """
        gripped_component = self.get_gripped_component()
        
        frames = self.get_all_assembly_and_target_frames_for_component(gripped_component)

        return frames

    def is_gripper_empty(self)-> bool:
        
        for obj in self._get_scene().objects_in_scene:
            if self.logger is not None:
                self.logger.warn(f"Object: {obj.obj_name}, Parent Frame: {obj.parent_frame}")
            obj:ami_msg.Object
            if obj.parent_frame == PM_ROBOT_GRIPPER_FRAME:
                return False
        return True
    

    def is_object_on_gonio_left(self, object_name: str, max_depth: int = 3) -> bool:
        """
        Check if the object is on the gonio left frame, up to a specified depth.

        :param object_name: Name of the object to check.
        :param max_depth: How many levels up the hierarchy to check.
        :return: True if the object is on or connected to the gonio left frame.
        """
        current_object = object_name
        try:
            for _ in range(max_depth):
                parent_frame = self.get_parent_of_component(current_object)

                if PM_ROBOT_GONIO_LEFT_FRAME_INDICATOR in parent_frame:
                    return True
                current_object = parent_frame

            return False
        
        except ComponentNotFoundError as e:
            return False

    def is_object_on_gonio_right(self, object_name: str, max_depth: int = 3) -> bool:
        """
        Check if the object is on the gonio right frame, up to a specified depth.
        
        :param object_name: Name of the object to check.
        :param max_depth: How many levels up the hierarchy to check.
        :return: True if the object is on or connected to the gonio right frame.
        """
        current_object = object_name

        try:
            for _ in range(max_depth):
                parent_frame = self.get_parent_of_component(current_object)

                if PM_ROBOT_GONIO_RIGHT_FRAME_INDICATOR in parent_frame:
                    return True
                current_object = parent_frame

            return False
        except ComponentNotFoundError as e:
            return False
        
    def get_gripping_frame(self, component_name:str)-> str:
        """
        Get the gripping frame of a component by searching for known identifiers in its reference frames.
        :param component_name: Name of the component to search for.
        :return: Name of the gripping frame.
        :raises ComponentNotFoundError: If the component is not found in the scene.
        :raises GrippingFrameNotFoundError: If no gripping frame or multiple gripping frames are found.
        """
        grip_frames = []
        component = self.get_component_by_name(component_name)

        for frame in component.ref_frames:
            frame:ami_msg.RefFrame
            for identificador in GRIPPING_FRAME_IDENTIFICATORS:
                # check if string is part of string
                if identificador in frame.frame_name:
                    grip_frames.append(frame.frame_name)                    

        if len(grip_frames) == 0:
            raise GrippingFrameNotFoundError(component_name)
        
        if len(grip_frames) > 1:
            raise GrippingFrameNotFoundError(component_name)
        
        else:
            return grip_frames[0]
