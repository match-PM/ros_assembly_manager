from assembly_scene_publisher.py_modules.AssemblySceneAnalyzer import AssemblySceneAnalyzer
from assembly_manager_interfaces.msg import ObjectScene
from rclpy.impl.rcutils_logger import RcutilsLogger
import assembly_manager_interfaces.msg as ami_msg
from geometry_msgs.msg import Pose, Transform

from assembly_scene_publisher.py_modules.scene_errors import *

from copy import copy
from typing import Union
import time
from assembly_scene_publisher.py_modules.frame_constraints import FrameConstraintsHandler

from copy import deepcopy


class AssemblySceneModifier(AssemblySceneAnalyzer):
    def __init__(self, 
                 scene_data: ObjectScene, 
                 logger: RcutilsLogger = None):
        super().__init__(scene_data, logger)

    
    def update_ref_frame_by_constraint(self,
                                        ref_frame:ami_msg.RefFrame, 
                                        component_name:str
                                        )-> bool:
        """Updates the reference frame by applying constraints.

        Args:
            ref_frame (ami_msg.RefFrame): The reference frame to update.
            component_name (str): The name of the component to update the constraints for.

        Returns:
            bool: True if the update was successful, False otherwise.
        """

        try:
            # make a deep copy just to be sure that the original ref frame is not modified
            # this is just for safety reasons

            #self.logger.warn(f"Update ref frame constraints for frame '{ref_frame.frame_name}'")
            
            frame_constraints_handler:FrameConstraintsHandler = FrameConstraintsHandler.return_handler_from_msg(msg=ref_frame.constraints,
                                                                                                                scene=self._get_scene())
            
            pose = frame_constraints_handler.calculate_frame_constraints(initial_pose = ref_frame.pose, 
                                                                            scene=self._get_scene(),
                                                                            frame_name=ref_frame.frame_name,
                                                                            component_name = component_name,
                                                                            logger=self.logger)
            
            ref_frame.pose = pose

            return True
        
        except Exception as e:
            self.logger.error(str(e))
            self.logger.error(f"Unknown Error. Constraint could not be updated!")
            return False

    def update_frame_constraint_activation(self,
                                        ref_frame:ami_msg.RefFrame,
                                        ):
        """Checks and sets the activation of the constraints for a reference frame."""

        # make a deep copy just to be sure that the original ref frame is not modified
        # this is just for safety reasons

        frame_constraints_handler:FrameConstraintsHandler = FrameConstraintsHandler.return_handler_from_msg(msg=ref_frame.constraints,
                                                                                                            scene=self._get_scene(),
                                                                                                            logger=self.logger)
        
        frame_constraints_handler.set_activations(scene=self._get_scene())
        ref_frame.constraints = frame_constraints_handler.return_as_msg()

    def update_all_frame_constraint_activations(self):
        """
        This function updates the activation of the constraints for all reference frames in the scene. It checks if the constraints are valid and sets the activation accordingly.
        """
        
        #self.logger.info(f"Updating constraint activations for all frames in the scene...")

        for comp in self._get_scene().objects_in_scene:
            comp: ami_msg.Object

            for ref_frame in comp.ref_frames:
                self.update_frame_constraint_activation(ref_frame)

        for ref_frame in self._get_scene().ref_frames_in_scene:
            self.update_frame_constraint_activation(ref_frame)

    def calculate_frame_constraints_for_frame_list(self,
                                                frame_list: list[ami_msg.RefFrame],
                                                component_name: str = None)-> bool:
        """
        This function calculates the frame constraints for a list of frames.
        """
        def get_dict_depth(d: dict) -> int:
            """Recursively calculates the depth of a nested dictionary."""
            if not isinstance(d, dict) or not d:  # Base case: empty or non-dict
                return 0
            return 1 + max(get_dict_depth(v) for v in d.values())

        def get_unique_elements_at_depth(d: dict, depth: int) -> list:
            """Returns unique keys at a specific depth in a nested dictionary."""
            if depth < 1:
                return []
            
            if depth == 1:
                return list(set(d.keys()))  # Base case: return unique keys at the current level
            
            elements = set()  # Use a set to store unique elements
            for v in d.values():
                if isinstance(v, dict):
                    elements.update(get_unique_elements_at_depth(v, depth - 1))  # Recurse deeper

            return list(elements)  # Convert back to a list for the final output
    
        reference_tree = self.build_frame_reference_tree(frame_list)
        max_depth = get_dict_depth(reference_tree)
        calculated_frames=[]

        for depth in range(max_depth, 0, -1):
            unique_elements = get_unique_elements_at_depth(reference_tree, depth)
            #logger.warn(f"unique_elements{unique_elements}")
            for frame_name in unique_elements:
                if frame_name not in calculated_frames:


                    frame = self.get_ref_frame_by_name(frame_name)
                    self.update_ref_frame_by_constraint(frame, component_name=component_name)
                    calculated_frames.append(frame_name)
        return True

    def calculate_constraints_for_component(self,
                                            component_name: str = None)-> bool:
        """
        This function calculates the frame constraints for a component.
        """
        frame_list = []
        if component_name is None:
            frame_list = self._get_scene().ref_frames_in_scene
        else:
            comp = self.get_component_by_name(component_name)

        return self.calculate_frame_constraints_for_frame_list(frame_list, component_name=component_name)


    def calculate_constraints_for_scene(self)-> bool:
        """
        This function calculates the frame constraints for the whole scene.
        """
        #logger.warn(f"initial_scene{str(scene)}")
        # calculate the constraints for all components in the scene
        for component in self._get_scene().objects_in_scene:
            component: ami_msg.Object
            self.calculate_constraints_for_component(component.obj_name)

        # calculate the constraints for frames not associated with any component
        self.calculate_constraints_for_component()
        
        #logger.warn(f"final_scene{str(scene)}")
        return True

    def set_to_occupied_frames(self, frame_name: str)-> None:
        """
        Set the given frame name to occupied frames in the scene. This means that the frame is currently used for spawning and should not be used for other purposes. If the frame is already in the list of occupied frames, it will not be added again.
        """
        for fr in self._get_scene().occupied_spawning_frames:
            if fr == frame_name:
                return 
        self._get_scene().occupied_spawning_frames.append(frame_name)

    def del_from_occupied_frames(self, frame_name: str)-> None:
        """
        Remove the given frame name from the list of occupied frames in the scene. This means that the frame is no longer used for spawning and can be used for other purposes. If the frame is not in the list of occupied frames, nothing will happen.
        """
        for fr in self._get_scene().occupied_spawning_frames:
            if fr == frame_name:
                self._get_scene().occupied_spawning_frames.remove(frame_name)
                return