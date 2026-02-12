from loguru import logger
from assembly_manager_interfaces.msg import ObjectScene
from rclpy.impl.rcutils_logger import RcutilsLogger
import assembly_manager_interfaces.msg as ami_msg
from geometry_msgs.msg import Pose, Transform
from assembly_scene_publisher.py_modules.frame_constraints import FrameConstraintsHandler
from assembly_scene_publisher.py_modules.scene_errors import *

from copy import copy, deepcopy
from typing import Union
import time
from typing import Union

class AssemblyConstants:
    ASSEMBLY_FRAME_INDICATOR = 'assembly_frame_Description'
    TARGET_FRAME_INDICATOR = 'target_frame_Description'

class UnInitializedScene():
    def __init__(self):
        self.scene = None

class AssemblySceneAnalyzer():
    ASSEMBLY_FRAME_INDICATOR = 'assembly_frame_Description'
    TARGET_FRAME_INDICATOR = 'target_frame_Description'

    def __init__(self,  scene: Union[ami_msg.ObjectScene, UnInitializedScene], 
                        logger: RcutilsLogger = None):
        
        self._scene = scene

        self.logger = logger

    def _get_scene(self)-> ObjectScene:
        """Returns the current scene, initializing it if necessary."""
        if isinstance(self._scene, UnInitializedScene):
            return self._scene.scene
        else:
            return self._scene

    def set_scene(self, scene: ObjectScene):
        """Sets the current scene."""
        self._scene = scene

    # def _check_scene(self):
    #     """Checks if the scene is set, raises an error otherwise."""
    #     counter = 10

    # def _check_scene(self):
    #     """Checks if the scene is set, raises an error otherwise."""
    #     counter = 10

    #     if self.logger is not None:
    #         self.logger.warning(f"The assembly scene has not been initialized. Timeout after {counter} seconds.")

    #     while isinstance(object.__getattribute__(self, "scene"), UnInitializedScene) and counter > 0:
    #         if self.logger is not None:
    #             self.logger.warning("Waiting for scene to initialize. Make sure the assembly manager is available!")
    #         time.sleep(1)
    #         counter -= 1

    #     if isinstance(object.__getattribute__(self, "scene"), UnInitializedScene):
    #         raise RuntimeError("Scene was never initialized within timeout!")

    # def __getattribute__(self, name):
    #     """Wraps public methods to check scene before executing."""
    #     attr = object.__getattribute__(self, name)

    #     # Exclude methods that must NEVER be wrapped
    #     excluded = {
    #         "_check_scene", "_scene_is_empty", "__getattribute__", "__init__"
    #     }

    #     # Don't wrap attributes or excluded functions
    #     if callable(attr) and not name.startswith("__") and name not in excluded:
    #         def wrapper(*args, **kwargs):
    #             # Check the scene before calling
    #             self._check_scene()
    #             return attr(*args, **kwargs)
    #         return wrapper

    #     return attr
    
    def wait_for_initial_scene_update(self):
        while self._get_scene() is None:
            if self.logger is not None:
                self.logger.warn("Waiting for object scene to be updated...")
                time.sleep(1.0)

    def _scene_is_empty(self)-> bool:
        """Checks if the scene is empty."""
        if (len(self._get_scene().objects_in_scene) == 0 and
            len(self._get_scene().axis_in_scene) == 0 and
            len(self._get_scene().planes_in_scene) == 0 and
            len(self._get_scene().ref_frames_in_scene) == 0 and
            len(self._get_scene().assembly_instructions) == 0):
            return True
        return False

    def get_frames_for_plane(self, plane_name: str)-> list[ami_msg.RefFrame]:
        """
        Returns the frames associated with the given plane name.
        parameters:
        - plane_name: name of the plane to get the frames for
        raises:
        - RefPlaneNotFoundError: if the plane name is not found in the scene
        - RefAxisNotFoundError: if the axis name associated with the plane is not found in the scene
        returns:
        - list of ref frames associated with the plane
        """

        plane = self.get_plane_from_scene(plane_name)
                
        frame_list = []
        axis_name = plane.axis_names[0]
        
        if axis_name != '':
            list_ax = self.get_frames_for_axis(axis_name)
            frame_list.extend(list_ax)
        
        for point_name in plane.point_names:
            if point_name == '':
                continue
            frame = self.get_ref_frame_by_name(point_name)
            frame_list.append(frame)
                
        return frame_list

    def get_plane_from_scene(self,plane_name:str)-> ami_msg.Plane:
        """
        Returns the plane from the plane list by the given plane name
        parameters:
        - plane_name: name of the plane to get
        raises:
        - RefPlaneNotFoundError: if the plane name is not found in the scene
        returns:
        - plane with the given name
        """
        for plane in self._get_scene().planes_in_scene:
            plane: ami_msg.Plane
            if plane_name == plane.ref_plane_name:
                return plane

        for obj in self._get_scene().objects_in_scene:
            obj:ami_msg.Object
            for plane in obj.ref_planes:
                plane: ami_msg.Plane
                if plane_name == plane.ref_plane_name:
                    return plane

        raise RefPlaneNotFoundError(plane_name)
    
    def get_frames_for_axis(self, axis_name: str)-> list[ami_msg.RefFrame]:
        """
        Returns the frames associated with the given axis name.
        parameters:
        - axis_name: name of the axis to get the frames for
        raises:
        - RefAxisNotFoundError: if the axis name is not found in the scene
        - RefFrameNotFoundError: if any of the frames associated with the axis is not found in the scene
        returns:
        - list of ref frames associated with the axis
        """
        axis = self.get_axis_from_scene(axis_name)
        
        frame_list = []

        for point_name in axis.point_names:
            ref_frame = self.get_ref_frame_by_name(point_name)
            frame_list.append(ref_frame)

        return frame_list


    def get_component_by_name(  self, obj_name:str) -> ami_msg.Object:
        """
        Returns the object from the object list by the given object name
        parameters:
        - obj_name: name of the object to get
        raises:
        - ComponentNotFoundError: if the object name is not found in the scene
        returns:
        - object with the given name
        """
        for obj in self._get_scene().objects_in_scene:
            obj:ami_msg.Object
            if obj.obj_name == obj_name:
                return obj
        
        raise ComponentNotFoundError(obj_name)

    def get_axis_from_scene(self, axis_name:str)-> ami_msg.Axis:
        """
        Returns the axis from the axis list by the given axis name
        parameters:
        - axis_name: name of the axis to get
        raises:
        - RefAxisNotFoundError: if the axis name is not found in the scene
        returns:
        - axis with the given name
        """
        for axis in self._get_scene().axis_in_scene:
            axis: ami_msg.Axis
            if axis_name == axis.axis_name:
                axis_msg = axis
                return axis_msg

        for obj in self._get_scene().objects_in_scene:
            obj:ami_msg.Object
            for axis in obj.ref_axis:
                axis: ami_msg.Axis
                if axis_name == axis.axis_name:
                    axis_msg = axis
                    return axis_msg

        raise RefAxisNotFoundError(axis_name)


    def get_ref_frame_by_name(self,frame_name:str) -> ami_msg.RefFrame:
        """
        Returns the ref frame from the ref frames list by the given frame name
        parameters:
        - frame_name: name of the frame to get
        raises:
        - RefFrameNotFoundError: if the frame name is not found in the scene
        returns:
        - ref frame with the given name
        """
        for obj in self._get_scene().objects_in_scene:
            obj: ami_msg.Object
            for ref_frame in obj.ref_frames:
                ref_frame:ami_msg.RefFrame
                if ref_frame.frame_name == frame_name:
                    return ref_frame
                
        for ref_frame in self._get_scene().ref_frames_in_scene:
            ref_frame:ami_msg.RefFrame
            if ref_frame.frame_name == frame_name:
                return ref_frame
            
        raise RefFrameNotFoundError(frame_name)

    def get_ref_frame_poses_by_names(self, frame_names: list[str])-> list[Pose]:
        """
        Returns the poses of the ref frames from the ref frames list by the given frame names.
        parameters:
        - frame_names: list of frame names to get the poses for
        raises:
        - RefFrameNotFoundError: if any of the frame names is not found in the
        returns:
        - list of poses of the ref frames
        """
        
        poses = []
        
        for frame_name in frame_names:
            frame = self.get_ref_frame_by_name(frame_name)
            poses.append(frame.pose)
                
        return poses

    def get_frame_names_from_list( self, frame_list: list[ami_msg.RefFrame])-> list[str]:
        """
        Returns the names of the frames from the given list of ref frames.
        parameters:
        - frame_list: list of ref frames to get the names from
        raises:
        - ValueError: if the list contains an element that is not of type RefFrame
        returns:
        - list of frame names
        """

        frame_names = []
        
        for frame in frame_list:          
            frame: ami_msg.RefFrame
            
            if isinstance(frame, ami_msg.RefFrame):
                frame_names.append(frame.frame_name)
            else:
                raise ValueError("The list contains an element that is not of type RefFrame.")
            
        return frame_names

    def get_frames_for_planes_of_component(self, component_name: str) -> list[ami_msg.RefFrame]:
        """
        Returns all the frames associated with the planes of the given component.
        parameters:
        - component_name: name of the component to get the frames for
        raises:
        - ComponentNotFoundError: if the component is not found in the scene
        returns:
        - list of ref frames associated with the planes of the component
        """

        frames = []
        
        component = self.get_component_by_name(component_name)
        
        for plane in component.ref_planes:
            plane: ami_msg.Plane
            fr = self.get_frames_for_plane(plane.ref_plane_name)

            frames.extend(fr)
        
        return frames
                    

    def get_parent_frame_for_ref_frame(self, frame_name:str)->str:
        """
        Returns the parent frame of the given ref frame name.
        parameters:
        - frame_name: name of the ref frame to get the parent frame for
        raises:
        - RefFrameNotFoundError: if the ref frame is not found in the scene
        returns:
        - parent frame name of the ref frame
        """
        for obj in self._get_scene().objects_in_scene:
            obj: ami_msg.Object
            for ref_frame in obj.ref_frames:
                ref_frame: ami_msg.RefFrame
                if ref_frame.frame_name == frame_name:
                    return ref_frame.parent_frame
                
        for ref_frame in self._get_scene().ref_frames_in_scene:
            ref_frame: ami_msg.RefFrame
            if ref_frame.frame_name == frame_name:
                return ref_frame.parent_frame    

        raise RefFrameNotFoundError(frame_name)


    def check_ref_frames_for_same_parent_frame(self, frame_names:list[str])->bool:
        """
        This function checks if all given ref frames have the same parent frame.
        If this is the case the function returns True. If not the function returns False.
        Parameters:
        - frame_names: list of strings with the names of the ref frames to check
        Raises:
        - RefFrameNotFoundError: if any of the ref frames is not found in the scene
        Returns:
        - True if all ref frames have the same parent frame, False otherwise
        """
        parent_frame = self.get_parent_frame_for_ref_frame(frame_name=frame_names[0])

        for frame in frame_names:
            _parent_frame = self.get_parent_frame_for_ref_frame(frame_name=frame)
            if parent_frame != _parent_frame:
                return False
        return True


    def check_frames_exist_in_scene(self, frame_names: list[list])->bool:
        """
        This function checks if all given ref frames exist in the scene.
        parameters:
        - frame_names: list of strings with the names of the ref frames to check
        returns:
        - True if all ref frames exist in the scene, False otherwise
        """

        frame_names_copy = copy(frame_names)
        for obj in self._get_scene().objects_in_scene:
            obj: ami_msg.Object
            for frame in obj.ref_frames:
                frame: ami_msg.RefFrame
                if frame.frame_name in frame_names:
                    frame_names_copy.remove(frame.frame_name)

        for frame in self._get_scene().ref_frames_in_scene:
            frame: ami_msg.RefFrame
            if frame.frame_name in frame_names:
                frame_names_copy.remove(frame.frame_name)                       

        if len(frame_names_copy) > 0:
            return False
        else:
            return True

    def check_for_duplicate_frames(self, frame_names: list[str])->bool:
        """
        Check if the given list of frame names contains duplicates.
        parameters:
        - frame_names: list of strings with the names of the ref frames to check
        returns:
        - True if there are duplicates, False otherwise
        """

        if len(frame_names) != len(set(frame_names)):
            return True
        else:
            return False
                

    def get_frame_from_scene(self, frame_name:str)->tuple[Union[str], ami_msg.RefFrame]:
        """
        If associated with an object returns
        (object_name, frame_name)
        If associated with a ref_frame returns
        (None, frame_name)
        parameters:
        - frame_name: name of the ref frame to check
        raises:
        - RefFrameNotFoundError: if the ref frame is not found in the scene
        returns:
        - tuple with object name and frame name or None and frame name
        """

        for obj in self._get_scene().objects_in_scene:
            obj:ami_msg.Object
            
            for ref_frame in obj.ref_frames:
                ref_frame:ami_msg.RefFrame
                if ref_frame.frame_name == frame_name:
                    return (obj.obj_name, ref_frame)

        for ref_frame in self._get_scene().ref_frames_in_scene:
            ref_frame:ami_msg.RefFrame
            if ref_frame.frame_name == frame_name:
                return None, ref_frame
            
        raise RefFrameNotFoundError(frame_name) 

    def is_frame_from_scene(self, frame_name:str)->bool:
        """
        Checks if the given frame name is associated with an object or a ref_frame in the scene.
        parameters:
        - frame_name: name of the frame to check
        returns:
        - True if the frame is found in the scene, False otherwise
        """

        for obj in self._get_scene().objects_in_scene:
            obj:ami_msg.Object

            for ref_frame in obj.ref_frames:
                ref_frame:ami_msg.RefFrame
                if ref_frame.frame_name == frame_name:
                    return True

        for ref_frame in self._get_scene().ref_frames_in_scene:
            ref_frame:ami_msg.RefFrame
            if ref_frame.frame_name == frame_name:
                return True

        return False
    
    def get_parent_of_component(self, component_name: str)->str:
        """
        Returns the parent object of the given component.
        parameters:
        - component_name: name of the component to get the parent for
        raises:
        - ComponentNotFoundError: if the component is not found in the scene
        returns:
        - parent object name of the component
        """
        for obj in self._get_scene().objects_in_scene:
            obj: ami_msg.Object
            if component_name == obj.obj_name:
                return obj.parent_frame
            
        raise ComponentNotFoundError(component_name)


    def get_list_of_components(self) -> list[str]:
        """
        Returns a list of component names in the scene.
        """
        return [obj.obj_name for obj in self._get_scene().objects_in_scene]



    def has_component_parent_of_name(self, parent_frame: str) -> bool:
        """
        Checks if any component in the scene has the given parent frame.
        parameters:
        - parent_frame: name of the parent frame to check
        returns:
        - True if any component has the given parent frame, False otherwise
        """

        for obj in self._get_scene().objects_in_scene:
            obj: ami_msg.Object
            if obj.parent_frame == parent_frame:
                return True  # ✅ Returns True if a matching parent is found
        return False  # ✅ Returns False otherwise


    def get_component_for_frame_name(self, frame_name:str)-> str:
        """
        Returns the component name associated with the given frame name.
        parameters:
        - frame_name: name of the frame to get the component for
        raises:
        - ComponentNotFoundError: if the frame name is not associated with any component in the scene
        - RefFrameNotFoundError: if the frame name is not found in the scene
        returns:
        - component name associated with the frame name
        """
        if not self.is_frame_from_scene(frame_name):
            raise RefFrameNotFoundError(frame_name)
        
        for obj in self._get_scene().objects_in_scene:
            obj:ami_msg.Object
            for frame in obj.ref_frames:
                frame:ami_msg.RefFrame
                if frame.frame_name == frame_name:
                    return obj.obj_name
        
        raise ComponentNotFoundError(frame_name)

    def is_frame_constrained(
        self,
        frame_name: str,
        except_centroid: bool = False,
        except_orthogonal: bool = False,
        except_in_plane: bool = False,
    ) -> bool:
        """
        Checks if the given frame is constrained in the scene.

        Parameters:
        - frame_name: name of the frame to check

        Returns:
        - True if the frame is constrained, False otherwise

        Raises:
        - RefFrameNotFoundError: if the frame name is not found in the scene
        """
        _obj, _frame = self.get_frame_from_scene(frame_name)
        constraints = _frame.constraints

        active_constraints = {
            "centroid": constraints.centroid.is_active,
            "orthogonal": constraints.orthogonal.is_active,
            "in_plane": constraints.in_plane.is_active,
        }

        if except_centroid:
            active_constraints["centroid"] = False
        if except_orthogonal:
            active_constraints["orthogonal"] = False
        if except_in_plane:
            active_constraints["in_plane"] = False

        return any(active_constraints.values())

    def get_property_types_for_frame(self, frame_name: str) -> list[str]:
        """
        Returns a list of active property types for the given frame name.

        Parameters:
        - frame_name: name of the frame to check

        Returns:
        - List of active property types (e.g., ["centroid", "orthogonal", "in_plane"])

        Raises:
        - RefFrameNotFoundError: if the frame name is not found in the scene
        """
        _obj, _frame = self.get_frame_from_scene(frame_name)
        properties = _frame.properties

        active_properties = []
        if properties.glue_pt_frame_properties.is_glue_point:
            active_properties.append("glue_point")
        if properties.gripping_frame_properties.is_gripping_frame:
            active_properties.append("gripping")
        if properties.laser_frame_properties.is_laser_frame:
            active_properties.append("laser")
        if properties.vision_frame_properties.is_vision_frame:
            active_properties.append("vision")

        return active_properties
    
    def check_object_exists(self, object_name:str)-> bool:
        """
        DEPRICATED! Use 'check_components_exists' instead.
        Checks if the given object exists in the scene.
        parameters:
        - object_name: name of the object to check
        returns:
        - True if the object exists in the scene, False otherwise
        """
        for obj in self._get_scene().objects_in_scene:
            obj:ami_msg.Object
            if obj.obj_name == object_name:
                return True
        return False
    
    def check_component_exists(self, component_name:str)-> bool:
        """
        Checks if the given component exists in the scene.
        parameters:
        - component_name: name of the component to check
        returns:
        - True if the component exists in the scene, False otherwise
        """
        for obj in self._get_scene().objects_in_scene:
            obj:ami_msg.Object
            if obj.obj_name == component_name:
                return True
        return False



    def is_component_stationary(self, component_name:str)-> bool:
        """
        Checks if the given component is stationary in the assembly instructions.
        A component is considered stationary if it is never a moving part in any assembly instruction.
        parameters:
        - component_name: name of the component to check
        raises:
        - ComponentNotFoundError: if the component is not found in the scene
        returns:
        - True if the component is stationary, False otherwise
        """
        # check if component exists
        self.get_component_by_name(component_name)

        is_stationary = True
        for instruction in self._get_scene().assembly_instructions:
            instruction:ami_msg.AssemblyInstruction
            if component_name == instruction.component_1 and instruction.component_1_is_moving_part:
                is_stationary = False

            if component_name == instruction.component_2 and not instruction.component_1_is_moving_part:
                is_stationary = False

        return is_stationary

    def is_component_assembled(self, component_name:str)-> bool:
        """
        Checks if the given component is assembled in the assembly instructions.
        A component is considered assembled if it is a part of any assembly instruction.
        parameters:
        - component_name: name of the component to check
        raises:
        - ComponentNotFoundError: if the component is not found in the scene
        returns:
        - True if the component is assembled, False otherwise
        """

        for instruction in self._get_scene().assembly_instructions:
            instruction:ami_msg.AssemblyInstruction
            parent = self.get_parent_of_component(self._get_scene(), component_name)

            if (component_name == instruction.component_1 and instruction.component_2 == parent) or \
            (component_name == instruction.component_2 and instruction.component_1 == parent):
                return True
        
        return False



    def find_matches_for_component(self, component_name:str, only_unassembled = True) -> list[str]:
        """
        Finds all components that match the given component name in the assembly instructions.
        parameters:
        - component_name: name of the component to find matches for
        - only_unassembled: if True, only unassembled components are considered
        raises:
        - ComponentNotFoundError: if the component is not found in the scene
        returns:
        - list of component names that match the given component name
        """

        # check if component exists
        self.get_component_by_name(component_name)

        matches = []
        for instruction in self._get_scene().assembly_instructions:
            instruction:ami_msg.AssemblyInstruction

                # if self.logger is not None:
                #     self.logger.warn(f"Instruction: {instruction.component_1} -> {instruction.component_2}")

            test = self.is_component_assembled(instruction.component_2)

                # if self.logger is not None:
                #     self.logger.warn(f"Is component {instruction.component_2} assembled: {test}")

            if component_name == instruction.component_1:
                if ((not only_unassembled or self.is_component_assembled(instruction.component_2)) and 
                    not instruction.component_1_is_moving_part):
                    #not instruction.component_2 == self.get_global_statonary_component()):

                    matches.append(instruction.component_2)

            if component_name == instruction.component_2:
                if ((not only_unassembled or self.is_component_assembled(instruction.component_1)) and 
                    instruction.component_1_is_moving_part):
                    #not instruction.component_1 == self.get_global_statonary_component()):

                    matches.append(instruction.component_1)
        return matches

    def get_global_stationary_component(self)-> str:
        """
        Returns the name of the global stationary component in the scene.
        The global stationary component is the first component found that is stationary.
        returns:
        - name of the global stationary component or None if no stationary component is found
        """

        components = self.get_list_of_components()
        
        for component in components:
            if self.is_component_stationary(component):
                return component
            
        return None
    


    def get_components_to_assemble(self)-> list[str]:
        """
        Returns a list of components that need to be assembled.
        A component needs to be assembled if it is a moving part in any assembly instruction and its
        stationary part is not its parent.
        returns:
        - list of component names that need to be assembled
        """

        components_to_assembly = []

        for instruction in self._get_scene().assembly_instructions:
            instruction:ami_msg.AssemblyInstruction
            if instruction.component_1_is_moving_part:
                moving_object = instruction.component_1
                stationary_object = instruction.component_2
            else:
                moving_object = instruction.component_2
                stationary_object = instruction.component_1
            
            parent = self.get_parent_of_component(component_name=moving_object)

            if stationary_object != parent:
                components_to_assembly.append(moving_object)
        
        return components_to_assembly


    # rework!
    def get_all_assembly_and_target_frames(self)-> list[tuple[str,str]]:
        """
        Returns a list of tuples with the object name and the frame name for all assembly and target frames in the scene.
        Each tuple contains (object_name, frame_name).
        returns:
        - list of tuples with object name and frame name for all assembly and target frames in the scene
        """
        frames = []
        for obj in self._get_scene().objects_in_scene:
            obj:ami_msg.Object
            for frame in obj.ref_frames:
                frame:ami_msg.RefFrame
                if AssemblyConstants.ASSEMBLY_FRAME_INDICATOR in frame.frame_name:
                    frames.append((obj.obj_name,frame.frame_name))
                if AssemblyConstants.TARGET_FRAME_INDICATOR in frame.frame_name:
                    frames.append((obj.obj_name,frame.frame_name))
        return frames

    # rework!
    def get_all_assembly_and_target_frames_for_component(self, component_name:str)-> list[str]:
        """
        Returns a list of assembly and target frame names for the given component name.
        parameters:
        - component_name: name of the component to get the frames for
        raises:
        - ComponentNotFoundError: if the component is not found in the scene
        returns:
        - list of assembly and target frame names for the given component
        """

        # check if component exists
        self.get_component_by_name(component_name)

        frames_tuple = self.get_all_assembly_and_target_frames()

        frames = []

        for frame_tuple in frames_tuple:
            if frame_tuple[0] == component_name:
                frames.append(frame_tuple[1])

        return frames

    # def get_target_frame_for_component(self, component_name:str)-> str:
    #     """
    #     Returns the target frame name for the given component name.
    #     That means the target frame to which the component should be assembled.
    #     This should be unique as a component can only be assembled once.
    #     parameters:
    #     - component_name: name of the component to get the target frame for
    #     raises:
    #     - ComponentNotFoundError: if the component is not found in the scene
    #     returns:
    #     - target frame name for the given component or None if no target frame is found
    #     """

    #     target_frame = None

    #     component_t_a_frames = self.get_all_assembly_and_target_frames_for_component(component_name=component_name)

    #     if len(component_t_a_frames) == 0:
    #         raise AssemblyFrameNotFoundError(component_name)

    #     all_involved_frames_tuples: list[tuple[str,str]] = self.get_all_assembly_and_target_frames()

    #     # iterate over all assembly and target frames of the component
    #     for component_frame in component_t_a_frames:
    #             # if the frame is an assembly frame
    #             if self.ASSEMBLY_FRAME_INDICATOR in component_frame:
    #                 # get the assembly description from the frame name
    #                 # strip indicator from frame name
    #                 assembly_description = component_frame.replace(self.ASSEMBLY_FRAME_INDICATOR, '')

    #                 for frame_tuple in all_involved_frames_tuples:
    #                     if assembly_description in frame_tuple[1] and frame_tuple[0] != component_name:
    #                         target_frame = frame_tuple[1]
    #                         break
        
    #     if target_frame is None:
    #         raise TargetFrameNotFoundError(component_name)

    #     return target_frame

    def get_target_frame_for_component(self, component_name:str)-> str:
        """
        Returns the target frame name for the given component name.
        That means the target frame to which the component should be assembled.
        This should be unique as a component can only be assembled once.
        parameters:
        - component_name: name of the component to get the target frame for
        raises:
        - ComponentNotFoundError: if the component is not found in the scene
        - AssemblyFrameNotFoundError: if no assembly frame is found for the given component
        - TargetFrameNotFoundError: if no target frame is found for the given component
        returns:
        - target frame name for the given component or None if no target frame is found
        """

        # check if component exists
        self.get_component_by_name(component_name)       

        assembly_frame = self.get_assembly_frame_for_component(component_name=component_name)

        target_frame = self.get_target_frame_for_assembly_frame(assembly_frame_name=assembly_frame)

        return target_frame


    # def get_assembly_frame_for_component(self, component_name:str)-> str:
    #     """
    #     Returns the assembly frame name for the given component name.
    #     That means the assembly frame which is used to assemble the component.
    #     This should be unique as a component can only have one assembly frame.
    #     parameters:
    #     - component_name: name of the component to get the assembly frame for
    #     raises:
    #     - ComponentNotFoundError: if the component is not found in the scene
    #     returns:
    #     - assembly frame name for the given component or None if no assembly frame is found
    #     """

    #     component_t_a_frames = self.get_all_assembly_and_target_frames_for_component(component_name=component_name)

    #     if len(component_t_a_frames) == 0:
    #         raise AssemblyFrameNotFoundError(component_name)

    #     all_involved_frames_tuples: list[tuple[str,str]] = self.get_all_assembly_and_target_frames()

    #     # iterate over all assembly and target frames of the component
    #     for component_frame in component_t_a_frames:
    #             # if the frame is an assembly frame
    #             if self.ASSEMBLY_FRAME_INDICATOR in component_frame:
    #                 return component_frame
        
    #     raise AssemblyFrameNotFoundError(component_name)

    def get_assembly_frame_for_component(self, component_name:str)-> str:
        """
        Returns the assembly frame name for the given component name.
        That means the assembly frame which is used to assemble the component.
        This should be unique as a component can only have one assembly frame.
        parameters:
        - component_name: name of the component to get the assembly frame for
        raises:
        - ComponentNotFoundError: if the component is not found in the scene
        - AssemblyFrameNotFoundError: if no assembly frame is found for the given component
        returns:
        - assembly frame name for the given component or None if no assembly frame is found
        """

        # check if component exists
        component = self.get_component_by_name(component_name)       

        assembly_frames = []

        for frame in component.ref_frames:
            frame: ami_msg.RefFrame
            if (frame.properties.assembly_frame_properties.is_assembly_frame == True and 
                frame.properties.assembly_frame_properties.is_target_frame == False):
                assembly_frames.append(frame.frame_name)    

        if len(assembly_frames) > 1:
            raise AssemblyFrameNotFoundError(f"Multiple assembly frames found for component {component_name}: {assembly_frames}. Expected only one assembly frame per component.")    
                
        if len(assembly_frames) == 0:
            raise AssemblyFrameNotFoundError(f"No assembly frame found for component {component_name}.")

        # return the first (and only) assembly frame
        return assembly_frames[0]

    def get_target_frames_of_component(self,component_name:str)-> list[str]:
        """
        Returns a list of the target frame names for the given component name.
        parameters:
        - component_name: name of the component to get the target frames for
        raises:
        - ComponentNotFoundError: if the component is not found in the scene
        returns:
        - list of target frame names for the given component
        """

        # check if component exists
        component = self.get_component_by_name(component_name)       

        target_frames = []

        for frame in component.ref_frames:
            frame: ami_msg.RefFrame
            if (frame.properties.assembly_frame_properties.is_assembly_frame == False and 
                frame.properties.assembly_frame_properties.is_target_frame == True):
                target_frames.append(frame.frame_name)    

        if len(target_frames) == 0:
            raise TargetFrameNotFoundError(component_name)

        return target_frames    
    
    # def get_assembly_frame_for_target_frame(self, target_frame_name:str)-> str:
    #     """
    #     Returns the assembly frame name associated with the given target frame name.
    #     parameters:
    #     - target_frame_name: name of the target frame to get the assembly frame for
    #     raises:
    #     - RefFrameNotFoundError: if the target frame is not found in the scene
    #     - AssemblyFrameNotFoundError: if no assembly frame is found for the given target frame
    #     returns:
    #     - assembly frame name associated with the target frame
    #     """

    #     # check if target frame exists
    #     self.get_ref_frame_by_name(target_frame_name)

    #     all_involved_frames_tuples: list[tuple[str,str]] = self.get_all_assembly_and_target_frames()

    #     assembly_target_frames = [t[1] for t in all_involved_frames_tuples]

    #     assembly_description = target_frame_name.replace(self.TARGET_FRAME_INDICATOR, '')

    #     # iterate over all assembly and target frames of the component
    #     for frame in assembly_target_frames:
    #         if assembly_description in frame and frame != target_frame_name:
    #             return frame

    #     raise AssemblyFrameNotFoundError(target_frame_name)
    


    def get_assembly_frame_for_target_frame(self, target_frame_name:str)-> str:
        
        target_frame = self.get_ref_frame_by_name(target_frame_name)

        if not target_frame.properties.assembly_frame_properties.is_assembly_frame:
            raise TargetFrameNotFoundError(f"The frame {target_frame_name} is not an assembly frame.")
        
        if target_frame.properties.assembly_frame_properties.is_target_frame:
            raise TargetFrameNotFoundError(f"The frame {target_frame_name} is marked as assembly as well as as target frame. This is not allowed!")
        
        associated_assembly_frame = target_frame.properties.assembly_frame_properties.associated_frame

        if associated_assembly_frame is None or associated_assembly_frame == "":
            raise AssemblyFrameNotFoundError(f"The target frame {target_frame_name} does not have an associated assembly frame.")
        
        frame_exists = self.is_frame_from_scene(associated_assembly_frame)
        if not frame_exists:
            raise RefFrameNotFoundError(f"The associated assembly frame {associated_assembly_frame} for target frame {target_frame_name} is not found in the scene.")
        
        assembly_frame = self.get_ref_frame_by_name(associated_assembly_frame)
        
        if not assembly_frame.properties.assembly_frame_properties.is_assembly_frame:
            raise AssemblyFrameNotFoundError(f"The associated frame {associated_assembly_frame} for target frame {target_frame_name} is not marked as assembly frame.")
        
        return associated_assembly_frame

    # def get_target_frame_for_assembly_frame(self, assembly_frame_name:str)-> str:
    #     """
    #     Returns the target frame name associated with the given assembly frame name.
    #     parameters:
    #     - assembly_frame_name: name of the assembly frame to get the target frame for
    #     raises:
    #     - RefFrameNotFoundError: if the target frame is not found in the scene
    #     - AssemblyFrameNotFoundError: if no assembly frame is found for the given target frame
    #     returns:
    #     - assembly frame name associated with the target frame
    #     """

    #     # check if target frame exists
    #     self.get_ref_frame_by_name(assembly_frame_name)

    #     all_involved_frames_tuples: list[tuple[str,str]] = self.get_all_assembly_and_target_frames()

    #     assembly_target_frames = [t[1] for t in all_involved_frames_tuples]

    #     assembly_description = assembly_frame_name.replace(self.ASSEMBLY_FRAME_INDICATOR, '')

    #     # iterate over all assembly and target frames of the component
    #     for frame in assembly_target_frames:
    #         if assembly_description in frame and frame != assembly_frame_name:
    #             return frame

    #     raise AssemblyFrameNotFoundError(assembly_frame_name)

    def get_target_frame_for_assembly_frame(self, assembly_frame_name:str)-> str:
        
        assembly_frame = self.get_ref_frame_by_name(assembly_frame_name)

        if not assembly_frame.properties.assembly_frame_properties.is_assembly_frame:
            raise AssemblyFrameNotFoundError(f"The frame {assembly_frame_name} is not an assembly frame.")
        
        if assembly_frame.properties.assembly_frame_properties.is_target_frame:
            raise AssemblyFrameNotFoundError(f"The frame {assembly_frame_name} is marked as assembly as well as as target frame. This is not allowed!")
        
        associated_target_frame = assembly_frame.properties.assembly_frame_properties.associated_frame

        if associated_target_frame is None or associated_target_frame == "":
            raise TargetFrameNotFoundError(f"The assembly frame {assembly_frame_name} does not have an associated target frame.")
        
        frame_exists = self.is_frame_from_scene(associated_target_frame)
        if not frame_exists:
            raise RefFrameNotFoundError(f"The associated target frame {associated_target_frame} for assembly frame {assembly_frame_name} is not found in the scene.")
        
        target_frame = self.get_ref_frame_by_name(associated_target_frame)
        
        if not target_frame.properties.assembly_frame_properties.is_target_frame:
            raise TargetFrameNotFoundError(f"The associated frame {associated_target_frame} for assembly frame {assembly_frame_name} is not marked as target frame.")
        
        return associated_target_frame


    # def get_target_frames_of_component(self,component_name:str)-> list[str]:
    #     """
    #     Returns a list of the target frame names for the given component name.
    #     parameters:
    #     - component_name: name of the component to get the target frames for
    #     raises:
    #     - ComponentNotFoundError: if the component is not found in the scene
    #     returns:
    #     - list of target frame names for the given component
    #     """

    #     # check if component exists
    #     self.get_component_by_name(component_name)

    #     target_frames = []

    #     component_t_a_frames = self.get_all_assembly_and_target_frames_for_component(component_name=component_name)

    #     if len(component_t_a_frames) == 0:
    #         raise AssemblyFrameNotFoundError(component_name)

    #     all_involved_frames_tuples: list[tuple[str,str]] = self.get_all_assembly_and_target_frames()

    #     # iterate over all assembly and target frames of the component
    #     for component_frame in component_t_a_frames:
    #             # if the frame is an assembly frame
    #             if self.TARGET_FRAME_INDICATOR in component_frame:
    #                 target_frames.append(component_frame)
        
    #     if len(target_frames) == 0:
    #         raise TargetFrameNotFoundError(component_name)

    #     return target_frames

    def get_assembly_instruction(self, assembly_component: str, target_component: str) -> ami_msg.AssemblyInstruction:
        """
        Returns the assembly instruction for the given assembly and target component names.
        parameters:
            - assembly_component: name of the assembly component
            - target_component: name of the target component
        raises:
            - AssemblyInstructionNotFoundError: if no assembly instruction is found for the given components
        returns:
            - assembly instruction for the given components
        """

        for instruction in self._get_scene().assembly_instructions:
            instruction:ami_msg.AssemblyInstruction

            if instruction.component_1_is_moving_part:
                if instruction.component_1 == assembly_component and instruction.component_2 == target_component:
                    return instruction
            else:
                if instruction.component_2 == assembly_component and instruction.component_1 == target_component:
                    return instruction

        raise AssemblyInstructionNotFoundError(assembly_component, target_component)
        
    def get_assembly_instruction_by_name(self, instruction_name: str) -> ami_msg.AssemblyInstruction:
        """
        Returns the assembly instruction for the given instruction name.
        parameters:
            - instruction_name: name of the assembly instruction
        raises:
            - AssemblyInstructionNotFoundError: if no assembly instruction is found for the given name
        returns:
            - assembly instruction for the given name
        """

        for instruction in self._get_scene().assembly_instructions:
            instruction:ami_msg.AssemblyInstruction

            if instruction.id == instruction_name:
                return instruction

        raise AssemblyInstructionNotFoundError(instruction_name)

    def get_gripping_frame_of_component(self, component_name:str)-> str:
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

            if frame.properties.gripping_frame_properties.is_gripping_frame:
                grip_frames.append(frame.frame_name)
                  
        if len(grip_frames) == 0:
            raise GrippingFrameNotFoundError(f"No gripping frame found for component: {component_name}")
        
        if len(grip_frames) > 1:
            raise GrippingFrameNotFoundError(f"Multiple gripping frames found for component: {component_name}")

        return grip_frames[0]

    def get_glue_pt_frames_of_component(self, component_name:str)-> list[ami_msg.RefFrame]:
        """
        Get the glue point frames of a component by searching for known identifiers in its reference frames.
        :param component_name: Name of the component to search for.
        :return: List of names of the glue point frames.
        :raises ComponentNotFoundError: If the component is not found in the scene.
        :raises GluePointFrameNotFoundError: If no glue point frames are found.
        """
        glue_pt_frames: list[ami_msg.RefFrame] = []
        component = self.get_component_by_name(component_name)
        
        for frame in component.ref_frames:
            frame:ami_msg.RefFrame

            if frame.properties.glue_point_frame_properties.is_glue_point_frame:
                glue_pt_frames.append(frame)
                  
        if len(glue_pt_frames) == 0:
            raise GluePointFrameNotFoundError(f"No glue point frames found for component: {component_name}")

        return glue_pt_frames

    def get_vision_frames_of_component(self, component_name:str)-> list[ami_msg.RefFrame]:
        """
        Get the vision frames of a component by searching for known identifiers in its reference frames.
        :param component_name: Name of the component to search for.
        :return: List of names of the vision frames.
        :raises ComponentNotFoundError: If the component is not found in the scene.
        :raises VisionFrameNotFoundError: If no vision frames are found.
        """
        vision_frames: list[ami_msg.RefFrame] = []
        component = self.get_component_by_name(component_name)
        
        for frame in component.ref_frames:
            frame:ami_msg.RefFrame

            if frame.properties.vision_frame_properties.is_vision_frame:
                vision_frames.append(frame)
                  
        if len(vision_frames) == 0:
            raise VisionFrameNotFoundError(f"No vision frames found for component: {component_name}")

        return vision_frames
    

    def get_laser_frames_of_component(self, component_name:str)-> list[ami_msg.RefFrame]:
        """
        Get the laser frames of a component by searching for known identifiers in its reference frames.
        :param component_name: Name of the component to search for.
        :return: List of names of the laser frames.
        :raises ComponentNotFoundError: If the component is not found in the scene.
        :raises LaserFrameNotFoundError: If no laser frames are found.
        """
        laser_frames: list[ami_msg.RefFrame] = []
        component = self.get_component_by_name(component_name)
        
        for frame in component.ref_frames:
            frame:ami_msg.RefFrame

            if frame.properties.laser_frame_properties.is_laser_frame:
                laser_frames.append(frame)
                  
        if len(laser_frames) == 0:
            raise LaserFrameNotFoundError(f"No laser frames found for component: {component_name}")

        return laser_frames
    
    def get_frame_properties(self, frame_name:str)-> ami_msg.RefFrameProperties:
        """
        Returns the properties of the given ref frame name.
        parameters:
        - frame_name: name of the ref frame to get the properties for
        raises:
        - RefFrameNotFoundError: if the ref frame is not found in the scene
        returns:
        - properties of the ref frame
        """
        frame = self.get_ref_frame_by_name(frame_name)
        return frame.properties
    
    def get_frame_properties_copy(self, frame_name:str)-> ami_msg.RefFrameProperties:
        """
        Returns a copy of the properties of the given ref frame name.
        parameters:
        - frame_name: name of the ref frame to get the properties for
        raises:
        - RefFrameNotFoundError: if the ref frame is not found in the scene
        returns:
        - copy of the properties of the ref frame
        """
        frame = self.get_ref_frame_by_name(frame_name)
        return copy(frame.properties)

    def get_constraint_frame_names_for_frame(self,
                                            frame: ami_msg.RefFrame)-> list[str]:
        """
        This function returns a list of frame names that are constraints for the given frame.
        """
        final_list = []
        constraints_handler = FrameConstraintsHandler()
        constraints_handler.set_from_msg(frame.constraints)
        
        ref_frames = constraints_handler.get_frame_references()
        
        if len(ref_frames) > 0:
            final_list.extend(ref_frames)
    
        return final_list

    def get_constraint_frames_for_frame(self,
                                        frame: ami_msg.RefFrame)-> list[ami_msg.RefFrame]:
        """
        This function returns a list of frame names that are constraints for the given frame.
        """
        final_list = []
        constraints_handler = FrameConstraintsHandler()
        constraints_handler.set_from_msg(frame.constraints)
        
        ref_frame_names = constraints_handler.get_frame_references()
        
        if len(ref_frame_names) > 0:
            for ref_frame_name in ref_frame_names:
                ref_frame = self.get_ref_frame_by_name(ref_frame_name)

                final_list.append(ref_frame)
        
        return final_list
    
    def build_frame_reference_tree(self,
                               frames: list[ami_msg.RefFrame]) -> dict:
        """Builds a tree structure representing the references between frames.

        Args:
            frames (list[ami_msg.RefFrame]): The list of frames to build the tree from.

        Raises:
            ValueError: If a cycle is detected in the frame references.

        Returns:
            dict: A dictionary representing the tree structure of frame references.
        """

        tree = {}  # Stores the final tree structure

        def iterate_frames(frames: list[ami_msg.RefFrame], frame_dict: dict, ancestry: set):

            for frame in frames:
            
                #logger.warn(f"HERE List : {str(frame)}")

                if frame.frame_name in ancestry:
                    raise ValueError(f"Cycle detected involving frame {frame.frame_name}")

                frame_references = self.get_constraint_frames_for_frame(frame)

                frame_dict[frame.frame_name] = {}
                # Recurse with an updated ancestry (specific to this path)
                iterate_frames(frame_references, frame_dict[frame.frame_name], ancestry | {frame.frame_name})

        iterate_frames(frames, tree, set())  # Start with an empty ancestry set
        return tree    
    
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
            copy_ref_frame = deepcopy(ref_frame)
            scene_copy = deepcopy(self._get_scene())

            #self.logger.warn(f"Update ref frame constraints for frame '{ref_frame.frame_name}'")
            
            frame_constraints_handler:FrameConstraintsHandler = FrameConstraintsHandler.return_handler_from_msg(msg=copy_ref_frame.constraints,
                                                                                                                scene=scene_copy)
            
            pose = frame_constraints_handler.calculate_frame_constraints(initial_pose = copy_ref_frame.pose, 
                                                                            scene=scene_copy,
                                                                            frame_name=copy_ref_frame.frame_name,
                                                                            component_name = component_name,
                                                                            logger=self.logger)
            
            ref_frame.pose = pose

            return True
        
        except Exception as e:
            self.logger.error(str(e))
            self.logger.error(f"Unknown Error. Constraint could not be updated!")
            return False

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

    # def get_identification_order(scene: ami_msg.ObjectScene,
    #                             frame_list: list[ami_msg.RefFrame],
    #                             logger: RcutilsLogger = None)->ConstraintRestrictionList:
    #     """
    #     Get the vectors and the frame names relevant for the assembly in oder of the identification.

    #     Args:
    #         scene (ami_msg.ObjectScene): _description_
    #         logger (RcutilsLogger, optional): _description_. Defaults to None.

    #     Returns:
    #         ConstraintRestrictionList: _description_
    #     """
        
    #     reference_tree = build_frame_reference_tree(scene, frame_list, logger)
    #     max_depth = get_dict_depth(reference_tree)
    #     calculated_frames=[]
        
    #     if max_depth == 0:
    #         if logger is not None:
    #             logger.warn("222: No frame constraints found in the scene.")
    #         return ConstraintRestrictionList()
        
    #     if logger is not None:
    #         logger.warn(f"reference_tree{str(reference_tree)}")
            
    #     total_list = ConstraintRestrictionList()
    #     for depth in range(max_depth-1, 0, -1):
    #         unique_elements = get_unique_elements_at_depth(reference_tree, depth)
    #         #logger.warn(f"unique_elements{unique_elements}")
            
    #         for frame_name in unique_elements:
                
    #             if frame_name not in calculated_frames:
                    
    #                 frame = get_ref_frame_by_name(scene, frame_name)
    #                 frame_constraints_handler = FrameConstraintsHandler.return_handler_from_msg(frame.constraints, scene=scene, logger=logger)
    #                 test_list = frame_constraints_handler.get_frame_references_const(scene=scene)
    #                 if logger is not None:
    #                     logger.warn(f"Frame: {frame_name} - {str(test_list)}")
    #                 total_list.add_list_to_list(test_list)

    #     return total_list