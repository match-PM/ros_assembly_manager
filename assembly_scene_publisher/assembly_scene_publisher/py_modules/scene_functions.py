
import assembly_manager_interfaces.msg as ami_msg
from rclpy.impl.rcutils_logger import RcutilsLogger
from geometry_msgs.msg import Pose
from copy import deepcopy, copy

from typing import Union

from geometry_msgs.msg import Vector3

class ConstraintRestriction:
    def __init__(self, frame_name: str, constraining_vectors: list[Vector3], vector_directions: list[str]):
        self.frame_name = frame_name
        self.constraining_vectors: list[Vector3] = constraining_vectors
        self.vector_directions: list[str]= vector_directions

class ConstraintRestrictionList:
    def __init__(self):
        self._restrictions: list[ConstraintRestriction] = []

    def get_list(self)->list[ConstraintRestriction]:
        return self._restrictions
    
    def add_entry(self, restriction: ConstraintRestriction):
        self._restrictions.append(restriction)

class AssemblyConstants:
    ASSEMBLY_FRAME_INDICATOR = 'assembly_frame_Description'
    TARGET_FRAME_INDICATOR = 'target_frame_Description'

def get_frames_for_planes_of_component(scene: ami_msg.ObjectScene, 
                             component_name: str,
                             logger: RcutilsLogger = None)-> list[ami_msg.RefFrame]:

    frames = []
    
    component = get_component_by_name(scene, component_name)
    
    if component is None:
        return []
    
    for plane in component.ref_planes:
        plane: ami_msg.Plane
        fr = get_frames_for_plane(scene, plane.ref_plane_name, component_name, logger=logger)
                    
        if fr is not None:
            frames.extend(fr)
    
    #frames_str = get_frame_names_from_list(frames)
    
    return frames
                

def get_frames_for_axis(scene: ami_msg.ObjectScene, 
                            axis_name: str, 
                            component_name: str,
                            logger: RcutilsLogger = None)-> list[ami_msg.RefFrame]:

    axis = get_axis_from_scene(scene, axis_name)
    
    if axis is None:
        return None
    
    frame_list = []
    for point_name in axis.point_names:
        ref_frame = get_ref_frame_by_name(scene, point_name)
        if ref_frame is not None:
            frame_list.append(ref_frame)

    return frame_list

def get_frame_names_from_list(  frame_list: list[ami_msg.RefFrame],
                                logger: RcutilsLogger = None)-> list[str]:
    frame_names = []
    
    for frame in frame_list:
        
        if logger is not None:
            logger.error(f'frame_list: {frame}')
        
        frame: ami_msg.RefFrame
        
        if isinstance(frame, ami_msg.RefFrame):
            frame_names.append(frame.frame_name)
        
    return frame_names

def get_component_by_name(  scene: ami_msg.ObjectScene, 
                            obj_name:str,
                            logger: RcutilsLogger = None) -> ami_msg.Object:
    """
    Returns the object from the objects list by the given obj name
    """
    for obj in scene.objects_in_scene:
        obj:ami_msg.Object
        if obj.obj_name == obj_name:
            return obj
    return None
    
def get_frames_for_plane(   scene: ami_msg.ObjectScene, 
                            plane_name: str, 
                            component_name: str,
                            logger: RcutilsLogger = None)-> list[ami_msg.RefFrame]:

    plane = get_plane_from_scene(scene, plane_name)
    
    if plane is None:
        return None
    
    frame_list = []
    axis_name = plane.axis_names[0]
    
    if axis_name != '':
        list_ax = get_frames_for_axis(scene, axis_name, component_name)
        
        if list_ax is not None:
            frame_list.extend(list_ax)
    
    for point_name in plane.point_names:
        frame = get_ref_frame_by_name(scene, point_name)
        if isinstance(frame, ami_msg.RefFrame):
            frame_list.append(frame)
            
    return frame_list


def get_plane_from_scene(   scene: ami_msg.ObjectScene, 
                            plane_name:str,
                            logger: RcutilsLogger = None)-> ami_msg.Plane:
    plane_msg = None
    for plane in scene.planes_in_scene:
        plane: ami_msg.Plane
        if plane_name == plane.ref_plane_name:
            plane_msg = plane
            break

    for obj in scene.objects_in_scene:
        obj:ami_msg.Object
        for plane in obj.ref_planes:
            plane: ami_msg.Plane
            if plane_name == plane.ref_plane_name:
                plane_msg = plane
                break

    return plane_msg
    
def get_axis_from_scene(scene: ami_msg.ObjectScene, 
                        axis_name:str,
                        logger: RcutilsLogger = None)-> ami_msg.Axis:
    axis_msg = None
    
    for axis in scene.axis_in_scene:
        axis: ami_msg.Axis
        if axis_name == axis.axis_name:
            axis_msg = axis
            break

    for obj in scene.objects_in_scene:
        obj:ami_msg.Object
        for axis in obj.ref_axis:
            axis: ami_msg.Axis
            if axis_name == axis.axis_name:
                axis_msg = axis
                break
    return axis_msg


def get_ref_frame_by_name(  scene: ami_msg.ObjectScene, 
                            frame_name:str,
                            logger: RcutilsLogger = None) -> ami_msg.RefFrame:
    """
    Returns the ref frame from the ref frames list by the given frame name.
    """
    for obj in scene.objects_in_scene:
        obj: ami_msg.Object
        for ref_frame in obj.ref_frames:
            ref_frame:ami_msg.RefFrame
            if ref_frame.frame_name == frame_name:
                return ref_frame
            
    for ref_frame in scene.ref_frames_in_scene:
        ref_frame:ami_msg.RefFrame
        if ref_frame.frame_name == frame_name:
            return ref_frame
    return None


def get_ref_frame_poses_by_names(   scene: ami_msg.ObjectScene,
                                    frame_names: list[str],
                                    logger: RcutilsLogger = None)-> list[Pose]:
    
    poses = []
    
    for frame_name in frame_names:
        frame = get_ref_frame_by_name(scene, frame_name)
        if frame is not None:
            poses.append(frame.pose)
            
    return poses
        
def get_parent_frame_for_ref_frame( scene: ami_msg.ObjectScene,
                                    frame_name:str,
                                    logger: RcutilsLogger = None)->str:
    """
    This function returns the parent frame for the given ref frame. 
    If the ref frame does not exist the function returns None.
    """
    for obj in scene.objects_in_scene:
        obj: ami_msg.Object
        for ref_frame in obj.ref_frames:
            ref_frame: ami_msg.RefFrame
            if ref_frame.frame_name == frame_name:
                return ref_frame.parent_frame
            
    for ref_frame in scene.ref_frames_in_scene:
        ref_frame: ami_msg.RefFrame
        if ref_frame.frame_name == frame_name:
            return ref_frame.parent_frame    

    return None 


def check_ref_frames_for_same_parent_frame(frame_names:list[str], 
                                            scene: ami_msg.ObjectScene)->bool:
    """
    This function checks if all given ref frames have the same parent frame.
    If this is the case the function returns True. If not the function returns False.
    Parameters:
    - frame_names: list of strings with the names of the ref frames to check
    """
    parent_frame = get_parent_frame_for_ref_frame(scene=scene, frame_name=frame_names[0])
    for frame in frame_names:
        if parent_frame != get_parent_frame_for_ref_frame(scene=scene, frame_name=frame) or parent_frame is None:
            return False
    return True

def check_frames_exist_in_scene(frame_names: list[list], 
                                scene: ami_msg.ObjectScene, 
                                logger: RcutilsLogger = None):
    # delete from the list the frames that do not exist
    frame_names_copy = copy(frame_names)
    for obj in scene.objects_in_scene:
        obj: ami_msg.Object
        for frame in obj.ref_frames:
            frame: ami_msg.RefFrame
            if frame.frame_name in frame_names:
                frame_names_copy.remove(frame.frame_name)
    
    for frame in scene.ref_frames_in_scene:
        frame: ami_msg.RefFrame
        if frame.frame_name in frame_names:
            frame_names_copy.remove(frame.frame_name)                       

    if len(frame_names_copy) > 0:
        return False
    else:
        return True

def check_for_duplicate_frames(frame_names: list[str])->bool:
    """
    Check if the given list of frame names contains duplicates.

    Args:
        frame_names (list[str]): List of frame names to check for duplicates.

    Returns:
        _type_: True if duplicates are found, False otherwise.
    """
    if len(frame_names) != len(set(frame_names)):
        return True
    else:
        return False
    

def is_frame_from_scene( scene:ami_msg.ObjectScene, frame_name:str)->tuple[Union[str,None], Union[str,None]]:
    """
    If associated with an object returns
    (object_name, frame_name)
    If associated with a ref_frame returns
    (None, frame_name)
    """

    for obj in scene.objects_in_scene:
        obj:ami_msg.Object
        
        for ref_frame in obj.ref_frames:
            ref_frame:ami_msg.RefFrame
            if ref_frame.frame_name == frame_name:
                return (obj.obj_name, ref_frame.frame_name)

    for ref_frame in scene.ref_frames_in_scene:
        ref_frame:ami_msg.RefFrame
        if ref_frame.frame_name == frame_name:
            return None, ref_frame.frame_name
        
    return None, None

def get_parent_of_component(scene: ami_msg.ObjectScene, 
                            component_name: str)->str:
    """
    Returns the parent object of the given component.
    """
    for obj in scene.objects_in_scene:
        obj: ami_msg.Object
        if component_name == obj.obj_name:
            return obj.parent_frame
    return None

def has_component_parent_of_name(scene: ami_msg.ObjectScene, parent_frame: str) -> bool:
    if scene is None:
        raise ValueError("Object scene not available!")

    for obj in scene.objects_in_scene:
        if obj.parent_frame == parent_frame:
            return True  # ✅ Returns True if a matching parent is found
    return False  # ✅ Returns False otherwise

def is_component_assembled(scene: ami_msg.ObjectScene, 
                        component_name:str)-> bool:

    for instruction in scene.assembly_instructions:
        instruction:ami_msg.AssemblyInstruction
        parent = get_parent_of_component(scene, component_name)
        if parent is not None:
            if (component_name == instruction.component_1 and instruction.component_2 == parent) or \
            (component_name == instruction.component_2 and instruction.component_1 == parent):
                return True
    
    return False

def get_components_to_assemble(scene: ami_msg.ObjectScene)-> list[str]:
    components_to_assembly = []

    for instruction in scene.assembly_instructions:
        instruction:ami_msg.AssemblyInstruction
        if instruction.component_1_is_moving_part:
            moving_object = instruction.component_1
            stationary_object = instruction.component_2
        else:
            moving_object = instruction.component_2
            stationary_object = instruction.component_1
        
        parent = get_parent_of_component(scene=scene, component_name=moving_object)
        if parent is None or stationary_object != parent:

            components_to_assembly.append(moving_object)
    
    return components_to_assembly

def get_component_for_frame_name(scene: ami_msg.ObjectScene, 
                                frame_name:str)-> str:
    """
    This function returns the component name for the given frame name.
    If the frame name is not associated with a component the function returns None.
    """
    for obj in scene.objects_in_scene:
        obj:ami_msg.Object
        for frame in obj.ref_frames:
            frame:ami_msg.RefFrame
            if frame.frame_name == frame_name:
                return obj.obj_name
    return None

def check_object_exists(scene: ami_msg.ObjectScene, 
                        object_name:str)-> bool:
    
    if scene is None:
        return False
    
    for obj in scene.objects_in_scene:
        obj:ami_msg.Object
        if obj.obj_name == object_name:
            return True
    return False

def get_assembly_and_target_frames(scene: ami_msg.ObjectScene)-> list[tuple[str,str]]:
    frames = []
    for obj in scene.objects_in_scene:
        obj:ami_msg.Object
        for frame in obj.ref_frames:
            frame:ami_msg.RefFrame
            if AssemblyConstants.ASSEMBLY_FRAME_INDICATOR in frame.frame_name:
                frames.append((obj.obj_name,frame.frame_name))
            if AssemblyConstants.TARGET_FRAME_INDICATOR in frame.frame_name:
                frames.append((obj.obj_name,frame.frame_name))
    return frames

def get_list_of_components(scene: ami_msg.ObjectScene) -> list[str]:
    return [obj.obj_name for obj in scene.objects_in_scene]

def is_component_stationary(scene: ami_msg.ObjectScene, 
                            component_name:str)-> bool:

    is_stationary = True
    for instruction in scene.assembly_instructions:
        instruction:ami_msg.AssemblyInstruction
        if component_name == instruction.component_1 and instruction.component_1_is_moving_part:
            is_stationary = False

        if component_name == instruction.component_2 and not instruction.component_1_is_moving_part:
            is_stationary = False

    return is_stationary

def get_global_statonary_component(scene: ami_msg.ObjectScene)-> str:
    """
    Returns the name of the component that is not moved among all of the components.
    """

    components = get_list_of_components(scene=scene)
    
    for component in components:
        if is_component_stationary(scene,
                                component):
            return component
        
    return None


def find_matches_for_component(scene: ami_msg.ObjectScene, 
                            component_name:str, 
                            only_unassembled = True,
                            logger: RcutilsLogger = None)-> list[str]:
    """
    The component to find the matches for should be the stationary component.
    """

    matches = []
    for instruction in scene.assembly_instructions:
        instruction:ami_msg.AssemblyInstruction

        if logger is not None:
            logger.warn(f"Instruction: {instruction.component_1} -> {instruction.component_2}")

        test = is_component_assembled(scene, instruction.component_2)

        if logger is not None:
            logger.warn(f"Is component {instruction.component_2} assembled: {test}")

        if component_name == instruction.component_1:
            if ((not only_unassembled or is_component_assembled(scene, instruction.component_2)) and 
                not instruction.component_1_is_moving_part):
                #not instruction.component_2 == self.get_global_statonary_component()):

                matches.append(instruction.component_2)

        if component_name == instruction.component_2:
            if ((not only_unassembled or is_component_assembled(scene, instruction.component_1)) and 
                instruction.component_1_is_moving_part):
                #not instruction.component_1 == self.get_global_statonary_component()):

                matches.append(instruction.component_1)
    return matches
