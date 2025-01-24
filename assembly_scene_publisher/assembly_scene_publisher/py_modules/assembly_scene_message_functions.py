import assembly_manager_interfaces.msg as ami_msg
from rclpy.impl.rcutils_logger import RcutilsLogger
from geometry_msgs.msg import Pose
from copy import deepcopy, copy


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

def get_ref_frame_by_name(frame_name:str, 
                          scene: ami_msg.ObjectScene)->ami_msg.RefFrame:
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
            
          
def get_parent_frame_for_ref_frame(frame_name:str, 
                                    scene: ami_msg.ObjectScene)->str:
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
    parent_frame = get_parent_frame_for_ref_frame(frame_names[0],scene=scene)
    for frame in frame_names:
        if parent_frame != get_parent_frame_for_ref_frame(frame,scene=scene) or parent_frame is None:
            return False
    return True
    