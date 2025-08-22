import assembly_manager_interfaces.srv as ami_srv
import assembly_manager_interfaces.msg as ami_msg
# import ros logger type
# RUtilsLogger
from rclpy.impl.rcutils_logger import RcutilsLogger
from geometry_msgs.msg import Pose
from copy import deepcopy, copy
import sympy as sp
from assembly_scene_publisher.py_modules.geometry_type_functions import rotation_matrix_to_quaternion

from assembly_scene_publisher.py_modules.scene_functions import     (check_frames_exist_in_scene, 
                                                                    check_for_duplicate_frames, 
                                                                    get_ref_frame_by_name,
                                                                    check_ref_frames_for_same_parent_frame,
                                                                    get_parent_frame_for_ref_frame,
                                                                    get_component_by_name,
                                                                    ConstraintRestrictionList)                                    

from assembly_scene_publisher.py_modules.CentroidConstraintHandler import CentroidConstraintHandler
from assembly_scene_publisher.py_modules.OrthogonalConstraintHandler import OrthogonalConstraintHandler
from assembly_scene_publisher.py_modules.InPlaneConstraintHandler import InPlaneConstraintHandler


class FrameConstraintsHandler(ami_msg.FrConstraints):
    
    CALCULATION_ORDER = ['centroid',                 
                         'in_plane',
                         'orthogonal']
    
    def __init__(self, logger: RcutilsLogger = None):
        super(FrameConstraintsHandler, self).__init__()
        self.logger = logger
        self.centroid_handler = CentroidConstraintHandler(logger=logger)
        self.orthogonal_handler = OrthogonalConstraintHandler(logger=logger)
        self.in_plane_handler = InPlaneConstraintHandler(logger=logger)

    @staticmethod
    def return_handler_from_dict(dictionary: dict, 
                                    component_name: str = None,
                                    unique_identifier = '',
                                    scene: ami_msg.ObjectScene = None, 
                                    logger: RcutilsLogger = None):
            
        frame_constraints = FrameConstraintsHandler(logger=logger)
        # init centroid constraint handler
        centroid_handler = CentroidConstraintHandler.return_handler_from_dict(dictionary.get('centroid',{}), 
                                                                                unique_identifier = unique_identifier,
                                                                                component_name=component_name,
                                                                                logger=logger)
        centroid_handler.set_is_active(scene=scene,
                                       component_name=component_name)
        
        frame_constraints.centroid = centroid_handler.return_as_msg()
        frame_constraints.centroid_handler = centroid_handler    
        
        
        # init orthogonal constraint handler
        orthogonal_handler = OrthogonalConstraintHandler.return_handler_from_dict(dictionary.get('orthogonal',{}),
                                                                                unique_identifier = unique_identifier,
                                                                                component_name=component_name,
                                                                                logger=logger)
        
        orthogonal_handler.set_is_active(scene=scene,
                                            component_name=component_name)
        
        frame_constraints.orthogonal = orthogonal_handler.return_as_msg()
        frame_constraints.orthogonal_handler = orthogonal_handler
        
        
        in_plane_handler = InPlaneConstraintHandler.return_handler_from_dict(dictionary.get('inPlane',{}),
                                                                            unique_identifier = unique_identifier,
                                                                            component_name=component_name,
                                                                            logger=logger)
        
        in_plane_handler.set_is_active(scene=scene,
                                       component_name=component_name)
        
        frame_constraints.in_plane = in_plane_handler.return_as_msg()
        frame_constraints.in_plane_handler = in_plane_handler
        
        return frame_constraints
    
    @staticmethod
    def return_handler_from_msg(msg: ami_msg.FrConstraints, 
                                scene: ami_msg.ObjectScene = None, 
                                component_name: str = None,
                                logger: RcutilsLogger = None):
        
        handler = FrameConstraintsHandler(logger)
        msg_copy = deepcopy(msg)
        handler.unit = msg_copy.unit
        
        centroid_handler = CentroidConstraintHandler.return_handler_from_msg(msg_copy.centroid, logger)
        centroid_handler.set_is_active(scene,component_name=component_name)
        handler.centroid = centroid_handler.return_as_msg()
        handler.centroid_handler = centroid_handler
        
        orthogonal_handler = OrthogonalConstraintHandler.return_handler_from_msg(msg_copy.orthogonal, logger)
        orthogonal_handler.set_is_active(scene,component_name=component_name)
        handler.orthogonal = orthogonal_handler.return_as_msg()
        handler.orthogonal_handler = orthogonal_handler
        
        in_plane_handler = InPlaneConstraintHandler.return_handler_from_msg(msg_copy.in_plane, logger)
        in_plane_handler.set_is_active(scene,component_name=component_name)
        handler.in_plane = in_plane_handler.return_as_msg()
        handler.in_plane_handler = in_plane_handler
        
        return handler
    
    def return_as_msg(self):
        msg = ami_msg.FrConstraints()
        msg.unit = self.unit
        msg.centroid = self.centroid
        msg.orthogonal = self.orthogonal
        msg.in_plane = self.in_plane
        return msg
    
    def set_from_msg(self, msg: ami_msg.FrConstraints):
        msg_copy = deepcopy(msg)
        self.unit = msg_copy.unit
        self.centroid = msg_copy.centroid
        self.centroid_handler = CentroidConstraintHandler.return_handler_from_msg(msg_copy.centroid, self.logger)
        self.orthogonal = msg_copy.orthogonal
        self.orthogonal_handler = OrthogonalConstraintHandler.return_handler_from_msg(msg_copy.orthogonal, self.logger)
        self.in_plane = msg_copy.in_plane
        self.in_plane_handler = InPlaneConstraintHandler.return_handler_from_msg(msg_copy.in_plane, self.logger)
    
    def get_frame_references(self)->list[str]:
        # get all the frames used in the constraints for this frame
        frame_references = []
        
        if self.centroid.is_active:
            frame_references.extend(self.centroid_handler.get_frame_references())
        if self.orthogonal.is_active:
            frame_references.extend(self.orthogonal_handler.get_frame_references())
        if self.in_plane.is_active:
            frame_references.extend(self.in_plane_handler.get_frame_references())
            
        return frame_references
    
    def get_frame_references_const(self, scene: ami_msg.ObjectScene)->ConstraintRestrictionList:
        
        ref_list  = ConstraintRestrictionList()
        
        if self.centroid.is_active:
            ref_list.add_list_to_list(self.centroid_handler.get_frame_references_const(scene=scene))
        
        if self.orthogonal.is_active:
            
            ref_list.add_list_to_list(self.orthogonal_handler.get_frame_references_const(scene=scene))
        
        if self.in_plane.is_active:
            
            ref_list.add_list_to_list(self.in_plane_handler.get_frame_references_const(scene=scene))
        
        return ref_list
    
    def calculate_frame_constraints(self, 
                                    initial_pose: Pose,
                                    scene: ami_msg.ObjectScene, 
                                    frame_name: str = None,
                                    component_name: str = None,
                                    logger:RcutilsLogger = None)->Pose:
        """
        This function calculates the frame constraints and returns the pose of the frame.
        """
        result_pose = Pose()
        
        for constraint in self.CALCULATION_ORDER:
            if constraint == 'centroid':
                # calculate the centroid constraint
                centroid_handler = CentroidConstraintHandler(logger=self.logger)
                centroid_handler.set_from_msg(self.centroid)
                initial_pose = centroid_handler.calculate_constraint(initial_frame_pose=initial_pose,
                                                                    scene=scene,
                                                                    component_name=component_name,
                                                                    unit=self.unit,
                                                                    frame_name=frame_name)
            if constraint == 'in_plane':
                # calculate the in-plane constraint
                in_plane_handler = InPlaneConstraintHandler(logger=self.logger)
                in_plane_handler.set_from_msg(self.in_plane)
                initial_pose = in_plane_handler.calculate_constraint(initial_frame_pose=initial_pose,
                                                                    scene=scene,
                                                                    component_name=component_name,
                                                                    unit=self.unit,
                                                                    frame_name=frame_name)

            if constraint == 'orthogonal':
                # calculate the orthogonal constraint
                orthogonal_handler = OrthogonalConstraintHandler(logger=self.logger)
                orthogonal_handler.set_from_msg(self.orthogonal)
                initial_pose = orthogonal_handler.calculate_constraint(initial_frame_pose=initial_pose,
                                                                    scene=scene,
                                                                    component_name=component_name,
                                                                    unit=self.unit,
                                                                    frame_name=frame_name)
        return initial_pose
    
    
def update_ref_frame_by_constraint(scene:ami_msg.ObjectScene, 
                                ref_frame:ami_msg.RefFrame, 
                                component_name:str,
                                logger: RcutilsLogger = None
                                )-> bool:
    try:
        # make a deep copy just to be sure that the original ref frame is not modified
        # this is just for safety reasons
        copy_ref_frame = deepcopy(ref_frame)
        scene_copy = deepcopy(scene)
        
        #self.logger.warn(f"Update ref frame constraints for frame '{ref_frame.frame_name}'")
        
        frame_constraints_handler:FrameConstraintsHandler = FrameConstraintsHandler.return_handler_from_msg(msg=copy_ref_frame.constraints,
                                                                                                            scene=scene_copy,
                                                                                                            logger=logger)
        
        pose = frame_constraints_handler.calculate_frame_constraints(initial_pose = copy_ref_frame.pose, 
                                                                        component_name = component_name,
                                                                        frame_name=copy_ref_frame.frame_name,
                                                                        scene=scene,
                                                                        logger=logger)
        
        ref_frame.pose = pose

        return True
    
    except Exception as e:
        if logger is not None:  
            logger.error(str(e))
            logger.error(f"Unknown Error. Constraint could not be updated!")
        return False

    
def get_constraint_frame_names_for_frame(frame: ami_msg.RefFrame,
                                    logger: RcutilsLogger = None)-> list[str]:
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

def get_constraint_frames_for_frame(scene: ami_msg.ObjectScene,
                                    frame: ami_msg.RefFrame,
                                    logger: RcutilsLogger = None)-> list[ami_msg.RefFrame]:
    """
    This function returns a list of frame names that are constraints for the given frame.
    """
    final_list = []
    constraints_handler = FrameConstraintsHandler()
    constraints_handler.set_from_msg(frame.constraints)
    
    ref_frame_names = constraints_handler.get_frame_references()
    
    if len(ref_frame_names) > 0:
        for ref_frame_name in ref_frame_names:
            ref_frame = get_ref_frame_by_name(scene, ref_frame_name)
            
            if ref_frame is None:
                raise ValueError(f"Error. There is no '{ref_frame_name}' in the assembly scene. However it is used in a constraint definition!")
            
            final_list.append(ref_frame)
    
    return final_list

def build_frame_reference_tree(scene: ami_msg.ObjectScene,
                               frames: list[ami_msg.RefFrame],
                               logger: RcutilsLogger) -> dict:
    
    tree = {}  # Stores the final tree structure

    def iterate_frames(frames: list[ami_msg.RefFrame], frame_dict: dict, ancestry: set):

        for frame in frames:

            #logger.warn(f"HERE List : {str(frame)}")

            if frame.frame_name in ancestry:
                raise ValueError(f"Cycle detected involving frame {frame.frame_name}")

            frame_references = get_constraint_frames_for_frame(scene, frame, logger=logger)

            frame_dict[frame.frame_name] = {}
            # Recurse with an updated ancestry (specific to this path)
            iterate_frames(frame_references, frame_dict[frame.frame_name], ancestry | {frame.frame_name})

    iterate_frames(frames, tree, set())  # Start with an empty ancestry set
    return tree    
    
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

def calculate_frame_contrains_for_frame_list(scene: ami_msg.ObjectScene,
                                            frame_list: list[ami_msg.RefFrame],
                                            component_name: str = None,
                                            logger: RcutilsLogger = None)-> bool:
    """
    This function calculates the frame constraints for a list of frames.
    """
    reference_tree = build_frame_reference_tree(scene, frame_list, logger)
    max_depth = get_dict_depth(reference_tree)
    calculated_frames=[]

    for depth in range(max_depth, 0, -1):
        unique_elements = get_unique_elements_at_depth(reference_tree, depth)
        #logger.warn(f"unique_elements{unique_elements}")
        for frame_name in unique_elements:
            if frame_name not in calculated_frames:
                
                if logger is not None:
                    #logger.warn(f"Update ref frame constraints for frame '{frame_name}'")
                    pass

                frame = get_ref_frame_by_name(scene, frame_name)
                update_ref_frame_by_constraint(scene, frame, component_name=component_name, logger=logger)
                calculated_frames.append(frame_name)
    return True

def calculate_constraints_for_component(scene: ami_msg.ObjectScene,
                                        component_name: str = None,
                                        logger: RcutilsLogger = None)-> bool:
    """
    This function calculates the frame constraints for a component.
    """
    frame_list = []
    if component_name is None:
        frame_list = scene.ref_frames_in_scene
    else:
        comp = get_component_by_name(scene, component_name, logger=logger)
        if comp is not None:
            frame_list = comp.ref_frames

    if logger is not None:
        #logger.warn(f"list:{frame_list}")
        pass

    return calculate_frame_contrains_for_frame_list(scene, frame_list, component_name=component_name, logger=logger)


def calculate_constraints_for_scene(scene: ami_msg.ObjectScene,
                                    logger: RcutilsLogger = None)-> bool:
    """
    This function calculates the frame constraints for the whole scene.
    """
    #logger.warn(f"initial_scene{str(scene)}")
    # calculate the constraints for all components in the scene
    for component in scene.objects_in_scene:
        component: ami_msg.Object
        calculate_constraints_for_component(scene, component.obj_name, logger=logger)

    # calculate the constraints for frames not associated with any component
    calculate_constraints_for_component(scene, logger=logger)

    #logger.warn(f"final_scene{str(scene)}")
    return True

def get_identification_order(scene: ami_msg.ObjectScene,
                            frame_list: list[ami_msg.RefFrame],
                            logger: RcutilsLogger = None)->ConstraintRestrictionList:
    """
    Get the vectors and the frame names relevant for the assembly in oder of the identification.

    Args:
        scene (ami_msg.ObjectScene): _description_
        logger (RcutilsLogger, optional): _description_. Defaults to None.

    Returns:
        ConstraintRestrictionList: _description_
    """
    
    reference_tree = build_frame_reference_tree(scene, frame_list, logger)
    max_depth = get_dict_depth(reference_tree)
    calculated_frames=[]
    
    if max_depth == 0:
        if logger is not None:
            logger.warn("222: No frame constraints found in the scene.")
        return ConstraintRestrictionList()
    
    if logger is not None:
        logger.warn(f"reference_tree{str(reference_tree)}")
        
    total_list = ConstraintRestrictionList()
    for depth in range(max_depth-1, 0, -1):
        unique_elements = get_unique_elements_at_depth(reference_tree, depth)
        #logger.warn(f"unique_elements{unique_elements}")
        
        for frame_name in unique_elements:
            
            if frame_name not in calculated_frames:
                
                frame = get_ref_frame_by_name(scene, frame_name)
                frame_constraints_handler = FrameConstraintsHandler.return_handler_from_msg(frame.constraints, scene=scene, logger=logger)
                test_list = frame_constraints_handler.get_frame_references_const(scene=scene)
                if logger is not None:
                    logger.warn(f"Frame: {frame_name} - {str(test_list)}")
                total_list.add_list_to_list(test_list)

    return total_list
