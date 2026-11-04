import assembly_manager_interfaces.srv as ami_srv
import assembly_manager_interfaces.msg as ami_msg
# import ros logger type
# RUtilsLogger
from rclpy.impl.rcutils_logger import RcutilsLogger
from geometry_msgs.msg import Pose
from copy import deepcopy, copy
import sympy as sp
from assembly_scene_publisher.py_modules.geometry_type_functions import rotation_matrix_to_quaternion

from assembly_scene_publisher.py_modules.AssemblySceneAnalyzer import (
    AssemblySceneAnalyzer,
)
from assembly_scene_publisher.py_modules.ConstraintRestriction import (
    ConstraintRestrictionList,
)
from assembly_scene_publisher.py_modules.scene_errors import RefFrameNotFoundError

from assembly_scene_publisher.py_modules.CentroidConstraintHandler import CentroidConstraintHandler
from assembly_scene_publisher.py_modules.OrthogonalConstraintHandler import OrthogonalConstraintHandler
from assembly_scene_publisher.py_modules.InPlaneConstraintHandler import InPlaneConstraintHandler
from assembly_scene_publisher.py_modules.TransformConstraintHandler import TransformConstraintHandler


class FrameConstraintsHandler(ami_msg.FrConstraints):
    
    CALCULATION_ORDER = ['centroid', 
                         'transform',                
                         'in_plane',
                         'orthogonal']
    
    def __init__(self, logger: RcutilsLogger = None):
        super(FrameConstraintsHandler, self).__init__()
        self.logger = logger
        self.centroid_handler = CentroidConstraintHandler(logger=logger)
        self.orthogonal_handler = OrthogonalConstraintHandler(logger=logger)
        self.in_plane_handler = InPlaneConstraintHandler(logger=logger)
        self.transform_handler = TransformConstraintHandler(logger=logger)
    
    @staticmethod
    def return_handler_from_dict(
            dictionary: dict,
            component_name: str = None,
            unique_identifier='',
            scene: ami_msg.ObjectScene = None,
            logger: RcutilsLogger = None):
        """Import only constraints present in the exported description."""
        frame_constraints = FrameConstraintsHandler(logger=logger)
        handlers = {
            'centroid': ('centroid', CentroidConstraintHandler),
            'orthogonal': ('orthogonal', OrthogonalConstraintHandler),
            'inPlane': ('in_plane', InPlaneConstraintHandler),
            'transform': ('transform', TransformConstraintHandler),
        }
        unknown = set(dictionary) - set(handlers)
        if unknown:
            raise ValueError(f'Unknown constraint types: {sorted(unknown)}')
        for name, data in dictionary.items():
            if not data:
                continue
            attribute, handler_type = handlers[name]
            handler = handler_type.return_handler_from_dict(
                deepcopy(data),
                component_name=component_name,
                unique_identifier=unique_identifier,
                logger=logger,
            )
            handler.set_is_active(scene=scene, component_name=component_name)
            setattr(frame_constraints, attribute, handler.return_as_msg())
            setattr(frame_constraints, f'{attribute}_handler', handler)
        return frame_constraints

    @staticmethod
    def return_handler_from_msg(msg: ami_msg.FrConstraints,
                                scene: ami_msg.ObjectScene = None, 
                                component_name: str = None,
                                logger: RcutilsLogger = None,
                                validate_activations: bool = True):
        """Create a handler, optionally trusting stored activation flags."""
        
        handler = FrameConstraintsHandler(logger)
        handler.unit = msg.unit
        
        centroid_handler = CentroidConstraintHandler.return_handler_from_msg(msg.centroid, logger)
        if validate_activations:
            centroid_handler.set_is_active(scene, component_name=component_name)
        handler.centroid = centroid_handler.return_as_msg()
        handler.centroid_handler = centroid_handler
        
        orthogonal_handler = OrthogonalConstraintHandler.return_handler_from_msg(msg.orthogonal, logger)
        if validate_activations:
            orthogonal_handler.set_is_active(scene, component_name=component_name)
        handler.orthogonal = orthogonal_handler.return_as_msg()
        handler.orthogonal_handler = orthogonal_handler
        
        in_plane_handler = InPlaneConstraintHandler.return_handler_from_msg(msg.in_plane, logger)
        if validate_activations:
            in_plane_handler.set_is_active(scene, component_name=component_name)
        handler.in_plane = in_plane_handler.return_as_msg()
        handler.in_plane_handler = in_plane_handler

        transform_handler = TransformConstraintHandler.return_handler_from_msg(msg.transform, logger)
        if validate_activations:
            transform_handler.set_is_active(scene, component_name=component_name)
        handler.transform = transform_handler.return_as_msg()
        handler.transform_handler = transform_handler
        
        return handler
    
    def return_as_msg(self):
        msg = ami_msg.FrConstraints()
        msg.unit = self.unit
        msg.centroid = self.centroid
        msg.orthogonal = self.orthogonal
        msg.in_plane = self.in_plane
        msg.transform = self.transform
        return msg
    
    def set_activations(self, scene: ami_msg.ObjectScene, component_name: str = None):
        self.centroid_handler.set_is_active(scene=scene, component_name=component_name)
        self.orthogonal_handler.set_is_active(scene=scene, component_name=component_name)
        self.in_plane_handler.set_is_active(scene=scene, component_name=component_name)
        self.transform_handler.set_is_active(scene=scene, component_name=component_name)   

    def set_from_msg(self, msg: ami_msg.FrConstraints):
        msg_copy = deepcopy(msg)
        self.unit = msg_copy.unit
        self.centroid = msg_copy.centroid
        self.centroid_handler = CentroidConstraintHandler.return_handler_from_msg(msg_copy.centroid, self.logger)
        self.orthogonal = msg_copy.orthogonal
        self.orthogonal_handler = OrthogonalConstraintHandler.return_handler_from_msg(msg_copy.orthogonal, self.logger)
        self.in_plane = msg_copy.in_plane
        self.in_plane_handler = InPlaneConstraintHandler.return_handler_from_msg(msg_copy.in_plane, self.logger)
        self.transform = msg_copy.transform
        self.transform_handler = TransformConstraintHandler.return_handler_from_msg(msg_copy.transform, self.logger)
    
    def get_frame_references(self)->list[str]:
        # get all the frames used in the constraints for this frame
        frame_references = []
        
        if self.centroid.is_active:
            frame_references.extend(self.centroid_handler.get_frame_references())
        if self.orthogonal.is_active:
            frame_references.extend(self.orthogonal_handler.get_frame_references())
        if self.in_plane.is_active:
            frame_references.extend(self.in_plane_handler.get_frame_references())
        if self.transform.is_active:
            frame_references.extend(self.transform_handler.get_frame_references())
            
        return frame_references
    
    def get_frame_references_const(self, scene: ami_msg.ObjectScene)->ConstraintRestrictionList:
        
        ref_list  = ConstraintRestrictionList()
        
        if self.centroid.is_active:
            ref_list.add_list_to_list(self.centroid_handler.get_frame_references_const(scene=scene))
        
        if self.orthogonal.is_active:
            
            ref_list.add_list_to_list(self.orthogonal_handler.get_frame_references_const(scene=scene))
        
        if self.in_plane.is_active:
            
            ref_list.add_list_to_list(self.in_plane_handler.get_frame_references_const(scene=scene))
        
        if self.transform.is_active:
            ref_list.add_list_to_list(self.transform_handler.get_frame_references_const(scene=scene))
        
        return ref_list

    def calculate_frame_constraints(self, 
                                    initial_pose: Pose,
                                    scene: ami_msg.ObjectScene, 
                                    frame_name: str = None,
                                    component_name: str = None,
                                    logger:RcutilsLogger = None,
                                    frame_index: dict = None)->Pose:
        """
        This function calculates the frame constraints and returns the pose of the frame.
        """
        
        for constraint in self.CALCULATION_ORDER:
            if constraint == 'centroid':
                initial_pose = self.centroid_handler.calculate_constraint(initial_frame_pose=initial_pose,
                                                                    scene=scene,
                                                                    component_name=component_name,
                                                                    unit=self.unit,
                                                                    frame_name=frame_name,
                                                                    frame_index=frame_index,
                                                                    validate_activation=False)
            if constraint == 'in_plane':
                initial_pose = self.in_plane_handler.calculate_constraint(initial_frame_pose=initial_pose,
                                                                    scene=scene,
                                                                    component_name=component_name,
                                                                    unit=self.unit,
                                                                    frame_name=frame_name,
                                                                    frame_index=frame_index,
                                                                    validate_activation=False)

            if constraint == 'orthogonal':
                initial_pose = self.orthogonal_handler.calculate_constraint(initial_frame_pose=initial_pose,
                                                                    scene=scene,
                                                                    component_name=component_name,
                                                                    unit=self.unit,
                                                                    frame_name=frame_name,
                                                                    frame_index=frame_index)

            if constraint == 'transform':
                initial_pose = self.transform_handler.calculate_constraint(initial_frame_pose=initial_pose,
                                                                    scene=scene,
                                                                    component_name=component_name,
                                                                    unit=self.unit,
                                                                    frame_name=frame_name,
                                                                    frame_index=frame_index)
        return initial_pose
    
    
    def get_info(self)->str:
        result_str = ''
        if self.centroid is not None and self.centroid_handler.is_active:
            result_str += self.centroid_handler.get_info()
        if self.in_plane is not None and self.in_plane_handler.is_active:
            result_str += self.in_plane_handler.get_info()
        if self.orthogonal is not None and self.orthogonal_handler.is_active:
            result_str += self.orthogonal_handler.get_info()
        if self.transform is not None and self.transform_handler.is_active:
            result_str += self.transform_handler.get_info()
        return result_str

def update_ref_frame_by_constraint(scene:ami_msg.ObjectScene, 
                                ref_frame:ami_msg.RefFrame, 
                                component_name:str = None,
                                logger: RcutilsLogger = None,
                                frame_index: dict = None,
                                validate_activations: bool = True,
                                )-> bool:
    try:
        # Keep failure atomic without copying the frame or the complete scene.
        initial_pose = deepcopy(ref_frame.pose)
        frame_constraints_handler = FrameConstraintsHandler.return_handler_from_msg(
            msg=ref_frame.constraints,
            scene=scene,
            component_name=component_name,
            logger=logger,
            validate_activations=validate_activations,
        )

        pose = frame_constraints_handler.calculate_frame_constraints(
            initial_pose=initial_pose,
            component_name=component_name,
            frame_name=ref_frame.frame_name,
            scene=scene,
            logger=logger,
            frame_index=frame_index,
        )
        
        ref_frame.pose = pose

        return True
    
    except Exception as e:
        if logger is not None:  
            logger.error(str(e))
            logger.error(f"Unknown Error. Constraint could not be updated!")
        return False


### delete here if possible
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

### Delete here if possible
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
            ref_frame = AssemblySceneAnalyzer(
                scene, logger).get_ref_frame_by_name(ref_frame_name)
            final_list.append(ref_frame)
    
    return final_list

### Delete here if possible
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
    
    
def _build_frame_indexes(scene: ami_msg.ObjectScene):
    """Return frame and ownership indexes in deterministic scene order."""
    frames = {}
    owners = {}
    for component in scene.objects_in_scene:
        for frame in component.ref_frames:
            frames[frame.frame_name] = frame
            owners[frame.frame_name] = component.obj_name
    for frame in scene.ref_frames_in_scene:
        frames[frame.frame_name] = frame
        owners[frame.frame_name] = None
    return frames, owners


def _active_constraint_references(frame: ami_msg.RefFrame) -> list[str]:
    """Read dependencies directly from previously validated constraint data."""
    constraints = frame.constraints
    references = []
    if constraints.centroid.is_active:
        references.extend(constraints.centroid.ref_frame_names)
    if constraints.orthogonal.is_active:
        references.extend((constraints.orthogonal.frame_1,
                           constraints.orthogonal.frame_2,
                           constraints.orthogonal.frame_3))
    if constraints.in_plane.is_active:
        references.extend(constraints.in_plane.ref_frame_names)
    if constraints.transform.is_active:
        references.append(constraints.transform.ref_frame)
    return list(dict.fromkeys(name for name in references if name))


def _all_constraint_references(frame: ami_msg.RefFrame) -> list[str]:
    """Read dependencies without requiring their constraints to be active.

    A constraint can be inactive while one of its referenced frames is missing.
    When that frame is later spawned, the inactive constraint is one of the
    constraints that must be revalidated.
    """
    constraints = frame.constraints
    references = []
    references.extend(constraints.centroid.ref_frame_names)
    references.extend((constraints.orthogonal.frame_1,
                       constraints.orthogonal.frame_2,
                       constraints.orthogonal.frame_3))
    references.extend(constraints.in_plane.ref_frame_names)
    references.append(constraints.transform.ref_frame)
    return list(dict.fromkeys(name for name in references if name))


def _build_dependency_graph(frame_index: dict):
    dependencies = {
        name: _active_constraint_references(frame)
        for name, frame in frame_index.items()
    }
    dependents = {name: [] for name in frame_index}
    for dependent, references in dependencies.items():
        for reference in references:
            dependents.setdefault(reference, []).append(dependent)
    return dependencies, dependents


def _build_all_dependency_graph(frame_index: dict):
    """Build a graph that also contains currently inactive constraints."""
    dependencies = {
        name: _all_constraint_references(frame)
        for name, frame in frame_index.items()
    }
    dependents = {name: [] for name in frame_index}
    for dependent, references in dependencies.items():
        for reference in references:
            dependents.setdefault(reference, []).append(dependent)
    return dependencies, dependents


def _dependency_closure(root_names: list[str], dependencies: dict,
                        frame_index: dict) -> set[str]:
    relevant = set()
    pending = list(reversed(root_names))
    while pending:
        name = pending.pop()
        if name in relevant:
            continue
        if name not in frame_index:
            raise RefFrameNotFoundError(name)
        relevant.add(name)
        pending.extend(reversed(dependencies[name]))
    return relevant


def _dependent_closure(frame_name: str, dependents: dict,
                       frame_index: dict) -> set[str]:
    if frame_name not in frame_index:
        raise RefFrameNotFoundError(frame_name)
    affected = set()
    pending = [frame_name]
    while pending:
        name = pending.pop()
        if name in affected:
            continue
        affected.add(name)
        pending.extend(reversed(dependents.get(name, ())))
    return affected


def _topological_order(names: set[str], dependencies: dict) -> list[str]:
    """Order a graph subset dependency-first and detect active cycles."""
    indegree = {
        name: sum(reference in names for reference in dependencies[name])
        for name in names
    }
    dependents = {name: [] for name in names}
    for dependent in dependencies:
        if dependent not in names:
            continue
        for reference in dependencies[dependent]:
            if reference in names:
                dependents[reference].append(dependent)

    ready = [name for name in dependencies
             if name in names and indegree[name] == 0]
    order = []
    cursor = 0
    while cursor < len(ready):
        name = ready[cursor]
        cursor += 1
        order.append(name)
        for dependent in dependents[name]:
            indegree[dependent] -= 1
            if indegree[dependent] == 0:
                ready.append(dependent)

    if len(order) != len(names):
        cyclic = [name for name in dependencies
                  if name in names and indegree[name] > 0]
        raise ValueError(
            f"Cycle detected involving frame(s): {', '.join(cyclic)}")
    return order


def calculate_frame_contrains_for_frame_list(scene: ami_msg.ObjectScene,
                                            frame_list: list[ami_msg.RefFrame],
                                            component_name: str = None,
                                            logger: RcutilsLogger = None)-> bool:
    """Calculate each requested frame and its dependencies exactly once."""
    frame_index, owners = _build_frame_indexes(scene)
    dependencies, _ = _build_dependency_graph(frame_index)
    root_names = [frame.frame_name for frame in frame_list]
    relevant = _dependency_closure(root_names, dependencies, frame_index)
    for frame_name in _topological_order(relevant, dependencies):
        update_ref_frame_by_constraint(
            scene,
            frame_index[frame_name],
            component_name=owners[frame_name],
            logger=logger,
            frame_index=frame_index,
            validate_activations=False,
        )
    return True


def calculate_constraints_for_frame_change(
        scene: ami_msg.ObjectScene,
        frame_name: str,
        logger: RcutilsLogger = None) -> bool:
    """Recalculate only a changed frame and its transitive dependents."""
    frame_index, owners = _build_frame_indexes(scene)
    dependencies, dependents = _build_dependency_graph(frame_index)
    affected = _dependent_closure(frame_name, dependents, frame_index)
    for affected_name in _topological_order(affected, dependencies):
        update_ref_frame_by_constraint(
            scene,
            frame_index[affected_name],
            component_name=owners[affected_name],
            logger=logger,
            frame_index=frame_index,
            validate_activations=False,
        )
    return True


def get_frames_affected_by_changes(scene: ami_msg.ObjectScene,
                                   frame_names: list[str]) -> list[str]:
    """Return changed frames and transitive dependents in scene order.

    Inactive constraints are included so callers can revalidate constraints
    that may become active when new reference frames are added.
    """
    frame_index, _ = _build_frame_indexes(scene)
    _, dependents = _build_all_dependency_graph(frame_index)
    affected = set()
    for frame_name in frame_names:
        affected.update(_dependent_closure(
            frame_name, dependents, frame_index))
    return [name for name in frame_index if name in affected]


def calculate_constraints_for_frame_changes(
        scene: ami_msg.ObjectScene,
        frame_names: list[str],
        logger: RcutilsLogger = None) -> bool:
    """Recalculate multiple changed frames and their dependents once."""
    frame_index, owners = _build_frame_indexes(scene)
    dependencies, dependents = _build_dependency_graph(frame_index)
    affected = set()
    for frame_name in frame_names:
        affected.update(_dependent_closure(
            frame_name, dependents, frame_index))
    for affected_name in _topological_order(affected, dependencies):
        update_ref_frame_by_constraint(
            scene,
            frame_index[affected_name],
            component_name=owners[affected_name],
            logger=logger,
            frame_index=frame_index,
            validate_activations=False,
        )
    return True

### Delete here if possible
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
        comp = AssemblySceneAnalyzer(
            scene, logger).get_component_by_name(component_name)
        frame_list = comp.ref_frames

    if logger is not None:
        #logger.warn(f"list:{frame_list}")
        pass
        
    return calculate_frame_contrains_for_frame_list(scene, frame_list, component_name=component_name, logger=logger)

### Delete here if possible
def calculate_constraints_for_scene(scene: ami_msg.ObjectScene,
                                    logger: RcutilsLogger = None)-> bool:
    """Calculate all scene constraints in one dependency-ordered pass."""
    frame_index, _ = _build_frame_indexes(scene)
    return calculate_frame_contrains_for_frame_list(
        scene, list(frame_index.values()), logger=logger)


def get_dict_depth(d: dict) -> int:
    """Recursively calculate the depth of a legacy reference tree."""
    if not isinstance(d, dict) or not d:
        return 0
    return 1 + max(get_dict_depth(value) for value in d.values())


def get_unique_elements_at_depth(d: dict, depth: int) -> list:
    """Return unique keys at a depth in a legacy reference tree."""
    if depth < 1:
        return []
    if depth == 1:
        return list(set(d.keys()))
    elements = set()
    for value in d.values():
        if isinstance(value, dict):
            elements.update(get_unique_elements_at_depth(value, depth - 1))
    return list(elements)


### Delete here if possible
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
                
                frame = AssemblySceneAnalyzer(
                    scene, logger).get_ref_frame_by_name(frame_name)
                frame_constraints_handler = FrameConstraintsHandler.return_handler_from_msg(frame.constraints, scene=scene, logger=logger)
                test_list = frame_constraints_handler.get_frame_references_const(scene=scene)
                if logger is not None:
                    logger.warn(f"Frame: {frame_name} - {str(test_list)}")
                total_list.add_list_to_list(test_list)

    return total_list
