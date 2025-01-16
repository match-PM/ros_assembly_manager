import assembly_manager_interfaces.srv as ami_srv
import assembly_manager_interfaces.msg as ami_msg
# import ros logger type
# RUtilsLogger
from rclpy.impl.rcutils_logger import RcutilsLogger
from geometry_msgs.msg import Pose
from copy import deepcopy, copy
import sympy as sp
from assembly_scene_publisher.py_modules.geometry_type_functions import rotation_matrix_to_quaternion

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
    
            
            
            





            
class FrameConstraintsHandler(ami_msg.FrConstraints):
    
    CALCULATION_ORDER = ['centroid',                 
                         'in_plane',
                         'orthogonal']
    
    def __init__(self, logger: RcutilsLogger = None):
        super(FrameConstraintsHandler, self).__init__()
        self.logger = logger

    @staticmethod
    def return_handler_from_dict(dictionary: dict, 
                                    component_name: str = None,
                                    scene: ami_msg.ObjectScene = None, 
                                    logger: RcutilsLogger = None):
        
        frame_constraints = FrameConstraintsHandler(logger=logger)
        
        
        # init centroid constraint handler
        centroid_handler = CentroidConstraintHandler.return_handler_from_dict(dictionary.get('centroid',{}), 
                                                                                component_name=component_name,
                                                                                logger=logger)
        centroid_handler.set_is_active(scene=scene,
                                       component_name=component_name)
        
        frame_constraints.centroid = centroid_handler.return_as_msg()
        
        # init orthogonal constraint handler
        orthogonal_handler = OrthogonalConstraintHandler.return_handler_from_dict(dictionary.get('orthogonal',{}),
                                                                                  component_name=component_name,
                                                                                  logger=logger)
        orthogonal_handler.set_is_active(scene=scene,
                                            component_name=component_name)
        
        frame_constraints.orthogonal = orthogonal_handler.return_as_msg()
        
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
        
        orthogonal_handler = OrthogonalConstraintHandler.return_handler_from_msg(msg_copy.orthogonal, logger)
        orthogonal_handler.set_is_active(scene,component_name=component_name)
        handler.orthogonal = orthogonal_handler.return_as_msg()

        return handler
    
    def return_as_msg(self):
        msg = ami_msg.FrConstraints()
        msg.unit = self.unit
        msg.centroid = self.centroid
        msg.orthogonal = self.orthogonal
        return msg
    
    def set_from_msg(self, msg: ami_msg.FrConstraints):
        msg_copy = deepcopy(msg)
        self.unit = msg_copy.unit
        self.centroid = msg_copy.centroid
        self.orthogonal = msg_copy.orthogonal
    
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
                result_pose = centroid_handler.calculate_constraint(initial_frame_pose=initial_pose,
                                                                    scene=scene,
                                                                    component_name=component_name,
                                                                    unit=self.unit,
                                                                    frame_name=frame_name)
            if constraint == 'in_plane':
                pass
                # result_pose = self.calculateconstraint(initial_pose=initial_pose,
                #                                                   scene=scene,
                #                                                   logger=logger)

            if constraint == 'orthogonal':
                # calculate the orthogonal constraint
                orthogonal_handler = OrthogonalConstraintHandler(logger=self.logger)
                orthogonal_handler.set_from_msg(self.orthogonal)
                result_pose = orthogonal_handler.calculate_constraint(initial_frame_pose=initial_pose,
                                                                    scene=scene,
                                                                    component_name=component_name,
                                                                    unit=self.unit,
                                                                    frame_name=frame_name)
        return result_pose
    
    

class OrthogonalConstraintHandler(ami_msg.FrConstraintOrthogonal):
    def __init__(self, logger: RcutilsLogger = None):
        super(OrthogonalConstraintHandler, self).__init__()
        self.logger = logger
    
    def return_as_msg(self, logger: RcutilsLogger = None):
        msg = ami_msg.FrConstraintOrthogonal()
        msg.is_active = self.is_active
        msg.frame_1 = self.frame_1
        msg.frame_2 = self.frame_2
        msg.frame_3 = self.frame_3
        msg.distance_from_f1 = self.distance_from_f1
        msg.distance_from_f1_f2_connection = self.distance_from_f1_f2_connection
        msg.unit_distance_from_f1 = self.unit_distance_from_f1
        msg.frame_orthogonal_connection_axis = self.frame_orthogonal_connection_axis
        msg.frame_normal_plane_axis = self.frame_normal_plane_axis
        return msg

    def set_from_msg(self, msg: ami_msg.FrConstraintOrthogonal):
        msg_copy = deepcopy(msg)
        self.is_active = msg_copy.is_active
        self.frame_1 = msg_copy.frame_1
        self.frame_2 = msg_copy.frame_2
        self.frame_3 = msg_copy.frame_3
        self.distance_from_f1 = msg_copy.distance_from_f1
        self.distance_from_f1_f2_connection = msg_copy.distance_from_f1_f2_connection
        self.unit_distance_from_f1 = msg_copy.unit_distance_from_f1
        self.frame_orthogonal_connection_axis = msg_copy.frame_orthogonal_connection_axis
        self.frame_normal_plane_axis = msg_copy.frame_normal_plane_axis
        

    def set_is_active(self, scene: ami_msg.ObjectScene = None, component_name = None):
        
        # check if enough reference frames are provided
        if self.frame_1 != '' and self.frame_2 != '' and self.frame_3 != '':
            self.is_active = True
            if self.logger is not None:
                self.logger.debug(f'Enough reference frames provided for orthogonal constraint {self.frame_1}, {self.frame_2}, {self.frame_3}.')
        else:
            self.is_active = False
            if self.logger is not None:
                pass
                #self.logger.error(f'Not enough reference frames provided for orthogonal constraint{self.ref_frame_names}. ')
            return
        
        # check if the dimension is valid
        if self.frame_orthogonal_connection_axis == self.frame_normal_plane_axis:
            self.is_active = False
            if self.logger is not None:
                self.logger.error('The orthogonal connection axis and the normal plane axis must be different.')
            return
        
        axis_vector = ['x','y','z']
        
        if not (self.frame_orthogonal_connection_axis in axis_vector and self.frame_normal_plane_axis in axis_vector):
            self.is_active = False
            if self.logger is not None:
                self.logger.error('Invalid axis provided for orthogonal constraint')
            return
        
        if scene is not None and not check_frames_exist_in_scene(frame_names=[self.frame_1,self.frame_2,self.frame_3],
                                                                    scene=scene,
                                                                    logger=self.logger):
            self.is_active = False
            if self.logger is not None:
                self.logger.error(f'Reference frames "{self.frame_1}, {self.frame_2}, {self.frame_3}" not found in the scene!')
        
        
        if check_for_duplicate_frames([self.frame_1,self.frame_2,self.frame_3]):
            self.is_active = False
            if self.logger is not None:
                self.logger.error('Duplicate reference frames provided for orthogonal constraint')
            return
    
    def calculate_constraint(self,
                            initial_frame_pose:Pose,
                            scene: ami_msg.ObjectScene,
                            unit: str = 'mm',
                            frame_name: str = None,
                            component_name: str = None) -> Pose:
        """_summary_

        Args:
            initial_frame_pose (Pose): _description_
            scene (ami_msg.ObjectScene): _description_
            component_name (str, optional): _description_. Defaults to None.
            logger (RcutilsLogger, optional): _description_. Defaults to None.

        Returns:
            Pose: _description_
        """
        if self.is_active == False:
            self.logger.debug('Orthogonal constraint is not active')
            return initial_frame_pose
        
        pose_frame_1 = get_ref_frame_by_name(frame_name=self.frame_1,
                                             scene=scene).pose
        
        pose_frame_2 = get_ref_frame_by_name(frame_name=self.frame_2,
                                                scene=scene).pose
        
        pose_frame_3 = get_ref_frame_by_name(frame_name=self.frame_3,
                                                scene=scene).pose
        
        p1 = sp.Matrix([pose_frame_1.position.x, pose_frame_1.position.y, pose_frame_1.position.z])
        p2 = sp.Matrix([pose_frame_2.position.x, pose_frame_2.position.y, pose_frame_2.position.z])
        p3 = sp.Matrix([pose_frame_3.position.x, pose_frame_3.position.y, pose_frame_3.position.z])
        
        multiplier = 1
        if unit == 'mm':
            multiplier = 1000
        if unit == 'um':
            multiplier = 1000000
        if unit == 'm':
            multiplier = 1
        
        connection_vector = p2 - p1
        connection_unit = connection_vector / connection_vector.norm()
        
            # Calculate the normal to the plane formed by frame_1, frame_2, and frame_3
        normal_vector = (p2 - p1).cross(p3 - p1)
        normal_unit = normal_vector / normal_vector.norm()
    
        # Determine the third orthogonal vector
        orthogonal_vector = connection_unit.cross(normal_unit)
        orthogonal_unit = orthogonal_vector / orthogonal_vector.norm()
        
        addition = 0
        if self.unit_distance_from_f1 == '%':
            addition = (connection_vector * self.distance_from_f1 / 100)
        else:
            addition = (connection_unit * self.distance_from_f1 / multiplier)
        self.logger.warn(f'Addition: {addition}')
        # Calculate the position of the constrained frame
        # constrained_position = (
        #     p1 +
        #     connection_unit * self.distance_from_f1 / multiplier +
        #     orthogonal_vector * self._distance_from_f1_f2_connection / multiplier
        # )     
        constrained_position = p1 + addition + orthogonal_unit * self.distance_from_f1_f2_connection / multiplier
        
        # Define axes of the constrained frame
        axes = {
            self.frame_orthogonal_connection_axis: orthogonal_unit,
            self.frame_normal_plane_axis: normal_unit,
            "remaining_axis": connection_unit,
        }
        
        # Ensure the axes are properly assigned
        remaining_axes = {"x", "y", "z"} - {self.frame_orthogonal_connection_axis, self.frame_normal_plane_axis}
        remaining_axis = remaining_axes.pop()
        axes[remaining_axis] = connection_unit
        
        # Construct the rotation matrix for the constrained frame
        rotation_matrix = sp.Matrix.hstack(
            sp.Matrix(axes["x"]),
            sp.Matrix(axes["y"]),
            sp.Matrix(axes["z"]),
        )
        
        # Convert the rotation matrix to a quaternion      
        quat = rotation_matrix_to_quaternion(rotation_matrix)   
    
        self.logger.warn(f'Constrained position: {constrained_position}')
        
        initial_frame_pose.position.x = float(constrained_position[0])
        initial_frame_pose.position.y = float(constrained_position[1])
        initial_frame_pose.position.z = float(constrained_position[2])
        initial_frame_pose.orientation = quat
        
        #initial_frame_pose.position.x = constrained_position[0]
        
        if frame_name is not None:  
            self.logger.warn(f'Start calculating constraint for: {frame_name}')
            
        return initial_frame_pose
    
    @staticmethod  
    def return_handler_from_msg( msg: ami_msg.FrConstraintOrthogonal, logger: RcutilsLogger = None):
        handler = OrthogonalConstraintHandler(logger)
        msg_copy = deepcopy(msg)
        handler.is_active = msg_copy.is_active
        handler.frame_1 = msg_copy.frame_1
        handler.frame_2 = msg_copy.frame_2
        handler.frame_3 = msg_copy.frame_3
        handler.distance_from_f1 = msg_copy.distance_from_f1
        handler.distance_from_f1_f2_connection = msg_copy.distance_from_f1_f2_connection
        handler.unit_distance_from_f1 = msg_copy.unit_distance_from_f1
        handler.frame_orthogonal_connection_axis = msg_copy.frame_orthogonal_connection_axis
        handler.frame_normal_plane_axis = msg_copy.frame_normal_plane_axis
        return handler
    
    @staticmethod
    def return_handler_from_dict(dictionary: dict, 
                                 component_name: str = None, 
                                 logger: RcutilsLogger = None):

        if dictionary == {}:
            return OrthogonalConstraintHandler(logger)

        constraint_handler = OrthogonalConstraintHandler(logger)
        constraint_handler.frame_1 = dictionary.get('frame_1','')
        constraint_handler.frame_2 = dictionary.get('frame_2','')
        constraint_handler.frame_3 = dictionary.get('frame_3','')
        constraint_handler.distance_from_f1 = dictionary.get('distance_from_f1',0.0)
        constraint_handler.distance_from_f1_f2_connection = dictionary.get('distance_from_f1_f2_connection',0.0)
        constraint_handler.unit_distance_from_f1 = dictionary.get('unit_distance_from_f1','%')
        constraint_handler.frame_orthogonal_connection_axis = dictionary.get('frame_orthogonal_connection_axis','x')
        constraint_handler.frame_normal_plane_axis = dictionary.get('frame_normal_plane_axis','z')
                
        if component_name is not None:
            constraint_handler.frame_1 = component_name + '_' + constraint_handler.frame_1
            constraint_handler.frame_2 = component_name + '_' + constraint_handler.frame_2
            constraint_handler.frame_3 = component_name + '_' + constraint_handler.frame_3

        return constraint_handler
        
    
    
    
    
    
    
    
    
    
    
    
class CentroidConstraintHandler(ami_msg.FrConstraintCentroid):
    def __init__(self, logger: RcutilsLogger = None):
        super(CentroidConstraintHandler, self).__init__()
        self.logger = logger
    
    def return_as_msg(self, logger: RcutilsLogger = None):
        msg = ami_msg.FrConstraintCentroid()
        msg.is_active = self.is_active
        msg.dim = self.dim
        msg.offset_values = self.offset_values
        msg.ref_frame_names = self.ref_frame_names
        return msg

    def set_from_msg(self, msg: ami_msg.FrConstraintCentroid):
        msg_copy = deepcopy(msg)
        self.is_active = msg_copy.is_active
        self.dim = msg_copy.dim
        self.offset_values = msg_copy.offset_values
        self.ref_frame_names = msg_copy.ref_frame_names

    def set_is_active(self, scene: ami_msg.ObjectScene = None, component_name = None):
        
        # check if enough reference frames are provided
        if len(self.ref_frame_names)>1:
            self.is_active = True
            if self.logger is not None:
                self.logger.debug(f'Enough reference frames provided for centroid constraint {self.ref_frame_names}.')
        else:
            self.is_active = False
            if self.logger is not None:
                pass
                #self.logger.error(f'Not enough reference frames provided for centroid constraint{self.ref_frame_names}. ')
            return
        
        # check if the dimension is valid
        if self.dim == '':
            self.is_active = False
            if self.logger is not None:
                self.logger.error('Dimension not provided for centroid constraint')
            return
        
        if not ((self.dim == 'xyz' or self.dim == 'xzy' or self.dim == 'yxz' or self.dim == 'yzx' or self.dim == 'zxy' or self.dim == 'zyx')
                or (self.dim == 'x' or self.dim == 'y' or self.dim == 'z')
                or (self.dim == 'xy' or self.dim == 'yx' or self.dim == 'yz' or self.dim == 'zy' or self.dim == 'xz' or self.dim == 'zx')):
            self.is_active = False
            if self.logger is not None:
                self.logger.error('Invalid dimension provided for centroid constraint')
            return
        else:
            self.is_active = True
        
        if check_for_duplicate_frames(self.ref_frame_names):
            self.is_active = False
            
            if self.logger is not None:
                self.logger.error('Duplicate reference frames provided for centroid constraint')
                
            return
        
        if scene is not None and not check_frames_exist_in_scene(frame_names=self.ref_frame_names, 
                                                                 scene=scene,
                                                                 logger=self.logger):
            self.is_active = False
            if self.logger is not None:
                self.logger.error(f'Reference frames "{self.ref_frame_names}" not found in the scene!')
            return
        
        if scene is not None and not check_ref_frames_for_same_parent_frame(frame_names=self.ref_frame_names, 
                                                                            scene=scene):
            self.is_active = False
            if self.logger is not None:
                self.logger.error('Reference frames do not have the same parent frame!')
            return
        

    
    def calculate_constraint(self, 
                             initial_frame_pose:Pose,
                             scene: ami_msg.ObjectScene,
                             unit: str = 'mm',
                             component_name: str = None,
                            frame_name: str = None) -> Pose:
        
        #def caluclate_frame_centroid(self, ref_frames: list[str], dim: str, offset_values: list[float], initial_pose: Pose) -> Pose:
        """
        This function calculates the centroid of the given ref frames and returns the pose of the centroid.
        """
        # Check if constraint is valid
        self.set_is_active(scene=scene, component_name=component_name)   
        
        if self.is_active == False:
            #self.logger.error('Centroid constraint is not active')
            return initial_frame_pose
        
        else:
            self.logger.debug('Centroid constraint is active')
        
        centroid_pose = Pose()
        for index, frame in enumerate(self.ref_frame_names):
            fr: ami_msg.RefFrame = get_ref_frame_by_name(frame_name=frame,
                                                         scene=scene)
            
            # This should normaly not happen as set_is_active should have already checked this
            if fr is None:
                self.logger.error(f"Frame {frame} not found in the scene")
                return initial_frame_pose
            
            frame_pose = fr.pose
            if 'x' in self.dim:
                centroid_pose.position.x += frame_pose.position.x
            if 'y' in self.dim:
                centroid_pose.position.y += frame_pose.position.y
            if 'z' in self.dim:
                centroid_pose.position.z += frame_pose.position.z
        centroid_pose.position.x = centroid_pose.position.x/len(self.ref_frame_names)
        centroid_pose.position.y = centroid_pose.position.y/len(self.ref_frame_names)
        centroid_pose.position.z = centroid_pose.position.z/len(self.ref_frame_names)
        
        multiplier = 1
        
        if unit == 'mm':
            multiplier = 1000
            
        if unit == 'um':
            multiplier = 1000000
        
        if unit == 'm':
            multiplier = 1
            
        centroid_pose.position.x += self.offset_values.x / multiplier
        centroid_pose.position.y += self.offset_values.y / multiplier
        centroid_pose.position.z += self.offset_values.z / multiplier
        
        if 'x' in self.dim:
            initial_frame_pose.position.x = centroid_pose.position.x
        else:
            initial_frame_pose.position.x = initial_frame_pose.position.x
        if 'y' in self.dim:
            initial_frame_pose.position.y = centroid_pose.position.y
        else:
            initial_frame_pose.position.y = initial_frame_pose.position.y
        if 'z' in self.dim:
            initial_frame_pose.position.z = centroid_pose.position.z
        else:
            initial_frame_pose.position.z = initial_frame_pose.position.z

        #self.logger.debug(f"Centroid pose: {str(initial_frame_pose)}")
        if frame_name is not None:
            self.logger.warn(f'Start calculating constraint for: {frame_name}')
            
        return initial_frame_pose
        
    @staticmethod
    def return_handler_from_msg( msg: ami_msg.FrConstraintCentroid, logger: RcutilsLogger = None):
        handler = CentroidConstraintHandler(logger)
        msg_copy = deepcopy(msg)
        handler.is_active = msg_copy.is_active
        handler.dim = msg_copy.dim
        handler.offset_values = msg_copy.offset_values
        handler.ref_frame_names = msg_copy.ref_frame_names
        return handler
    
    @staticmethod
    def return_handler_from_dict(dictionary: dict, 
                                 component_name: str = None, 
                                 logger: RcutilsLogger = None):

        if dictionary == {}:
            return CentroidConstraintHandler(logger)

        constraint_handler = CentroidConstraintHandler(logger)
        constraint_handler.dim = dictionary.get('dim','')
        constraint_handler.ref_frame_names = dictionary.get('ref_frame_names',[])
        
        if component_name is not None:
            for index, frame_name in enumerate(constraint_handler.ref_frame_names):
                constraint_handler.ref_frame_names[index] = component_name + '_' + frame_name
            
        offset_value_vector = dictionary.get('offset_values',[])
        
        if len(offset_value_vector) < 3:
            offset_value_vector = [0,0,0]
        constraint_handler.offset_values.x = float(offset_value_vector[0])
        constraint_handler.offset_values.y = float(offset_value_vector[1])
        constraint_handler.offset_values.z = float(offset_value_vector[2])
        return constraint_handler
    

