import assembly_manager_interfaces.srv as ami_srv
import assembly_manager_interfaces.msg as ami_msg
# import ros logger type
# RUtilsLogger
from rclpy.impl.rcutils_logger import RcutilsLogger
from geometry_msgs.msg import Pose
from copy import deepcopy, copy
import sympy as sp

from assembly_scene_publisher.py_modules.geometry_type_functions import (rotation_matrix_to_quaternion, 
                                                                         get_normal_vectors_from_quaternion)

from assembly_scene_publisher.py_modules.scene_functions import (check_frames_exist_in_scene, 
                                                                                    check_for_duplicate_frames, 
                                                                                    get_ref_frame_by_name,
                                                                                    check_ref_frames_for_same_parent_frame,
                                                                                    get_parent_frame_for_ref_frame,
                                                                                    ConstraintRestriction,
                                                                                    ConstraintRestrictionList)  

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
        
        if self.frame_1 == '' and self.frame_2 == '' and self.frame_3 == '':
            self.is_active = False
            #if self.logger is not None:
            #    self.logger.error('No reference frames provided for orthogonal constraint')
            return
        
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
                self.logger.warn(f'Constraint warning: Reference frames "{self.frame_1}, {self.frame_2}, {self.frame_3}" not found in the scene!')
        
        
        if check_for_duplicate_frames([self.frame_1, self.frame_2, self.frame_3]):
            self.is_active = False
            if self.logger is not None:
                self.logger.error('asdf Duplicate reference frames provided for orthogonal constraint')
            return
        
    def get_frame_references(self)->list[str]:
        
        frame_names = []
        frame_names.append(self.frame_1)
        frame_names.append(self.frame_2)
        frame_names.append(self.frame_3)
                
        return frame_names
    
    def get_frame_references_const(self, scene: ami_msg.ObjectScene)->ConstraintRestrictionList:
        
        frame_names = [self.frame_1, self.frame_2, self.frame_3]

        restriction_collection = ConstraintRestrictionList()
        
        for frame in frame_names:
            fr: ami_msg.RefFrame = get_ref_frame_by_name(frame_name=frame,
                                                         scene=scene)
            
            normal_vectors = get_normal_vectors_from_quaternion(fr.pose.orientation)
            
            restriction = ConstraintRestriction(frame_name=frame,
                                                constraining_vectors=normal_vectors,
                                                vector_directions=['x','y','z'])
            
            restriction_collection.add_entry(restriction)
        
        return restriction_collection
    
    
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
        
        if (component_name is not None and 
            constraint_handler.frame_1 != '' and
            constraint_handler.frame_2 != '' and
            constraint_handler.frame_3 != ''):
            constraint_handler.frame_1 = component_name + '_' + constraint_handler.frame_1
            constraint_handler.frame_2 = component_name + '_' + constraint_handler.frame_2
            constraint_handler.frame_3 = component_name + '_' + constraint_handler.frame_3

        return constraint_handler
        
    
    
    
    
    
    
    
    