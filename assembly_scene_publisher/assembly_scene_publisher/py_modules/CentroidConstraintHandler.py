import assembly_manager_interfaces.srv as ami_srv
import assembly_manager_interfaces.msg as ami_msg
# import ros logger type
# RUtilsLogger
from rclpy.impl.rcutils_logger import RcutilsLogger
from geometry_msgs.msg import Pose, Vector3, Quaternion
from copy import deepcopy, copy
import sympy as sp
from assembly_scene_publisher.py_modules.geometry_type_functions import rotation_matrix_to_quaternion

from assembly_scene_publisher.py_modules.scene_functions import (check_frames_exist_in_scene, 
                                                                                    check_for_duplicate_frames, 
                                                                                    get_ref_frame_by_name,
                                                                                    check_ref_frames_for_same_parent_frame,
                                                                                    get_parent_frame_for_ref_frame,
                                                                                    get_ref_frame_poses_by_names)    

from assembly_scene_publisher.py_modules.geometry_functions import (get_transformed_pose, 
                                                                    compute_eigenvectors_and_centroid )

class CentroidConstraintHandler(ami_msg.FrConstraintCentroid):
    def __init__(self, logger: RcutilsLogger = None):
        super(CentroidConstraintHandler, self).__init__()
        self.logger = logger
        self.multiplier = 1
    
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

    def set_multiplier(self, unit: str = 'mm'):
        self.multiplier = 1
        
        if unit == 'mm':
            self.multiplier = 1/1000
            
        if unit == 'um':
            self.multiplier = 1/1000000
        
        if unit == 'm':
            self.multiplier = 1
            
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
                self.logger.error(f'Dimension not provided for centroid constraint {self.ref_frame_names}.')
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
                self.logger.warn(f'Constraint warning: Reference frames "{self.ref_frame_names}" not found in the scene!')
            return
        
        if scene is not None and not check_ref_frames_for_same_parent_frame(frame_names=self.ref_frame_names, 
                                                                            scene=scene):
            self.is_active = False
            if self.logger is not None:
                self.logger.error('Reference frames do not have the same parent frame!')
            return
        
    def get_frame_references(self)->list[str]:
        
        frame_names = self.ref_frame_names
        
        return frame_names
    
    # def calculate_constraint(self, 
    #                          initial_frame_pose:Pose,
    #                          scene: ami_msg.ObjectScene,
    #                          unit: str = 'mm',
    #                          component_name: str = None,
    #                         frame_name: str = None) -> Pose:
        
    #     #def caluclate_frame_centroid(self, ref_frames: list[str], dim: str, offset_values: list[float], initial_pose: Pose) -> Pose:
    #     """
    #     This function calculates the centroid of the given ref frames and returns the pose of the centroid.
    #     """
    #     # Check if constraint is valid
    #     self.set_is_active(scene=scene, component_name=component_name)   
        
    #     if self.is_active == False:
    #         #self.logger.error('Centroid constraint is not active')
    #         return initial_frame_pose
        
    #     else:
    #         self.logger.debug('Centroid constraint is active')
        
    #     centroid_pose = Pose()
        
    #     for index, frame in enumerate(self.ref_frame_names):
    #         fr: ami_msg.RefFrame = get_ref_frame_by_name(frame_name=frame,
    #                                                      scene=scene)
            
    #         # This should normaly not happen as set_is_active should have already checked this
    #         if fr is None:
    #             self.logger.error(f"Frame {frame} not found in the scene")
    #             return initial_frame_pose
            
    #         frame_pose = fr.pose
    #         if 'x' in self.dim:
    #             centroid_pose.position.x += frame_pose.position.x
    #         if 'y' in self.dim:
    #             centroid_pose.position.y += frame_pose.position.y
    #         if 'z' in self.dim:
    #             centroid_pose.position.z += frame_pose.position.z
            
    #     centroid_pose.position.x = centroid_pose.position.x/len(self.ref_frame_names)
    #     centroid_pose.position.y = centroid_pose.position.y/len(self.ref_frame_names)
    #     centroid_pose.position.z = centroid_pose.position.z/len(self.ref_frame_names)
        
    #     self.set_multiplier(unit=unit)
            
    #     centroid_pose.position.x += self.offset_values.x / self.multiplier
    #     centroid_pose.position.y += self.offset_values.y / self.multiplier
    #     centroid_pose.position.z += self.offset_values.z / self.multiplier
        
    #     if 'x' in self.dim:
    #         initial_frame_pose.position.x = centroid_pose.position.x
    #     else:
    #         initial_frame_pose.position.x = initial_frame_pose.position.x
    #     if 'y' in self.dim:
    #         initial_frame_pose.position.y = centroid_pose.position.y
    #     else:
    #         initial_frame_pose.position.y = initial_frame_pose.position.y
    #     if 'z' in self.dim:
    #         initial_frame_pose.position.z = centroid_pose.position.z
    #     else:
    #         initial_frame_pose.position.z = initial_frame_pose.position.z

    #     #self.logger.debug(f"Centroid pose: {str(initial_frame_pose)}")
    #     if frame_name is not None:
    #         self.logger.warn(f'Start calculating constraint for: {frame_name}')
            
    #     return initial_frame_pose

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
        
        self.set_multiplier(unit=unit)
        
        result_pose = Pose()
        
        centroid_pose = Pose()
        
        vector = Vector3()
        vector.x = self.offset_values.x*self.multiplier
        vector.y = self.offset_values.y*self.multiplier
        vector.z = self.offset_values.z*self.multiplier
            
        if len(self.ref_frame_names) == 1:
                        
            ref_frame = get_ref_frame_by_name(frame_name=self.ref_frame_names[0],
                                                         scene=scene)
            if ref_frame is None:
                centroid_pose.position.x = initial_frame_pose.position.x
                centroid_pose.position.y = initial_frame_pose.position.y
                centroid_pose.position.z = initial_frame_pose.position.z
                centroid_pose.orientation.w = initial_frame_pose.orientation.w
                centroid_pose.orientation.x = initial_frame_pose.orientation.x
                centroid_pose.orientation.y = initial_frame_pose.orientation.y
                centroid_pose.orientation.z = initial_frame_pose.orientation.z
            else:
                centroid_pose.position.x = ref_frame.pose.position.x
                centroid_pose.position.y = ref_frame.pose.position.y
                centroid_pose.position.z = ref_frame.pose.position.z
                centroid_pose.orientation.w = ref_frame.pose.orientation.w
                centroid_pose.orientation.x = ref_frame.pose.orientation.x
                centroid_pose.orientation.y = ref_frame.pose.orientation.y
                centroid_pose.orientation.z = ref_frame.pose.orientation.z


            result_pose = get_transformed_pose(centroid_pose, vector)
        
        else:
            poses_list = get_ref_frame_poses_by_names(scene=scene, frame_names=self.ref_frame_names, logger=self.logger)
            orientation, centroid = compute_eigenvectors_and_centroid(poses_list)
            orientation: Quaternion
            centroid: Vector3
            
            centroid_pose.position.x = float(centroid.x)
            centroid_pose.position.y = float(centroid.y)
            centroid_pose.position.z = float(centroid.z)
            centroid_pose.orientation = orientation

            result_pose = get_transformed_pose(centroid_pose, vector)
            
            
        if 'x' not in self.dim:
            result_pose.position.x = float(initial_frame_pose.position.x)

        if 'y' not in self.dim:
            result_pose.position.y = float(initial_frame_pose.position.y)

        if 'z' not in self.dim:
            result_pose.position.z = float(initial_frame_pose.position.z)


        #self.logger.debug(f"Centroid pose: {str(initial_frame_pose)}")
        if frame_name is not None:
            self.logger.warn(f'Start calculating constraint for: {frame_name}')
            
        return result_pose
            
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
        constraint_handler.dim = dictionary.get('dim','xyz')
        constraint_handler.ref_frame_names = dictionary.get('refFrameNames',[])
        
        if component_name is not None:
            for index, frame_name in enumerate(constraint_handler.ref_frame_names):
                constraint_handler.ref_frame_names[index] = component_name + '_' + frame_name
        
        offset_value_vector = dictionary.get('offsetValues',[])
        
        if len(offset_value_vector) < 3:
            offset_value_vector = [0,0,0]
        constraint_handler.offset_values.x = float(offset_value_vector[0])
        constraint_handler.offset_values.y = float(offset_value_vector[1])
        constraint_handler.offset_values.z = float(offset_value_vector[2])
        return constraint_handler