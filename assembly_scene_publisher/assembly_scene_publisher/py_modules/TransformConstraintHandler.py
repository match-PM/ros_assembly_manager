import assembly_manager_interfaces.srv as ami_srv
import assembly_manager_interfaces.msg as ami_msg
# import ros logger type
# RUtilsLogger
from rclpy.impl.rcutils_logger import RcutilsLogger
from geometry_msgs.msg import Pose, Vector3, Quaternion, Transform
from copy import deepcopy, copy
import sympy as sp
from assembly_scene_publisher.py_modules.geometry_type_functions import (rotation_matrix_to_quaternion, 
                                                                         get_normal_vectors_from_quaternion)

from assembly_scene_publisher.py_modules.scene_functions import (check_frames_exist_in_scene, 
                                                                                    get_ref_frame_by_name,
                                                                                    get_ref_frame_poses_by_names,
                                                                                    ConstraintRestriction,
                                                                                    ConstraintRestrictionList)    

from assembly_scene_publisher.py_modules.geometry_functions import (get_transformed_pose, 
                                                                    compute_eigenvectors_and_centroid,
                                                                    multiply_ros_transforms)

class TransformConstraintHandler(ami_msg.FrConstraintTransform):
    def __init__(self, logger: RcutilsLogger = None):
        super(TransformConstraintHandler, self).__init__()
        self.logger = logger
        self.multiplier = 1
    
    def return_as_msg(self, logger: RcutilsLogger = None):
        msg = ami_msg.FrConstraintTransform()
        msg.is_active = self.is_active
        msg.ref_frame = self.ref_frame
        msg.transform = self.transform
        return msg
    
    def set_from_msg(self, msg: ami_msg.FrConstraintTransform):
        msg_copy = deepcopy(msg)
        self.is_active: bool = msg_copy.is_active
        self.ref_frame: str = msg_copy.ref_frame
        self.transform: Transform = msg_copy.transform

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
        if self.ref_frame == '':
            self.is_active = False
            return
        
        if scene is None:
            self.is_active = False

            return
        
        if check_frames_exist_in_scene(scene=scene, 
                                       frame_names=[self.ref_frame], 
                                       logger=self.logger):
            self.is_active = True

        else:
            self.is_active = False
        
        return
        
        
    def get_frame_references(self)->list[str]:
        
        frame_names = [self.ref_frame]
        
        return frame_names
    
    
    def get_frame_references_const(self, scene: ami_msg.ObjectScene)->ConstraintRestrictionList:
        
        frame_names = [self.ref_frame]

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
                             component_name: str = None,
                            frame_name: str = None) -> Pose:
        
        
        #def caluclate_frame_centroid(self, ref_frames: list[str], dim: str, offset_values: list[float], initial_pose: Pose) -> Pose:
        """
        This function calculates the centroid of the given ref frames and returns the pose of the centroid.
        """
        # Check if constraint is valid
        self.set_is_active(scene=scene, component_name=component_name)   
        
        if not self.is_active:
            #self.logger.error('Centroid constraint is not active')
            return initial_frame_pose
        
        else:
            self.logger.debug('Centroid constraint is active')
        
        self.set_multiplier(unit=unit)
        
        result_pose = Pose()
                
        tansform_m = Transform()
        tansform_m.translation.x = self.transform.translation.x*self.multiplier
        tansform_m.translation.y = self.transform.translation.y*self.multiplier
        tansform_m.translation.z = self.transform.translation.z*self.multiplier

        tansform_m.rotation = self.transform.rotation

        ref_frame_pose = get_ref_frame_poses_by_names(scene=scene, frame_names=[self.ref_frame])[0]
        result_pose = multiply_ros_transforms(a = ref_frame_pose, 
                                              b = tansform_m, 
                                              output_type=Pose)
        
        return result_pose
    
    def get_info(self)->str:
        info_str = f'Transform constraint:\n\t Reference frame: {self.ref_frame} \n\tTransform: {self.transform}'
        return info_str
    
    @staticmethod
    def return_handler_from_msg( msg: ami_msg.FrConstraintTransform, logger: RcutilsLogger = None):
        handler = TransformConstraintHandler(logger)
        msg_copy = deepcopy(msg)
        handler.is_active = msg_copy.is_active
        handler.ref_frame = msg_copy.ref_frame
        handler.transform = msg_copy.transform
        return handler
    
    @staticmethod
    def return_handler_from_dict(dictionary: dict, 
                                 component_name: str = None, 
                                 unique_identifier = '',
                                 logger: RcutilsLogger = None):

        if dictionary == {}:
            return TransformConstraintHandler(logger)

        constraint_handler = TransformConstraintHandler(logger)
        constraint_handler.ref_frame = dictionary.get('refFrame','')

        constraint_handler.ref_frame = unique_identifier + constraint_handler.ref_frame
        if component_name is not None:
            constraint_handler.ref_frame = component_name + '_' + constraint_handler.ref_frame

        x = dictionary.get('transform',{}).get('translation',{}).get('X',0.0)
        y = dictionary.get('transform',{}).get('translation',{}).get('Y',0.0)
        z = dictionary.get('transform',{}).get('translation',{}).get('Z',0.0)
        qx = dictionary.get('transform',{}).get('rotation',{}).get('X',0.0)
        qy = dictionary.get('transform',{}).get('rotation',{}).get('Y',0.0)
        qz = dictionary.get('transform',{}).get('rotation',{}).get('Z',0.0)
        qw = dictionary.get('transform',{}).get('rotation',{}).get('w',1.0)
        constraint_handler.transform.translation.x = x
        constraint_handler.transform.translation.y = y
        constraint_handler.transform.translation.z = z
        constraint_handler.transform.rotation.x = qx
        constraint_handler.transform.rotation.y = qy
        constraint_handler.transform.rotation.z = qz
        constraint_handler.transform.rotation.w = qw

        return constraint_handler
