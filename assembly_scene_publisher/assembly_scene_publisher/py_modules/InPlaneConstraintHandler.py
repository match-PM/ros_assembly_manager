import assembly_manager_interfaces.srv as ami_srv
import assembly_manager_interfaces.msg as ami_msg
# import ros logger type
# RUtilsLogger
from rclpy.impl.rcutils_logger import RcutilsLogger
from geometry_msgs.msg import Pose
from copy import deepcopy, copy
import sympy as sp
from assembly_scene_publisher.py_modules.geometry_type_functions import rotation_matrix_to_quaternion

from assembly_scene_publisher.py_modules.scene_functions import (check_frames_exist_in_scene, 
                                                                                    check_for_duplicate_frames, 
                                                                                    get_ref_frame_by_name,
                                                                                    check_ref_frames_for_same_parent_frame,
                                                                                    get_parent_frame_for_ref_frame,
                                                                                    get_ref_frame_poses_by_names
                                                                                    )   

from assembly_scene_publisher.py_modules.geometry_functions import (create_3D_plane_2,
                                                                    project_pose_on_plane,
                                                                    rotate_point)

class InPlaneConstraintHandler(ami_msg.FrConstraintInPlane):
    def __init__(self, logger: RcutilsLogger = None):
        super(InPlaneConstraintHandler, self).__init__()
        self.logger = logger
    
    def return_as_msg(self, logger: RcutilsLogger = None):
        msg = ami_msg.FrConstraintInPlane()
        msg.is_active = self.is_active
        msg.normal_axis = self.normal_axis
        msg.ref_frame_names = self.ref_frame_names
        msg.plane_offset = self.plane_offset
        return msg

    def set_from_msg(self, msg: ami_msg.FrConstraintInPlane):
        msg_copy = deepcopy(msg)
        self.is_active = msg_copy.is_active
        self.normal_axis = msg_copy.normal_axis
        self.plane_offset = msg_copy.plane_offset
        self.ref_frame_names = msg_copy.ref_frame_names

    def set_is_active(self, scene: ami_msg.ObjectScene = None, component_name = None):
        
        if len(self.ref_frame_names)==0:
            self.is_active = False
            return
        
        # check if enough reference frames are provided
        if len(self.ref_frame_names)>2:
            self.is_active = True
        else:
            self.is_active = False
            if self.logger is not None:
                self.logger.error(f'Not enough reference frames provided for centroid constraint{self.ref_frame_names}. ')
            return
        
        if not self.normal_axis in ['x','y','z']:
            self.is_active = False
            if self.logger is not None:
                self.logger.error('Invalid normal axis provided for in_plane constraint')
            return
        
        if check_for_duplicate_frames(self.ref_frame_names):
            self.is_active = False
            if self.logger is not None:
                self.logger.error('Duplicate reference frames provided for in_plane constraint')
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
        
    def get_frame_references(self)->list[str]:
        
        frame_names = self.ref_frame_names
        
        return frame_names
    
    def set_multiplier(self, unit: str = 'mm'):
        self.multiplier = 1
        
        if unit == 'mm':
            self.multiplier = 1/1000
            
        if unit == 'um':
            self.multiplier = 1/1000000
        
        if unit == 'm':
            self.multiplier = 1

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
        #self.logger.warn(f'InPlane constraint is active: {frame_name}')

        self.set_multiplier(unit=unit)

        frames = get_ref_frame_poses_by_names(scene=scene,frame_names=self.ref_frame_names)

        plane = create_3D_plane_2(frames,
                                  plane_offset=self.plane_offset*self.multiplier,
                                  logger=self.logger)

        new_pose = project_pose_on_plane(initial_frame_pose, 
                                         plane, 
                                         self.logger)
        
        new_pose = rotate_point(new_pose, 
                                plane, 
                                target_axis=self.normal_axis,
                                logger=self.logger)
            
        return new_pose
        
    @staticmethod
    def return_handler_from_msg( msg: ami_msg.FrConstraintInPlane, logger: RcutilsLogger = None):
        handler = InPlaneConstraintHandler(logger)
        msg_copy = deepcopy(msg)
        handler.is_active = msg_copy.is_active
        handler.normal_axis = msg_copy.normal_axis
        handler.plane_offset = msg_copy.plane_offset
        handler.ref_frame_names = msg_copy.ref_frame_names
        return handler
    
    @staticmethod
    def return_handler_from_dict(dictionary: dict, 
                                 component_name: str = None, 
                                 logger: RcutilsLogger = None):

        if dictionary == {}:
            return InPlaneConstraintHandler(logger)

        constraint_handler = InPlaneConstraintHandler(logger)
        constraint_handler.plane_offset = dictionary.get('planeOffset',0.0)
        constraint_handler.normal_axis = dictionary.get('normalAxis','z')
        constraint_handler.ref_frame_names = dictionary.get('refFrameNames',[])
    
        if component_name is not None:
            for index, frame_name in enumerate(constraint_handler.ref_frame_names):
                constraint_handler.ref_frame_names[index] = component_name + '_' + frame_name
                
        if constraint_handler.ref_frame_names != []:
            logger.debug(f'DEBUG InPlane constraint with: {constraint_handler.ref_frame_names}')  
                  
        return constraint_handler