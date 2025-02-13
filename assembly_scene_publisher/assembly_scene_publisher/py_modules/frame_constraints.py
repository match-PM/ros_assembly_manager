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
                                                                    get_parent_frame_for_ref_frame)                                    

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
        frame_constraints.centroid_handler = centroid_handler

        
        
        
        # init orthogonal constraint handler
        orthogonal_handler = OrthogonalConstraintHandler.return_handler_from_dict(dictionary.get('orthogonal',{}),
                                                                                  component_name=component_name,
                                                                                  logger=logger)
        
        orthogonal_handler.set_is_active(scene=scene,
                                            component_name=component_name)
        
        frame_constraints.orthogonal = orthogonal_handler.return_as_msg()
        frame_constraints.orthogonal_handler = orthogonal_handler

        
        
        in_plane_handler = InPlaneConstraintHandler.return_handler_from_dict(dictionary.get('inPlane',{}),
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
    
    def get_frame_references(self):
        frame_references = []
        
        if self.centroid.is_active:
            frame_references.extend(self.centroid_handler.get_frame_references())
        if self.orthogonal.is_active:
            frame_references.extend(self.orthogonal_handler.get_frame_references())
        if self.in_plane.is_active:
            frame_references.extend(self.in_plane_handler.get_frame_references())
            
        return frame_references
    
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
                # calculate the in-plane constraint
                in_plane_handler = InPlaneConstraintHandler(logger=self.logger)
                in_plane_handler.set_from_msg(self.in_plane)
                result_pose = in_plane_handler.calculate_constraint(initial_frame_pose=initial_pose,
                                                                    scene=scene,
                                                                    component_name=component_name,
                                                                    unit=self.unit,
                                                                    frame_name=frame_name)

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
    
    


    
    
    
    

