from assembly_scene_publisher.py_modules.AssemblySceneAnalyzer import AssemblySceneAnalyzer
from assembly_scene_publisher.py_modules.AssemblySceneAnalyzerAdv import AssemblySceneAnalyzerAdv

from rclpy.node import Node
import assembly_manager_interfaces.msg as ami_msg
import assembly_manager_interfaces.srv as ami_srv
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
import time
from copy import deepcopy
from geometry_msgs.msg import Transform, Vector3, Quaternion, Pose
from assembly_scene_publisher.py_modules.geometry_functions import multiply_ros_transforms, inverse_ros_transform
from assembly_scene_publisher.py_modules.LineOfSightChecker import LineOfSightChecker
import numpy as np
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Pose, Transform
from assembly_scene_publisher.py_modules.scene_errors import *

class LineOfSightManager:
    def __init__(self, node: Node, 
                 assembly_scene_topic: str = '/assembly_manager/scene'):
        
        self.node = node
        # create subscriber to listen for scene updates and correct positions when a new scene is received
        self.logger = self.node.get_logger()
        self.scene = None  # This will hold the current scene for comparison
        self.scene_subscription = node.create_subscription(
            ami_msg.ObjectScene,
            assembly_scene_topic,
            self.scene_callback,
            10,
            callback_group=ReentrantCallbackGroup()
        )
        self.internal_scene = ami_msg.ObjectScene()
        self.internal_scene_analyzer = AssemblySceneAnalyzerAdv(self.internal_scene, logger=self.logger)

        self.incoming_scene = ami_msg.ObjectScene()
        self.incoming_scene_analyzer = AssemblySceneAnalyzerAdv(self.incoming_scene, logger=self.logger)

    def scene_callback(self, msg: ami_msg.ObjectScene):
        self.incoming_scene_analyzer.set_scene(msg)

        if self.incoming_scene_analyzer.check_is_any_frame_measured():
            #self.logger.warn("Not updating as at least one frame has been measured.")
            return  # If any frame is measured, skip LOS correction
        else:
            #self.logger.warn("No measured frames in the scene, updating internal scene without LOS correction.")
            self.internal_scene.objects_in_scene = msg.objects_in_scene
            self.internal_scene_analyzer.set_scene(self.internal_scene)
            #self.logger.warn("OVERWRITING")


    def check_line_of_sight(self, request: ami_srv.CheckLineOfSight.Request):
        
        response = ami_srv.CheckLineOfSight.Response()
        try:
            # Implement line of sight checking logic here
            component_name = self.internal_scene_analyzer.get_component_for_frame_name(request.frame_name)
            component = self.internal_scene_analyzer.get_component_by_name(component_name)

            frame = self.internal_scene_analyzer.get_ref_frame_by_name(request.frame_name)
            endeffector_pose = multiply_ros_transforms(frame.pose, request.transform_from_frame)

            checker = LineOfSightChecker(mesh_filepath=component.cad_data,
                                         viewpoint=endeffector_pose,
                                         target_point=frame.pose,
                                         logger=self.logger)
            
            points, collision_detected = checker.check_line_of_sight()
            
            if collision_detected:
                self.logger.error(f"Line of sight blocked for frame '{request.frame_name}'.")
                checker.visualize(timeout_seconds=10)
            else:
                self.logger.info(f"Line of sight clear for frame '{request.frame_name}'.")
                checker.visualize(timeout_seconds=10)

            response.success = not collision_detected
            
        except (RefFrameNotFoundError, ComponentNotFoundError) as e:
            self.logger.error(f"Reference frame not found: {e}")
            response.success = False
        
        except Exception as e:
            self.logger.error(f"Error during line of sight check: {e}")
            response.success = False
        
        return response