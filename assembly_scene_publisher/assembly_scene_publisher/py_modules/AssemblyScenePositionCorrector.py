from assembly_scene_publisher.py_modules.AssemblySceneAnalyzer import AssemblySceneAnalyzer
from rclpy.node import Node
import assembly_manager_interfaces.msg as ami_msg
import assembly_manager_interfaces.srv as ami_srv
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
import time
from copy import copy
from geometry_msgs.msg import Transform

class AssemblyScenePositionCorrector:
    def __init__(self, node: Node, 
                 get_scene_callback,
                 assembly_scene_topic: str = '/assembly_manager/scene'):
        
        self.node = node
        self.get_scene_callback = get_scene_callback
        # create subscriber to listen for scene updates and correct positions when a new scene is received
        self.scene = None  # This will hold the current scene for comparison
        self.scene_subscription = node.create_subscription(
            ami_msg.ObjectScene,
            assembly_scene_topic,
            self.scene_callback,
            10,
            callback_group=ReentrantCallbackGroup()
        )
        
        self.external_scene_analyzer = AssemblySceneAnalyzer(None, node.get_logger())  # Initialize with None, will set the current scene in the callback
        
        self.internal_scene = ami_msg.ObjectScene()  # This will hold the original scene
        self.internal_scene_analyzer = AssemblySceneAnalyzer(self.internal_scene, node.get_logger())  # Analyzer for corrected scene
        self.logger = node.get_logger()
        self.counter = 0

    def scene_callback(self, msg: ami_msg.ObjectScene):

        start = time.perf_counter()

        # get the current scene from the callback
        self.external_scene_analyzer.set_scene(msg)  # Update the analyzer with the new scene

        # iterate through components in the received scene
        for component in msg.objects_in_scene:
            component: ami_msg.Object

            if not self.internal_scene_analyzer.check_component_exists(component.obj_name):
                self.internal_scene.objects_in_scene.append(component)
                self.logger.error(f"DEBUG - Component {component.obj_name} missing in internal scene, adding it.")

            # iterate through frames in the component of the received scene
            for frame in component.ref_frames:
                frame: ami_msg.RefFrame

                if self.internal_scene_analyzer.check_frames_exist_in_scene(frame_names=[frame.frame_name]):
                    _, frame_from_internal_scene = self.internal_scene_analyzer.get_frame_from_scene(frame.frame_name)
                    frame_from_internal_scene.properties = copy(frame.properties)  # Update properties of the existing frame in the internal scene
                    frame_from_internal_scene.constraints = copy(frame.constraints)  # Update constraints of the existing frame in the internal scene
                    continue
                
                if self.external_scene_analyzer.is_frame_constrained(frame_name=frame.frame_name, except_in_plane= True):
                    continue
                
                else:
                    internal_component = self.internal_scene_analyzer.get_component_by_name(component.obj_name)

                    internal_component.ref_frames.append(frame)
                    self.logger.error(f"DEBUG - Frame {frame.frame_name} missing in internal scene, adding it.")

        duration_ms = (time.perf_counter() - start) * 1000
        self.counter += 1
        self.logger.error(f"DEBUG Scene correction check completed. Total corrections made: {self.counter}, Duration: {duration_ms:.2f} ms")

        for obj in self.internal_scene.objects_in_scene:
            obj:ami_msg.Object
            for frame in obj.ref_frames:
                frame: ami_msg.RefFrame
                self.logger.error(
                    f"DEBUG - Internal Scene Frame: {obj.obj_name}, "
                    f"Frame: {frame.frame_name}, "
                    f"Properties: {self.internal_scene_analyzer.get_property_types_for_frame(frame.frame_name)}"
                )

        if self.counter == 50:
            true_scene: ami_msg.ObjectScene = self.get_scene_callback()
            true_scene_analyzer = AssemblySceneAnalyzer(true_scene, self.logger)
            first_component: ami_msg.Object = true_scene.objects_in_scene[0]
            first_component.obj_pose.position.x += 0.1  # Introduce a known error for testing

    def transform_component(self, component_name: str, 
                            transform: Transform,
                            frames_to_except: list[str] = []):
        
        true_scene: ami_msg.ObjectScene = self.get_scene_callback()
        true_scene_analyzer = AssemblySceneAnalyzer(true_scene, self.logger)
        true_component = true_scene_analyzer.get_component_by_name(component_name)

        true_component_transform = Transform()
        true_component_transform.translation = true_component.obj_pose.position
        true_component_transform.rotation = true_component.obj_pose.orientation

        new_true_component_transform = Transform()

        new_true_component_transform:Transform = true_component_transform * transform

        true_component.obj_pose.position = new_true_component_transform.translation 
        true_component.obj_pose.orientation = new_true_component_transform.rotation
