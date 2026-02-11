from assembly_scene_publisher.py_modules.AssemblySceneAnalyzer import AssemblySceneAnalyzer
from rclpy.node import Node
import assembly_manager_interfaces.msg as ami_msg
import assembly_manager_interfaces.srv as ami_srv
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
import time
from copy import deepcopy
from geometry_msgs.msg import Transform, Vector3, Quaternion, Pose
from assembly_scene_publisher.py_modules.geometry_functions import multiply_ros_transforms, inverse_ros_transform
class AssemblyScenePositionCorrector:
    def __init__(self, node: Node, 
                 get_scene_callback,
                 update_scene_with_constraints_callback,
                 assembly_scene_topic: str = '/assembly_manager/scene'):
        
        self.node = node
        self.get_scene_callback = get_scene_callback
        self.update_scene_with_constraints_callback = update_scene_with_constraints_callback
        # create subscriber to listen for scene updates and correct positions when a new scene is received
        self.scene = None  # This will hold the current scene for comparison
        self.scene_subscription = node.create_subscription(
            ami_msg.ObjectScene,
            assembly_scene_topic,
            self.scene_callback,
            10,
            callback_group=ReentrantCallbackGroup()
        )
        
        # Analyzer for the external scene received from the topic, will be updated in the callback
        self.external_scene_analyzer = AssemblySceneAnalyzer(None, node.get_logger())  # Initialize with None, will set the current scene in the callback
        
        self.internal_scene = ami_msg.ObjectScene()  # This will hold the original scene
        
        # Analyzer for the internal scene, will be updated in the callback
        self.internal_scene_analyzer = AssemblySceneAnalyzer(self.internal_scene, node.get_logger())  # Analyzer for corrected scene
        self.true_scene = None

        # Analyzer for the true scene, will be set when needed
        self.true_scene_analyzer = AssemblySceneAnalyzer(None, node.get_logger())  # Analyzer for the true scene, will be set when needed
        self.logger = node.get_logger()
        self.counter = 0
        self._lock = False
        # tolerances for frame comparisons (configurable)
        self.position_tolerance = 1e-5
        self.orientation_tolerance = 1e-5

    def is_locked(self):
        return self._lock
    
    def set_lock(self, value: bool):
        self._lock = value
    
    def scene_callback(self, msg: ami_msg.ObjectScene):
        
        if self.is_locked():
            #self.logger.warn("Scene callback is currently locked, skipping scene correction.")
            return
        
        start = time.perf_counter()

        # get the current scene from the callback
        self.external_scene_analyzer.set_scene(msg)  # Update the analyzer with the new scene

        # iterate through components in the received scene
        for component in msg.objects_in_scene:
            component: ami_msg.Object

            if not self.internal_scene_analyzer.check_component_exists(component.obj_name):
                # store a deep copy of the incoming object to avoid shared-message mutation
                new_component = deepcopy(component)
                new_component.ref_frames = []  # Clear frames to be added separately to avoid shared-state issues
                self.internal_scene.objects_in_scene.append(new_component)
                self.logger.debug(f"Component {component.obj_name} missing in internal scene, adding it.")
            else:
                existing_component = self.internal_scene_analyzer.get_component_by_name(component.obj_name)
                # copy properties to avoid aliasing with the incoming message
                existing_component.obj_pose = deepcopy(component.obj_pose)  # Update pose of the existing component

            # iterate through frames in the component of the received scene
            for frame in component.ref_frames:
                frame: ami_msg.RefFrame

                if self.internal_scene_analyzer.check_frames_exist_in_scene(frame_names=[frame.frame_name]):
                    _, frame_from_internal_scene = self.internal_scene_analyzer.get_frame_from_scene(frame.frame_name)
                    # copy properties/constraints to avoid aliasing with the incoming message
                    frame_from_internal_scene.properties = deepcopy(frame.properties)  # Update properties of the existing frame in the internal scene
                    frame_from_internal_scene.constraints = deepcopy(frame.constraints)  # Update constraints of the existing frame in the internal scene
                    continue
                
                if self.external_scene_analyzer.is_frame_constrained(frame_name=frame.frame_name, except_in_plane = True):
                    continue
                
                else:
                    internal_component = self.internal_scene_analyzer.get_component_by_name(component.obj_name)
                    if internal_component is None:
                        self.logger.debug(f"internal component {component.obj_name} not found when adding frame {frame.frame_name}")
                        continue
                    
                    # append a deep copy of the frame to avoid shared-state with incoming message
                    internal_component.ref_frames.append(deepcopy(frame))
                    self.logger.warn(f"Frame {frame.frame_name} missing in internal scene, adding it.")

        duration_ms = (time.perf_counter() - start) * 1000
        self.counter += 1
        self.logger.debug(f"Scene correction check completed. Total corrections made: {self.counter}, Duration: {duration_ms:.2f} ms")


        # delete later
        for obj in self.internal_scene.objects_in_scene:
            obj:ami_msg.Object
            for frame in obj.ref_frames:
                frame: ami_msg.RefFrame
                self.logger.debug(
                    f"Internal Scene Frame: {obj.obj_name}, "
                    f"Frame: {frame.frame_name}, "
                    f"Properties: {self.internal_scene_analyzer.get_property_types_for_frame(frame.frame_name)}"
                )

    def itentify_relevant_frames(self, component_name: str) -> list[ami_msg.RefFrame]:
        # Identify frames that are relevant for the position correction of the component
        relevant_frames = []
        component = self.internal_scene_analyzer.get_component_by_name(component_name)  # Ensure the component exists in the internal scene
        
        if component is None:
            self.logger.debug(f"Component {component_name} not found in internal scene when identifying relevant frames")
            return []

        for frame in component.ref_frames:
            frame: ami_msg.RefFrame

            if self.check_frames_equal(frame.frame_name):
                continue
            
            if frame.constraints.centroid.is_active:
                self.logger.warn(f"Frame {frame.frame_name} is constrained by centroid, skipping it for position correction.")
                continue

            if frame.properties.vision_frame_properties.is_vision_frame:
                self.logger.warn(f"Frame {frame.frame_name} is a vision frame, skipping it for position correction.")
                continue

            relevant_frames.append(frame)

        return relevant_frames
    

    def update_true_scene(self):
        # Update the true scene analyzer with the corrected internal scene
        self.true_scene = self.get_scene_callback()
        self.true_scene_analyzer.set_scene(self.true_scene)

    def transform_component(self, component_name: str, 
                            transform: Transform,
                            frames_to_except: list[str]):
        
        # o - origin
        # B - component
        # k - key frame
        # transformation = o_T_b_1
        
        # b_T_0_1
        inv_transform = inverse_ros_transform(transform, output_type=Transform)

        key_frame = frames_to_except[0] if len(frames_to_except) > 0 else None

        # is equal to the internal component pose, which is the original pose before correction
        true_component = self.true_scene_analyzer.get_component_by_name(component_name)
        
        o_T_b_0 = deepcopy(true_component.obj_pose)

        for frame in true_component.ref_frames:
            frame: ami_msg.RefFrame

            if not (frame.frame_name in frames_to_except):
                continue
            
            o_T_f_0 = multiply_ros_transforms(o_T_b_0, frame.pose, output_type=Transform)

            b_T_f_1 = multiply_ros_transforms(inv_transform, o_T_f_0, output_type=Pose)

            frame.pose = b_T_f_1

        true_component.obj_pose.position.x = transform.translation.x
        true_component.obj_pose.position.y = transform.translation.y
        true_component.obj_pose.position.z = transform.translation.z
        true_component.obj_pose.orientation = transform.rotation

    def correct_component_position(self, component_name: str) -> bool:
        self.set_lock(True)
        self.update_true_scene()

        relevant_frames = self.itentify_relevant_frames(component_name)
        relevant_frames_str = [frame.frame_name for frame in relevant_frames]

        if len(relevant_frames) == 0:
            self.logger.warning(f"No relevant frames found for component {component_name}, skipping position correction.")
            return True

        transform = self.calculate_transform(relevant_frames)

        self.logger.warn(f"Relevant frames for component {component_name}: {relevant_frames_str}")

        self.transform_component(component_name, transform, frames_to_except = relevant_frames_str)

        self.clear_internal_scene()
        self.set_lock(False)

        self.update_scene_with_constraints_callback()
        return True
    
    def calculate_transform(self, frame_list: list[ami_msg.RefFrame]) -> Transform:
        # Placeholder for transform calculation logic based on the differences between the frames in the internal and true scenes
        # This function should compute the necessary transform to correct the position of the component based on the discrepancies in the frames
        # For now, we return an identity transform as a placeholder
        
        internal_component_name = self.internal_scene_analyzer.get_component_for_frame_name(frame_list[0].frame_name)
        internal_component = self.internal_scene_analyzer.get_component_by_name(internal_component_name)
        
        # 0_T_b_0
        internal_component_pose = internal_component.obj_pose

        if len(frame_list) == 1:
            # b_T_k_0
            b_T_k_0 = self.internal_scene_analyzer.get_frame_from_scene(frame_list[0].frame_name)[1].pose

            # b_T_k_1
            b_T_k_1 = self.true_scene_analyzer.get_frame_from_scene(frame_list[0].frame_name)[1].pose
            
            o_T_k_1 = multiply_ros_transforms(internal_component_pose,
                                              b_T_k_1,
                                              output_type=Pose)
            

            
            k_T_b_0 = inverse_ros_transform(b_T_k_0, output_type=Transform)
            # Compute the transform needed to correct the position
            # transform = multiply_ros_transforms(k_T_b_0, 
            #                                     b_T_k_1)

            o_T_b_1 = multiply_ros_transforms(o_T_k_1, 
                                                k_T_b_0,
                                                output_type=Transform)
            return o_T_b_1

        if len(frame_list) == 2:
            return Transform()  # Placeholder for the actual transform calculation logic when there are two frames to consider

        if len(frame_list) > 2:
            return Transform()  # Placeholder for the actual transform calculation logic when there are more than two frames to consider
        
    
    def clear_internal_scene(self):
        self.internal_scene = ami_msg.ObjectScene()
        self.internal_scene_analyzer.set_scene(self.internal_scene)


    def check_frames_equal(self, frame_name: str) -> bool:
        """
        This function compares the position and orientation of a specific frame in the true scene and the internal scene to check if they match within a certain tolerance.
        
        :param self: Description
        :param frame_name: Description
        :type frame_name: str
        :return: Description
        :rtype: bool
        """
        frame_true = self.true_scene_analyzer.get_frame_from_scene(frame_name)
        frame_internal = self.internal_scene_analyzer.get_frame_from_scene(frame_name)

        if frame_true is None or frame_internal is None:
            self.logger.debug(f"Frame {frame_name} missing in one of the scenes when checking equality")
            return False

        true_frame = frame_true[1]
        internal_frame = frame_internal[1]

        true_position = true_frame.pose.position
        internal_position = internal_frame.pose.position

        position_match = (abs(true_position.x - internal_position.x) < self.position_tolerance and
                          abs(true_position.y - internal_position.y) < self.position_tolerance and
                          abs(true_position.z - internal_position.z) < self.position_tolerance)
        
        true_orientation = true_frame.pose.orientation
        internal_orientation = internal_frame.pose.orientation
        orientation_match = (abs(true_orientation.x - internal_orientation.x) < self.orientation_tolerance and
                             abs(true_orientation.y - internal_orientation.y) < self.orientation_tolerance and
                             abs(true_orientation.z - internal_orientation.z) < self.orientation_tolerance and
                             abs(true_orientation.w - internal_orientation.w) < self.orientation_tolerance)

        self.logger.debug(f"Checking frame {frame_name}: Position match: {position_match}, Orientation match: {orientation_match}")

        return position_match and orientation_match