#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from rclpy.callback_groups import ReentrantCallbackGroup
import assembly_manager_interfaces.srv as ami_srv
import assembly_manager_interfaces.msg as ami_msg
import assembly_manager_interfaces.action as ami_action

# The adhesive displayer lives in a separate repo (match_pm_robot). Import it
# optionally so the scene publisher still starts if pm_msgs is not present.
try:
    from pm_msgs.srv import EmptyWithSuccess
except ImportError:
    EmptyWithSuccess = None

from rclpy.executors import MultiThreadedExecutor

import os
import json
from assembly_scene_publisher.py_modules.AssemblyScene import AssemblyManagerScene
from assembly_scene_publisher.py_modules.AssemblyDescriptionHandler import (
    AssemblyDescriptionHandler,
)
from assembly_scene_publisher.py_modules.scene_errors import (
    AddRefFrameError,
    AssemblyTransformationError,
    AssemblyInstructionNotFoundError,
    ComponentNotFoundError,
    RefFrameNotFoundError,
    RefPlaneNotFoundError,
    TfFrameLookupError,
)
from assembly_scene_publisher.py_modules.AssemblyScenePositionCorrector import AssemblyScenePositionCorrector
from assembly_scene_publisher.py_modules.LineOfSightManager import LineOfSightManager
from assembly_scene_publisher.py_modules.AssemblySceneMonteCarloSimulator import AssemblySceneMonteCarloSimulator


class AssemblyScenePublisherNode(Node):
    def __init__(self):
        super().__init__('assembly_scene_publisher')

        assembly_scene_topic = '/assembly_manager/scene'
        self.callback_group = ReentrantCallbackGroup()

        self.object_scene = AssemblyManagerScene(
            node=self,
            assembly_scene_topic=assembly_scene_topic,
        )
        self.position_corrector = AssemblyScenePositionCorrector(
            node=self,
            get_scene_callback=self.object_scene.get_scene,
            assembly_scene_topic=assembly_scene_topic,
            update_scene_with_constraints_callback=(
                self.object_scene.update_scene_with_constraints),
        )
        self.line_of_sight_manager = LineOfSightManager(
            node=self,
            assembly_scene_topic=assembly_scene_topic,
        )
        self.monte_carlo_simulator = AssemblySceneMonteCarloSimulator(
            self.object_scene)
        self.description_handler = AssemblyDescriptionHandler(
            node=self,
            object_scene=self.object_scene,
            spawn_object_callback=self.spawn_object_callback,
            create_ref_frame_callback=self.create_ref_frame,
            create_axis_callback=self.create_axis,
            create_ref_plane_callback=self.create_ref_plane,
            create_assembly_instructions_callback=(
                self.srv_create_assembly_instructions),
            create_frame_assembly_instructions_callback=(
                self.srv_create_frame_assembly_instructions),
        )

        self._create_ros_interfaces()
        self._create_adhesive_client()
        self.get_logger().info('Assembly scene publisher started!')

    def _create_ros_interfaces(self):
        """Create service servers, action servers, and required clients."""
        manager = 'assembly_manager'
        publisher = 'assembly_scene_publisher'
        service_specs = (
            ('spawn_object_srv', ami_srv.SpawnObject,
             f'{publisher}/spawn_object', self.spawn_object_callback),
            ('destroy_object_srv', ami_srv.DestroyObject,
             f'{publisher}/destroy_object', self.destroy_object_callback),
            ('manager_spawn_object_srv', ami_srv.SpawnObject,
             f'{manager}/spawn_object', self.spawn_object_callback),
            ('manager_destroy_object_srv', ami_srv.DestroyObject,
             f'{manager}/destroy_object', self.destroy_object_callback),
            ('spawn_component_from_description_srv',
             ami_srv.SpawnComponentFromDescription,
             f'{manager}/spawn_component_from_description',
             self.description_handler.spawn_component_callback),
            ('create_assembly_instruction_from_description_srv',
             ami_srv.CreateAssemblyInstructionFromDescription,
             f'{manager}/create_assembly_instruction_from_description',
             self.description_handler.create_assembly_instructions_callback),
            ('create_ref_frame_srv', ami_srv.CreateRefFrame,
             f'{manager}/create_ref_frame', self.create_ref_frame),
            ('delete_ref_frame_srv', ami_srv.DeleteRefFrame,
             f'{manager}/delete_ref_frame', self.destroy_ref_frame),
            ('change_parent_frame_srv', ami_srv.ChangeParentFrame,
             f'{manager}/change_obj_parent_frame',
             self.change_obj_parent_frame),
            ('modify_frame_absolut_srv', ami_srv.ModifyPoseAbsolut,
             f'{manager}/modify_frame_absolut', self.modify_frame_absolut),
            ('modify_frame_relative_srv', ami_srv.ModifyPoseRelative,
             f'{manager}/modify_frame_relative', self.modify_frame_relative),
            ('get_info_srv', ami_srv.GetScene,
             f'{manager}/get_scene', self.get_scene),
            ('create_ref_plane_srv', ami_srv.CreateRefPlane,
             f'{manager}/create_ref_plane', self.create_ref_plane),
            ('create_axis_srv', ami_srv.CreateAxis,
             f'{manager}/create_axis', self.create_axis),
            ('create_assembly_instructions_srv',
             ami_srv.CreateAssemblyInstructions,
             f'{manager}/create_assembly_instructions',
             self.srv_create_assembly_instructions),
            ('create_frame_assembly_instructions_srv',
             ami_srv.CreateFrameAssemblyInstructions,
             f'{manager}/create_frame_assembly_instructions',
             self.srv_create_frame_assembly_instructions),
            ('calculate_assembly_instructions_srv',
             ami_srv.CalculateAssemblyInstructions,
             f'{manager}/calculate_assembly_instructions',
             self.calculate_assembly_instructions),
            ('check_assembly_prerequisites_srv',
             ami_srv.CheckAssemblyPrerequisites,
             f'{manager}/check_assembly_prerequisites',
             self.check_assembly_prerequisites),
            ('spawn_frames_from_description_srv',
             ami_srv.SpawnFramesFromDescription,
             f'{manager}/spawn_frames_from_description',
             self.spawn_frames_from_description),
            ('get_frames_for_component_srv', ami_srv.FramesForComponent,
             f'{manager}/get_frames_for_component',
             self.get_frames_for_component),
            ('modify_frame_from_frame_srv', ami_srv.ModifyPoseFromFrame,
             f'{manager}/modify_frame_from_frame',
             self.modify_frame_from_frame),
            ('clear_scene_srv', ami_srv.ClearScene,
             f'{manager}/clear_scene', self.clear_scene),
            ('save_scene_to_file_srv', ami_srv.SaveSceneToFile,
             f'{manager}/save_scene_to_file', self.save_scene_to_file),
            ('load_scene_from_file_srv', ami_srv.LoadSceneFromFile,
             f'{manager}/load_scene_from_file', self.load_scene_from_file),
            ('invalidate_frame_measurements_srv', ami_srv.InvalidateFrameMeasurements,
             f'{manager}/invalidate_frame_measurements',
             self.invalidate_frame_measurements),
            ('reset_frame_properties_srv', ami_srv.ResetFrameProperties,
             f'{manager}/reset_frame_properties',
             self.reset_frame_properties),
            ('set_frame_properties_srv', ami_srv.SetFrameProperties,
             f'{manager}/set_frame_properties', self.set_frame_properties),
            ('set_component_uuid_srv', ami_srv.SetComponentUuid,
             f'{manager}/set_component_uuid', self.set_component_uuid),
            ('check_line_of_sight_srv', ami_srv.CheckLineOfSight,
             f'{manager}/check_line_of_sight', self.check_line_of_sight),
            ('set_component_properties_srv', ami_srv.SetComponentProperties,
             f'{manager}/set_component_properties',
             self.set_component_properties),
            ('correct_component_position_srv',
             ami_srv.CorrectComponentPosition,
             f'{manager}/correct_component_position',
             self.correct_component_position),
        )

        for attribute, service_type, name, callback in service_specs:
            service = self.create_service(
                service_type,
                name,
                callback,
                callback_group=self.callback_group,
            )
            setattr(self, attribute, service)

        self.monte_carlos_simulation_action_srv = rclpy.action.ActionServer(
            self,
            ami_action.MonteCarloSimulation,
            f'{manager}/monte_carlo_simulation',
            execute_callback=self.monte_carlo_simulator.execute_action,
            goal_callback=self.monte_carlo_simulator.goal_callback,
            cancel_callback=self.monte_carlo_simulator.cancel_callback,
            callback_group=self.callback_group,
        )

    def _create_adhesive_client(self):
        """Create the optional adhesive-displayer integration."""
        self.declare_parameter(
            'clear_adhesive_points_service',
            '/pm_adhesive_displayer/clear_points',
        )
        service_name = self.get_parameter(
            'clear_adhesive_points_service').value
        if EmptyWithSuccess is None:
            self.clear_adhesive_points_client = None
            return

        self.clear_adhesive_points_client = self.create_client(
            EmptyWithSuccess,
            service_name,
            callback_group=self.callback_group,
        )

    def get_scene(self, request :ami_srv.GetScene.Request, response:ami_srv.GetScene.Response):
        response.scene = self.object_scene.scene
        response.success = True
        return response

    def change_obj_parent_frame(self, request:ami_srv.ChangeParentFrame.Request, response:ami_srv.ChangeParentFrame.Response):
        '''This function is the callback function for the /ChangeParentFrame Service. It iterates through the objects list, 
        and in case it finds the valid object it calculates the transformation to a new given parent frame. It then publishes the calculated.'''
        change_success = self.object_scene.change_obj_parent_frame(     obj_id=request.obj_name,
                                                                        new_parent_frame=request.new_parent_frame)
        response.success = change_success
        return response

    def create_ref_frame(self, request: ami_srv.CreateRefFrame.Request, response: ami_srv.CreateRefFrame.Response):
        """This is the callback function for the /CreateRefFrame service. If the TF Frame does not exist, the function creates a new TF Frame. 
        If the frame already exists, the inforamtion will be updated!"""
        
        try:
            self.object_scene.add_ref_frame_to_scene(request.ref_frame)

        except AddRefFrameError as e:
            self.get_logger().error(f"Error occurred while adding reference frame: {e}")
            response.success = False
            return response

        if request.recalculate_constraints:
            self.object_scene.update_scene_with_constraints()

        response.success = True
        
        return response

    def create_ref_plane(self,request:ami_srv.CreateRefPlane.Request, response:ami_srv.CreateRefPlane.Response):
        create_success = self.object_scene.create_ref_plane(plane = request.ref_plane)
        response.success = create_success
        return response

    def create_axis(self,request:ami_srv.CreateAxis.Request, response:ami_srv.CreateAxis.Response):
        create_success = self.object_scene.create_axis(axis = request.axis)
        response.success = create_success
        return response
    
    def destroy_ref_frame(self, request: ami_srv.DeleteRefFrame.Request, response: ami_srv.DeleteRefFrame.Response):
        """This is the callback function for the /CreateRefFrame service. If the TF Frame does not exist, the function creates a new TF Frame. 
        If the frame already exists, the inforamtion will be updated!"""
        
        add_success = self.object_scene.destroy_ref_frame(frame_id = request.frame_name)
        response.success = add_success

        return response   

    def spawn_object_callback(self, request: ami_srv.SpawnObject.Request, response: ami_srv.SpawnObject.Response):
        """This method spawns an object. This means that all the relevant object information are stored in a list that contains all the objects. 
        It then publishes the information as topics and also publishes a Frame in TF"""
        new_obj = ami_msg.Object()
        new_obj.cad_data = request.cad_data
        new_obj.cad_data_collision = request.cad_data_collision
        new_obj.obj_name = request.obj_name
        new_obj.uuid = request.uuid
        new_obj.component_type_uuid = request.component_type_uuid
        new_obj.apperance_color = request.apperance_color
        new_obj.parent_frame = request.parent_frame
        new_obj.obj_pose.orientation = request.rotation
        new_obj.obj_pose.position.x = request.translation.x
        new_obj.obj_pose.position.y = request.translation.y
        new_obj.obj_pose.position.z = request.translation.z

        add_success, add_message = self.object_scene.add_obj_to_scene(new_obj)
        response.success = add_success
        response.message = add_message

        return response
    
    def destroy_object_callback(self, request:ami_srv.DestroyObject.Request, response:ami_srv.DestroyObject.Response):
        del_success = self.object_scene.destroy_object(request.obj_name)
        response.success = del_success
        return response

    def modify_frame_absolut(self, request: ami_srv.ModifyPoseAbsolut.Request, response: ami_srv.ModifyPoseAbsolut.Response):
        modify_success = self.object_scene.modify_frame_absolut(request = request)
        response.success = modify_success
        return response

    def modify_frame_relative(self, request: ami_srv.ModifyPoseRelative.Request, response: ami_srv.ModifyPoseRelative.Response):
        modify_success = self.object_scene.modify_frame_relative(request = request)
        response.success = modify_success
        return response
        
    def srv_create_assembly_instructions(self, request: ami_srv.CreateAssemblyInstructions.Request, response: ami_srv.CreateAssemblyInstructions.Response):
        try:
            create_success = self.object_scene.create_assembly_instructions(instruction=request.assembly_instruction)
            response.instruction_id = request.assembly_instruction.id
            response.success = create_success

        except (RefPlaneNotFoundError, RefFrameNotFoundError,
                ComponentNotFoundError, AssemblyTransformationError,
                TfFrameLookupError, ValueError) as e:
            self.get_logger().error(f"Error creating assembly instructions: {e}")
            response.success = False
            response.message = str(e)
    
        return response
    
    def srv_create_frame_assembly_instructions(self, request, response):
        """Create an instruction in the dedicated frame-match scene list."""
        response.instruction_id = request.assembly_instruction.id
        try:
            response.success = self.object_scene.create_frame_assembly_instructions(
                request.assembly_instruction)
        except (RefFrameNotFoundError, ComponentNotFoundError,
                AssemblyTransformationError, ValueError) as error:
            response.success = False
            response.message = str(error)
        return response

    def calculate_assembly_instructions(self, request: ami_srv.CalculateAssemblyInstructions.Request, response: ami_srv.CalculateAssemblyInstructions.Response):
        
        try:
            transfrom = self.object_scene.get_assembly_transformation_by_id(request.instruction_id)
            
        except (AssemblyTransformationError,
                ComponentNotFoundError,
                RefPlaneNotFoundError,
                RefFrameNotFoundError,
                TfFrameLookupError,
                ValueError,
                AddRefFrameError) as e:
            self.get_logger().error(f"Error getting the assembly transformation: {e}")
            response.success = False
            return response
        
        if transfrom == None:
            response.success = False
        else:
            response.success = True
            response.assembly_transform = transfrom
        return response

    def check_assembly_prerequisites(self, request, response):
        """Report readiness without blocking geometry calculation."""
        try:
            response.unmet_prerequisites = \
                self.object_scene.assembly_scene_analyzer.check_assembly_prerequisites(
                    request.instruction_id)
            response.satisfied = not response.unmet_prerequisites
            response.success = True
        except AssemblyInstructionNotFoundError as error:
            response.success = False
            response.message = str(error)
        return response
    
    def modify_frame_from_frame(self, request: ami_srv.ModifyPoseFromFrame.Request, response: ami_srv.ModifyPoseFromFrame.Response):
        modify_success = self.object_scene.modify_frame_set_to_frame(request = request)
        response.success = modify_success
        return response
    
    def spawn_frames_from_description(self, request: ami_srv.SpawnFramesFromDescription.Request, response: ami_srv.SpawnFramesFromDescription.Response):
        frames_dictionary = {}

        if os.path.exists(request.dict_or_path):

            try:
                with open(request.dict_or_path, 'r') as file:
                    frames_dictionary = json.loads(file.read())
            except FileNotFoundError:
                self.get_logger().error(f"File not found {request.dict_or_path}")
            except json.JSONDecodeError as e:
                self.get_logger().error(f"Error loading the json file {e}")
        else:
            self.get_logger().warn(f"Did not find the file {request.dict_or_path}. Assuming the input is a json string!")
            try:
                frames_dictionary = json.loads(request.dict_or_path)
            except json.JSONDecodeError as e:
                self.get_logger().error(f"Error loading the json string {e}")
        
        if frames_dictionary == {}:
            response.success = False
            return response
        
        response.success = self.object_scene.add_ref_frames_to_scene_from_dict(frames_dictionary)
        return response
    
    def get_frames_for_component(self, request: ami_srv.FramesForComponent.Request, response: ami_srv.FramesForComponent.Response):
        frames = self.object_scene.get_core_frames_for_component(component_name = request.component_name)
        
        if frames == None:
            response.success = False
            return response
        
        if len(frames) == 0:
            response.success = False
            return response
        
        response.frame_names = frames
        response.success = True
        return response
    
    def clear_scene(self, request: ami_srv.ClearScene.Request, response: ami_srv.ClearScene.Response):
        response.success = self.object_scene.clear_scene(request.save_data)
        self.clear_adhesive_points()
        return response

    def clear_adhesive_points(self):
        """Notify the adhesive displayer to drop its points so it stops
        publishing adhesive markers once the scene has been cleared. This is
        best-effort: if the displayer is not running the scene is still cleared."""
        if self.clear_adhesive_points_client is None:
            return

        if not self.clear_adhesive_points_client.service_is_ready():
            self.get_logger().debug("Adhesive displayer clear service not available; skipping.")
            return

        # Fire-and-forget so clearing the scene is never blocked by the displayer.
        self.clear_adhesive_points_client.call_async(EmptyWithSuccess.Request())
    
    def save_scene_to_file(self, request: ami_srv.SaveSceneToFile.Request, response: ami_srv.SaveSceneToFile.Response):
        try:
            response.success, path = self.object_scene.save_scene_to_file(request.file_path)
        except Exception as e:
            self.get_logger().error(f"Error saving scene to file: {e}")
            response.success = False    
        return response

    def load_scene_from_file(self, request: ami_srv.LoadSceneFromFile.Request, response: ami_srv.LoadSceneFromFile.Response):
        try:

            response.success = self.object_scene.load_scene_from_file(request.file_path)
        except Exception as e:
            self.get_logger().error(f"Error loading scene from file: {e}")
            response.success = False
        return response

    def invalidate_frame_measurements(self, request, response):
        return self.object_scene.invalidate_frame_measurements(request)

    def reset_frame_properties(self, request: ami_srv.ResetFrameProperties.Request, response: ami_srv.ResetFrameProperties.Response):
        response = self.object_scene.reset_frame_properties(request)
        return response
    
    def set_frame_properties(self, request: ami_srv.SetFrameProperties.Request, response: ami_srv.SetFrameProperties.Response):
        response = self.object_scene.set_frame_properties(request)
        return response

    def set_component_uuid(self, request: ami_srv.SetComponentUuid.Request, response: ami_srv.SetComponentUuid.Response):
        response = self.object_scene.set_component_uuid(request)
        return response

    def set_component_properties(self, request: ami_srv.SetComponentProperties.Request, response: ami_srv.SetComponentProperties.Response):
        response = self.object_scene.set_component_properties(request)
        return response

    def correct_component_position(self, request: ami_srv.CorrectComponentPosition.Request, response: ami_srv.CorrectComponentPosition.Response):
        try:
            response.success = self.position_corrector.correct_component_position(
                request.component_name,
                request.reference_frame,
            )
            response.message = ""
        
        except (ComponentNotFoundError, RefFrameNotFoundError) as e:
            self.get_logger().error(f"Error correcting component position: {e}")
            response.success = False
            response.message = str(e)
        return response

    def check_line_of_sight(self, request: ami_srv.CheckLineOfSight.Request, response: ami_srv.CheckLineOfSight.Response):
        response = self.line_of_sight_manager.check_line_of_sight(request)
        return response



def main(args=None):
    rclpy.init(args=args)
    node = AssemblyScenePublisherNode()
   
    executor = MultiThreadedExecutor(num_threads=6) 

    executor.add_node(node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()
