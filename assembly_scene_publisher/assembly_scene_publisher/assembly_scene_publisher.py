#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from tf2_ros import Buffer, TransformListener, TransformBroadcaster, StaticTransformBroadcaster
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

from std_srvs.srv import Empty
import assembly_manager_interfaces.srv as ami_srv
import assembly_manager_interfaces.msg as ami_msg

from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

import os
import json
from typing import Union
from assembly_scene_publisher.py_modules.AssemblyScene import AssemblyManagerScene
from assembly_scene_publisher.py_modules.scene_errors import *

class AssemblyScenePublisherNode(Node):
    def __init__(self):
        super().__init__("assembly_scene_publisher")

        #self.callback_group = MutuallyExclusiveCallbackGroup()
        self.callback_group = ReentrantCallbackGroup()

        self.object_scene = AssemblyManagerScene(self)
        mng_str = "assembly_manager" 
        pub_str = "assembly_scene_publisher"
        self.spawn_object_srv = self.create_service(ami_srv.SpawnObject,f'{pub_str}/spawn_object',self.spawn_object_callback,callback_group=self.callback_group)
        self.destroy_object_srv = self.create_service(ami_srv.DestroyObject,f'{pub_str}/destroy_object',self.destroy_object_callback,callback_group=self.callback_group)

        self.create_ref_frame_srv = self.create_service(ami_srv.CreateRefFrame,f'{mng_str}/create_ref_frame',self.create_ref_frame,callback_group=self.callback_group)
        self.delete_ref_frame_srv = self.create_service(ami_srv.DeleteRefFrame,f'{mng_str}/delete_ref_frame',self.destroy_ref_frame,callback_group=self.callback_group)  
        self.change_parent_frame_srv = self.create_service(ami_srv.ChangeParentFrame,f'{mng_str}/change_obj_parent_frame',self.change_obj_parent_frame,callback_group=self.callback_group)  
        
        self.modify_frame_absolut_srv = self.create_service(ami_srv.ModifyPoseAbsolut,f'{mng_str}/modify_frame_absolut',self.modify_frame_absolut,callback_group=self.callback_group)  
        
        self.modify_frame_relative_srv = self.create_service(ami_srv.ModifyPoseRelative,f'{mng_str}/modify_frame_relative',self.modify_frame_relative,callback_group=self.callback_group)  

        self.get_info_srv = self.create_service(ami_srv.GetScene,f'{mng_str}/get_scene',self.get_scene,callback_group=self.callback_group)      

        self.create_ref_plane_srv = self.create_service(ami_srv.CreateRefPlane,f'{mng_str}/create_ref_plane',self.create_ref_plane,callback_group=self.callback_group)      
        
        self.create_axis_srv = self.create_service(ami_srv.CreateAxis,f'{mng_str}/create_axis',self.create_axis,callback_group=self.callback_group)      

        self.create_assembly_instructions_srv = self.create_service(ami_srv.CreateAssemblyInstructions,f'{mng_str}/create_assembly_instructions',self.srv_create_assembly_instructions,callback_group=self.callback_group)      

        self.calculate_assembly_instructions_srv = self.create_service(ami_srv.CalculateAssemblyInstructions,f'{mng_str}/calculate_assembly_instructions',self.calculate_assembly_instructions,callback_group=self.callback_group)      

        self.spawn_frames_from_description_srv = self.create_service(ami_srv.SpawnFramesFromDescription,f'{mng_str}/spawn_frames_from_description',self.spawn_frames_from_description,callback_group=self.callback_group) 

        self.get_frames_for_component_srv = self.create_service(ami_srv.FramesForComponent,f'{mng_str}/get_frames_for_component', self.get_frames_for_component,callback_group=self.callback_group)

        self.modify_frame_from_frame_srv = self.create_service(ami_srv.ModifyPoseFromFrame,f'{mng_str}/modify_frame_from_frame',self.modify_frame_from_frame,callback_group=self.callback_group)

        self.clear_scene_srv = self.create_service(ami_srv.ClearScene,f'{mng_str}/clear_scene', self.clear_scene,callback_group=self.callback_group)

        self.save_scene_to_file_srv = self.create_service(ami_srv.SaveSceneToFile,f'{mng_str}/save_scene_to_file', self.save_scene_to_file,callback_group=self.callback_group)

        self.load_scene_from_file_srv = self.create_service(ami_srv.LoadSceneFromFile,f'{mng_str}/load_scene_from_file', self.load_scene_from_file,callback_group=self.callback_group)

        self.reset_frame_properties_srv = self.create_service(ami_srv.ResetFrameProperties,f'{mng_str}/reset_frame_properties', self.reset_frame_properties,callback_group=self.callback_group)

        self.set_frame_properties_srv = self.create_service(ami_srv.SetFrameProperties,f'{mng_str}/set_frame_properties', self.set_frame_properties,callback_group=self.callback_group)

        self.set_component_uuid_srv = self.create_service(ami_srv.SetComponentUuid,f'{mng_str}/set_component_uuid', self.set_component_uuid,callback_group=self.callback_group)
        #self.timer = self.create_timer(5.0, self.object_scene.publish_information,callback_group=self.callback_group)
        
        self.get_logger().info("Assembly scene publisher started!")
    
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
        
        add_success = self.object_scene.add_ref_frame_to_scene(request.ref_frame)

        if request.recalculate_constraints:
            self.object_scene.update_scene_with_constraints()

        response.success = add_success

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
        new_obj.component_type_uuid = request.component_type_uuid
        new_obj.apperance_color = request.apperance_color
        new_obj.parent_frame = request.parent_frame
        new_obj.obj_pose.orientation = request.rotation
        new_obj.obj_pose.position.x = request.translation.x
        new_obj.obj_pose.position.y = request.translation.y
        new_obj.obj_pose.position.z = request.translation.z

        add_success = self.object_scene.add_obj_to_scene(new_obj)
        response.success = add_success
        
        return response
    
    def destroy_object_callback(self, request:ami_srv.DestroyObject.Request, response:ami_srv.DestroyObject.Response):
        del_success = self.object_scene.destroy_object(request.obj_name)
        response.success = del_success
        return response

    def modify_frame_absolut(self, request: ami_srv.ModifyPoseAbsolut.Request, response: ami_srv.ModifyPoseAbsolut.Response):
        modify_success = self.object_scene.modify_frame_absolut(request.frame_name, new_world_pose=request.pose)
        response.success = modify_success
        return response

    def modify_frame_relative(self, request: ami_srv.ModifyPoseRelative.Request, response: ami_srv.ModifyPoseRelative.Response):
        modify_success = self.object_scene.modify_frame_relative(request.frame_name, 
                                                                 translation=request.rel_position, 
                                                                 rotation=request.rel_rotation,
                                                                 not_relativ_to_parent_but_child=request.not_relativ_to_parent_but_child)
        response.success = modify_success
        return response
        
    def srv_create_assembly_instructions(self, request: ami_srv.CreateAssemblyInstructions.Request, response: ami_srv.CreateAssemblyInstructions.Response):
        try:
            create_success = self.object_scene.create_assembly_instructions(instruction=request.assembly_instruction)
            response.instruction_id = request.assembly_instruction.id
            response.success = create_success

        except (ComponentNotFoundError) as e:
            self.get_logger().warn(f"Components not found when creating assembly instructions '{request.assembly_instruction.id}: {e}. Check if the components exist in the scene!")
            response.success = True

        except (RefPlaneNotFoundError, ComponentNotFoundError) as e:
            self.get_logger().error(f"Error creating assembly instructions: {e}")
            response.success = False
    
        return response
    
    def calculate_assembly_instructions(self, request: ami_srv.CalculateAssemblyInstructions.Request, response: ami_srv.CalculateAssemblyInstructions.Response):
        
        try:
            transfrom = self.object_scene.get_assembly_transformation_by_id(request.instruction_id)
            
        except ValueError as e:
            self.get_logger().error(f"Error getting the assembly transformation: {e}")
            response.success = False
            return response
        
        if transfrom == None:
            response.success = False
        else:
            response.success = True
            response.assembly_transform = transfrom
        return response
    
    def modify_frame_from_frame(self, request: ami_srv.ModifyPoseFromFrame.Request, response: ami_srv.ModifyPoseFromFrame.Response):
        modify_success = self.object_scene.modify_frame_set_to_frame(frame_to_set=request.frame_to_set,
                                                                frame_to_set_to=request.from_frame)
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
        return response
    
    def save_scene_to_file(self, request: ami_srv.SaveSceneToFile.Request, response: ami_srv.SaveSceneToFile.Response):
        try:
            response.success = self.object_scene.save_scene_to_file(request.file_path)
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

    def reset_frame_properties(self, request: ami_srv.ResetFrameProperties.Request, response: ami_srv.ResetFrameProperties.Response):
        response = self.object_scene.reset_frame_properties(request)
        return response
    
    def set_frame_properties(self, request: ami_srv.SetFrameProperties.Request, response: ami_srv.SetFrameProperties.Response):
        response = self.object_scene.set_frame_properties(request)
        return response

    def set_component_uuid(self, request: ami_srv.SetComponentUuid.Request, response: ami_srv.SetComponentUuid.Response):
        response = self.object_scene.set_component_uuid(request)
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
