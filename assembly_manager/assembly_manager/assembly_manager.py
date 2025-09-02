#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String, ColorRGBA
from geometry_msgs.msg import Pose
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, StaticTransformBroadcaster
import assembly_manager_interfaces.srv as ami_srv
import assembly_manager_interfaces.msg as ami_msg
import assembly_scene_publisher.py_modules.frame_constraints as f_constraints

from rclpy.time import Duration
from threading import Event
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from threading import Thread
import os
import json
import ast
from functools import partial
import time

# import standard srv msg with empty request and response
from std_srvs.srv import Empty

class AssemblyManagerNode(Node):

    SPAWN_COMPONENT_OFFSET_X = 0.0
    SPAWN_COMPONENT_OFFSET_Y = 0.0
    SPAWN_COMPONENT_OFFSET_Z = 0.00000
    def __init__(self):
        super().__init__("assembly_manager")

        # Callbacks run simutaniously
        self.callback_group_re = ReentrantCallbackGroup()
        # Callbacks can not run simutaniously
        self.callback_group_mu_ex = MutuallyExclusiveCallbackGroup()

        # create service for obj
        self.object_topic_publisher_srv_spawn = self.create_service(ami_srv.SpawnObject,'assembly_manager/spawn_object',self.spawn_object_callback,callback_group=self.callback_group_re)
        self.object_topic_publisher_srv_destroy = self.create_service(ami_srv.DestroyObject,'assembly_manager/destroy_object',self.destroy_component_callback,callback_group=self.callback_group_re)

        # create service for spawning from description
        self.spawn_component_from_description_srv = self.create_service(ami_srv.SpawnComponentFromDescription,'assembly_manager/spawn_component_from_description',self.spawn_component_from_description_callback,callback_group=self.callback_group_re)
        self.create_assembly_instruction_from_description_srv = self.create_service(ami_srv.CreateAssemblyInstructionFromDescription,'assembly_manager/create_assembly_instruction_from_description',self.create_assembly_instruction_from_description_callback,callback_group=self.callback_group_re)

        # create client for publisher node
        self.object_topic_publisher_client_spawn = self.create_client(ami_srv.SpawnObject,'assembly_scene_publisher/spawn_object',callback_group=self.callback_group_mu_ex) 
        self.object_topic_publisher_client_destroy = self.create_client(ami_srv.DestroyObject,'assembly_scene_publisher/destroy_object',callback_group=self.callback_group_mu_ex) 

        # create client for moveit node
        self.moveit_object_spawner_client = self.create_client(ami_srv.SpawnObject,'moveit_component_spawner/spawn_object',callback_group=self.callback_group_mu_ex) 
        self.moveit_object_destroyer_client = self.create_client(ami_srv.DestroyObject,'moveit_component_spawner/destroy_object',callback_group=self.callback_group_mu_ex)   

        # Service for Spawning from Dictionary
        self.create_ref_frame_client = self.create_client(ami_srv.CreateRefFrame,'assembly_manager/create_ref_frame',callback_group=self.callback_group_mu_ex) 
        self.create_ref_axis_client = self.create_client(ami_srv.CreateAxis,'assembly_manager/create_axis',callback_group=self.callback_group_mu_ex) 
        self.create_ref_plane_client = self.create_client(ami_srv.CreateRefPlane,'assembly_manager/create_ref_plane',callback_group=self.callback_group_mu_ex) 
        self.create_assembly_instructions_client= self.create_client(ami_srv.CreateAssemblyInstructions,'assembly_manager/create_assembly_instructions',callback_group=self.callback_group_mu_ex)

        self.logger = self.get_logger()
        self.logger.info("Assembly manager started!")
        

    def destroy_component(self, request: ami_srv.DestroyObject.Request)->bool:
        self.logger.info('Destroy component request received!')
        moveit_destroy_executed = None
        object_destroy_executed =  None
        request_forwarding = ami_srv.DestroyObject.Request()
        request_forwarding.obj_name = request.obj_name 
        call_async = False

        if not self.object_topic_publisher_client_destroy.wait_for_service(timeout_sec=2.0):
            self.logger.info('Destroy component service not available')
            object_destroy_executed =  False
        
        if object_destroy_executed is None:
            # Spawning part in topic publisher
            if call_async:
                future = self.object_topic_publisher_client_destroy.call_async(request_forwarding)
                while not future.done():
                    rclpy.spin_once(self)   
                object_destroy_executed =future.result().success
            else:
                response = self.object_topic_publisher_client_destroy.call(request_forwarding)
                object_destroy_executed = response.success

        # spawning part in moveit
        if not self.moveit_object_destroyer_client.wait_for_service(timeout_sec=2.0):
            moveit_destroy_executed = False
        
        if moveit_destroy_executed is None:
            if call_async:
                future = self.moveit_object_destroyer_client.call_async(request_forwarding)
                while not future.done():
                    rclpy.spin_once(self)
                moveit_destroy_executed =future.result().success
            else:
                result = self.moveit_object_destroyer_client.call(request_forwarding)
                moveit_destroy_executed = result.success


        success = object_destroy_executed and moveit_destroy_executed

        return success
    
    def destroy_component_callback(self, request: ami_srv.DestroyObject.Request, response: ami_srv.DestroyObject.Response):
        response.success = self.destroy_component(request)
        return response
    
    def spawn_object_callback(self, request: ami_srv.SpawnObject.Request, response: ami_srv.SpawnObject.Response):

        response.success = self.spawn_component(request, call_async = False)
        
        return response

    # def spawn_component(self, SpawnRequest: ami_srv.SpawnObject.Request, call_async = False)->bool:

    #     self.logger.info('Spawn component request received!')
    #     object_publish_executed =  None
    #     moveit_spawner_executed =  None
    #     unity_spawner_executed =  None
    #     object_publish_success = False
    #     moveit_spawner_success = False
    #     unity_spawner_success = False

    #     SpawnRequest.translation.x = SpawnRequest.translation.x + self.SPAWN_COMPONENT_OFFSET_X
    #     SpawnRequest.translation.y = SpawnRequest.translation.y + self.SPAWN_COMPONENT_OFFSET_Y
    #     SpawnRequest.translation.z = SpawnRequest.translation.z + self.SPAWN_COMPONENT_OFFSET_Z

    #     if not self.object_topic_publisher_client_spawn.wait_for_service(timeout_sec=2.0):
    #         self.logger.info('Spawn Service not available')
    #         object_publish_executed =  False
        
    #     if object_publish_executed is None:
    #         if call_async:
    #             future = self.object_topic_publisher_client_spawn.call_async(SpawnRequest)
    #             while not future.done():
    #                 rclpy.spin_once(self)
    #             object_publish_success=future.result().success
    #         else:
    #             response = self.object_topic_publisher_client_spawn.call(SpawnRequest)
    #             object_publish_success = response.success

    #     self.logger.info(f"Object publish success: {object_publish_success}. Gazebo running: {self.is_gazebo_running()}")

    #     # spawing part in unity
    #     if object_publish_success and not self.is_gazebo_running():
    #         SpawnRequestUnity = unity_srv.SpawnObjectUnity.Request()
    #         for key in SpawnRequest.__slots__:
    #             setattr(SpawnRequestUnity, key, getattr(SpawnRequest, key))
            
    #         if not self.unity_object_spawner_client.wait_for_service(timeout_sec=2.0):
    #             self.logger.info('Spawn Service not available')
    #             unity_spawner_executed = False
            
    #         if unity_spawner_executed is None:
    #             response_unity = self.unity_object_spawner_client.call(SpawnRequestUnity)
    #             unity_spawner_success = response_unity.success
    #     else:
    #         unity_spawner_success = True

    #     # spawning part in moveit
    #     if object_publish_success:
    #         if not self.moveit_object_spawner_client.wait_for_service(timeout_sec=2.0):
    #             self.logger.info('Spawn Service not available')
    #             moveit_spawner_executed =  False
            
    #         if moveit_spawner_executed is None:
    #             if call_async:
    #                 future = self.moveit_object_spawner_client.call_async(SpawnRequest)
    #                 while not future.done():
    #                     rclpy.spin_once(self)
    #                 moveit_spawner_success=future.result().success
    #             else:
    #                 response = self.moveit_object_spawner_client.call(SpawnRequest)
    #                 moveit_spawner_success = response.success

    #     self.logger.info(f"Object publish success: {object_publish_success}, Moveit spawner success: {moveit_spawner_success}, Unity spawner success: {unity_spawner_success}")

    #     # Destroy object from publisher if spawn in moveit failed
    #     if not moveit_spawner_success or not unity_spawner_success:
    #         request_destroy = ami_srv.DestroyObject.Request()
    #         request_destroy.obj_name=SpawnRequest.obj_name

    #         if not self.object_topic_publisher_client_destroy.wait_for_service(timeout_sec=2.0):
    #             self.logger.info('Destroy Service not available')
    #             return False
            
    #         if call_async:  
    #             future = self.object_topic_publisher_client_destroy.call_async(request_destroy)
    #             while not future.done():
    #                 rclpy.spin_once(self)
    #             destroy_success = future.result().success
    #         else:
    #             response = self.object_topic_publisher_client_destroy.call(request_destroy)
    #             destroy_success = response.success

    #         if (destroy_success):
    #             self.logger.error('Object was spawned in publisher, but failed to spawn in Moveit. Object was deleted from publisher! Service call ignored!')
        
    #     return (object_publish_success and moveit_spawner_success and unity_spawner_success)
    
    
    
    def spawn_component(self, SpawnRequest: ami_srv.SpawnObject.Request, call_async = False)->bool:

        self.logger.info('Spawn component request received!')
        object_publish_executed =  None
        object_publish_success = False

        SpawnRequest.translation.x = SpawnRequest.translation.x + self.SPAWN_COMPONENT_OFFSET_X
        SpawnRequest.translation.y = SpawnRequest.translation.y + self.SPAWN_COMPONENT_OFFSET_Y
        SpawnRequest.translation.z = SpawnRequest.translation.z + self.SPAWN_COMPONENT_OFFSET_Z

        if not self.object_topic_publisher_client_spawn.wait_for_service(timeout_sec=2.0):
            self.logger.info('Spawn Service not available')
            object_publish_executed =  False
        
        if object_publish_executed is None:
            if call_async:
                future = self.object_topic_publisher_client_spawn.call_async(SpawnRequest)
                while not future.done():
                    rclpy.spin_once(self)
                object_publish_success=future.result().success
            else:
                response = self.object_topic_publisher_client_spawn.call(SpawnRequest)
                object_publish_success = response.success

        self.logger.info(f"Object publish success: {object_publish_success}. Gazebo running: {self.is_gazebo_running()}")
        
        return object_publish_success

    def spawn_component_from_description_callback(self, request: ami_srv.SpawnComponentFromDescription.Request, response: ami_srv.SpawnComponentFromDescription.Response):
        response.success = self.spawn_component_from_description(request)
        return response
        
    def spawn_component_from_description(self, request: ami_srv.SpawnComponentFromDescription.Request, component_name_override = None)->bool:
        self.logger.info("Spawning component from description!")
        try:
            file_data = None
            # Load the JSON data from the file
            with open(request.file_path, 'r') as file:
                file_data = json.load(file)

        except FileNotFoundError:
            self.logger.error(f"Error: File not found at path '{request.file_path}'.")
            return False

        except json.JSONDecodeError as e:
            self.logger.error(f"Error decoding JSON: {e}")
            return False

        except Exception as e:
            self.logger.error(f"An unexpected error occurred: {e}")
            return False
        try:
            if component_name_override is not None:
                comp_name = component_name_override
            elif request.component_name_override != "":
                comp_name = request.component_name_override
            else:
                comp_name = file_data.get("name")
            description = file_data.get("description")
            guid = file_data.get("guid")
            type = file_data.get("type")
            save_date = file_data.get("saveDate")
            cad_path = file_data.get("cadPath")
            cad_data_collision = file_data.get("cadPathCollision", "")

            if cad_data_collision == "":
                cad_data_collision = cad_path

            #self.logger.error(f"Collision: {cad_data_collision}")

            mounting_references = file_data.get("mountingDescription").get("mountingReferences")
            ref_frames = mounting_references.get("ref_frames")
            ref_axis = mounting_references.get("ref_axes")
            ref_planes = mounting_references.get("ref_planes")
            doc_units = file_data.get("documentUnits")
            if doc_units == "mm":
                multiplier = 0.001
            elif doc_units == "cm":
                multiplier = 0.01
            elif doc_units == "um":
                multiplier = 0.000001
            else:
                multiplier = 1
            
            color = ColorRGBA()
            color.r = file_data.get("color",{}).get("R", 0.0)
            color.g = file_data.get("color",{}).get("G", 0.0)
            color.b = file_data.get("color",{}).get("B", 0.0)
            spawn_request = ami_srv.SpawnObject.Request()
            spawn_request.obj_name = comp_name
            spawn_request.apperance_color = color
            spawn_request.parent_frame = mounting_references.get("spawningOrigin")
            spawn_request.cad_data = f"{os.path.dirname(request.file_path)}/{cad_path}"
            spawn_request.cad_data_collision = f"{os.path.dirname(request.file_path)}/{cad_data_collision}"
            spawn_request.translation.x = mounting_references.get("spawningTransformation").get("translation").get("X")*multiplier
            spawn_request.translation.y = mounting_references.get("spawningTransformation").get("translation").get("Y")*multiplier
            spawn_request.translation.z = mounting_references.get("spawningTransformation").get("translation").get("Z")*multiplier
            spawn_request.rotation.w = mounting_references.get("spawningTransformation").get("rotation").get("W")
            spawn_request.rotation.x = mounting_references.get("spawningTransformation").get("rotation").get("X")
            spawn_request.rotation.y = mounting_references.get("spawningTransformation").get("rotation").get("Y")
            spawn_request.rotation.z = mounting_references.get("spawningTransformation").get("rotation").get("Z")
            spawn_success = self.spawn_component(spawn_request)

            if not spawn_success:
                self.logger.error(f"Error while spawning component {spawn_request.obj_name}!")
                return False

            self.logger.info(f"Start processing ref frames for '{spawn_request.obj_name}'...")
            # This is needed for tf to update
            time.sleep(1.5)
            
            num_of_ref_frames = len(ref_frames)
            # go through the dictionary
            for ind, ref_frame in enumerate(ref_frames):
                create_ref_frame_request = ami_srv.CreateRefFrame.Request()
                create_ref_frame_request.recalculate_constraints = False
                create_ref_frame_request.ref_frame.frame_name = f'{comp_name}_{ref_frame.get("name")}'
                create_ref_frame_request.ref_frame.parent_frame = comp_name
                create_ref_frame_request.ref_frame.pose.position.x = ref_frame.get("transformation").get("translation").get("X")*multiplier
                create_ref_frame_request.ref_frame.pose.position.y = ref_frame.get("transformation").get("translation").get("Y")*multiplier
                create_ref_frame_request.ref_frame.pose.position.z = ref_frame.get("transformation").get("translation").get("Z")*multiplier
                create_ref_frame_request.ref_frame.pose.orientation.w = ref_frame.get("transformation").get("rotation").get("W")
                create_ref_frame_request.ref_frame.pose.orientation.x = ref_frame.get("transformation").get("rotation").get("X")
                create_ref_frame_request.ref_frame.pose.orientation.y = ref_frame.get("transformation").get("rotation").get("Y")
                create_ref_frame_request.ref_frame.pose.orientation.z = ref_frame.get("transformation").get("rotation").get("Z")
                constraint_dict = ref_frame.get("constraints",{})
                #constraint_dict['units'] = doc_units
                #constraint_dict = {"constraints": constraint_dict}
                #create_ref_frame_request.ref_frame.constraints_dict = str(constraint_dict)

                frame_constraint_handler = f_constraints.FrameConstraintsHandler.return_handler_from_dict(dictionary=constraint_dict,
                                                                                                          component_name=comp_name,
                                                                                                          logger = self.logger)
                frame_constraint_handler.unit = doc_units
                
                msg = frame_constraint_handler.return_as_msg()
                                
                create_ref_frame_request.ref_frame.constraints = msg
                
                # after the last frame, recalculate the constraints
                if ind == num_of_ref_frames - 1:
                    create_ref_frame_request.recalculate_constraints = True
                    self.logger.error(f" !!!!!!!!!!!!!!!!!!!!   Recalculation of constraints triggered for {create_ref_frame_request.ref_frame.frame_name}!")

                spawn_ref_frame_success = self.create_ref_frame(create_ref_frame_request)

                if not spawn_ref_frame_success:
                    self.logger.error(f"Error while spawning ref frame {create_ref_frame_request.ref_frame.frame_name}!")
                    return False
                
            self.logger.info(f"Start processing ref axis for '{spawn_request.obj_name}'...")    
            for axis in ref_axis:
                create_axis_request = ami_srv.CreateAxis.Request()
                create_axis_request.axis.axis_name = f'{comp_name}_{axis.get("name")}'
                for index, frame in enumerate(axis.get("refPointNames")):
                    if not frame == "":
                        frame = f'{comp_name}_{frame}'
                        create_axis_request.axis.point_names[index] = frame
                spawn_axis_success = self.create_ref_axis(create_axis_request)
                if not spawn_axis_success:
                    self.logger.error(f"Error while spawning axis {create_axis_request.axis.axis_name}!")
                    return False
                
            self.logger.info(f"Start processing ref planes for '{spawn_request.obj_name}'...")    
            for plane in ref_planes:
                create_plane_request = ami_srv.CreateRefPlane.Request()
                create_plane_request.ref_plane.ref_plane_name = f'{comp_name}_{plane.get("name")}'
                for index, frame in enumerate(plane.get("refPointNames")):
                    if not frame == "":
                        frame = f'{comp_name}_{frame}'
                        create_plane_request.ref_plane.point_names[index] = frame
                for index, axis in enumerate(plane.get("refAxisNames")):
                    if not axis == "":
                        axis = f'{comp_name}_{axis}'
                        create_plane_request.ref_plane.axis_names[index] = axis
                create_plane_request.ref_plane.ideal_norm_vector.x = plane.get("normalVector").get("X")
                create_plane_request.ref_plane.ideal_norm_vector.y = plane.get("normalVector").get("Y")
                create_plane_request.ref_plane.ideal_norm_vector.z = plane.get("normalVector").get("Z")
                spawn_plane_success = self.create_ref_plane(create_plane_request)
                if not spawn_plane_success:
                    self.logger.error(f"Error while spawning plane {create_plane_request.ref_plane.ref_plane_name}!")
                    return False
            
            return True
        except Exception as e:
            self.logger.error(f"Error while spawning component '{comp_name}' from description: {e}")
            return False
   
    def create_assembly_instruction_from_description_callback(self, request: ami_srv.CreateAssemblyInstructionFromDescription.Request, response: ami_srv.CreateAssemblyInstructionFromDescription.Response):
        self.logger.info("Creating assembly instructions!")
        try:
            # Load the JSON data from the file
            with open(request.file_path, 'r') as file:
                file_data = json.load(file)

        except FileNotFoundError:
            self.logger.error(f"Error: File not found at path '{request.file_path}'.")
            response.success = False
            return response

        except json.JSONDecodeError as e:
            self.logger.error(f"Error decoding JSON: {e}")
            response.success = False
            return response

        except Exception as e:
            self.logger.error(f"An unexpected error occurred: {e}")
            response.success = False
            return response
        
        try:
            type = file_data.get("type")
            mounting_description = file_data.get("mountingDescription")

            # Check if file is an assembly file
            if type != "Assembly":
                self.logger.error(f"Error: File is not an assembly file!")
                response.success = False
                return response
            
            # Get document units
            doc_units = file_data.get("documentUnits")
            if doc_units == "mm":
                multiplier = 0.001
            elif doc_units == "cm":
                multiplier = 0.01
            elif doc_units == "um":
                multiplier = 0.000001
            else:
                multiplier = 1

            # Spawn components
            if request.spawn_components:
                for component in file_data.get("mountingDescription").get("components"):
                    component_name = component.get("name")
                    directory, filename = os.path.split(request.file_path)
                    component_path = os.path.join(directory.replace("assemblies", "components"), f"{component_name[:-2]}.json")
                    request = ami_srv.SpawnComponentFromDescription.Request()
                    #self.logger.debug(f"Spawning component from description: {component_path}")
                    request.file_path = component_path
                    spawn_success = self.spawn_component_from_description(request, component_name_override = component_name)
                    if not spawn_success:
                        self.logger.error(f"Error while spawning component from description!")
                        response.success = False
                        return response
                    
            # Create assembly instructions
            assembly_constraints = mounting_description.get("assemblyConstraints")

            for constraint in assembly_constraints:
                component_1 = constraint.get("component_1")
                component_2 = constraint.get("component_2")
                create_assembly_instruction_request = ami_srv.CreateAssemblyInstructions.Request()
                create_assembly_instruction_request.assembly_instruction.id =                                       constraint.get("name")
                response.instruction_id = create_assembly_instruction_request.assembly_instruction.id
                create_assembly_instruction_request.assembly_instruction.component_1_is_moving_part =               constraint.get("moveComponent_1")
                create_assembly_instruction_request.assembly_instruction.plane_match_1.plane_name_component_1 =     f'{component_1}_{constraint.get("description").get("planeMatch_1").get("planeNameComponent_1")}'
                create_assembly_instruction_request.assembly_instruction.plane_match_1.plane_name_component_2 =     f'{component_2}_{constraint.get("description").get("planeMatch_1").get("planeNameComponent_2")}'
                create_assembly_instruction_request.assembly_instruction.plane_match_1.plane_offset =               constraint.get("description").get("planeMatch_1").get("planeOffset") * multiplier
                create_assembly_instruction_request.assembly_instruction.plane_match_1.inv_normal_vector =          constraint.get("description").get("planeMatch_1").get("invertNormalVector")
                create_assembly_instruction_request.assembly_instruction.plane_match_2.plane_name_component_1 =     f'{component_1}_{constraint.get("description").get("planeMatch_2").get("planeNameComponent_1")}'
                create_assembly_instruction_request.assembly_instruction.plane_match_2.plane_name_component_2 =     f'{component_2}_{constraint.get("description").get("planeMatch_2").get("planeNameComponent_2")}'
                create_assembly_instruction_request.assembly_instruction.plane_match_2.plane_offset =               constraint.get("description").get("planeMatch_2").get("planeOffset") * multiplier
                create_assembly_instruction_request.assembly_instruction.plane_match_2.inv_normal_vector =          constraint.get("description").get("planeMatch_2").get("invertNormalVector")
                create_assembly_instruction_request.assembly_instruction.plane_match_3.plane_name_component_1 =     f'{component_1}_{constraint.get("description").get("planeMatch_3").get("planeNameComponent_1")}'
                create_assembly_instruction_request.assembly_instruction.plane_match_3.plane_name_component_2 =     f'{component_2}_{constraint.get("description").get("planeMatch_3").get("planeNameComponent_2")}'
                create_assembly_instruction_request.assembly_instruction.plane_match_3.plane_offset =               constraint.get("description").get("planeMatch_3").get("planeOffset") * multiplier
                create_assembly_instruction_request.assembly_instruction.plane_match_3.inv_normal_vector =          constraint.get("description").get("planeMatch_3").get("invertNormalVector")
                create_assembly_instruction_request.assembly_instruction.component_1 = component_1
                create_assembly_instruction_request.assembly_instruction.component_2 = component_2
                create_success = self.create_assembly_instructions(create_assembly_instruction_request)
                if not create_success:
                    self.logger.error(f"Error while creating assembly instruction {create_assembly_instruction_request.assembly_instruction.id}!")
                    # commented out to make possible that only some of the instructions are created
                    response.success = False
                    #return response

            response.success = True
        except Exception as e:
            self.logger.error(f"Error while creating assembly instructions from description: {e}")
            response.success = False
        finally:    
            return response
        
    def create_ref_frame(self, request: ami_srv.CreateRefFrame.Request)->bool:
        if not self.create_ref_frame_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Create Reference Frame Service not available')
            return False
        
        response = self.create_ref_frame_client.call(request)
        return response.success

    def create_ref_axis(self, request: ami_srv.CreateAxis.Request)->bool:
        if not self.create_ref_axis_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Create Reference Axis Service not available!')
            return False
      
        response = self.create_ref_axis_client.call(request)
        return response.success

    def create_ref_plane(self, request: ami_srv.CreateRefPlane.Request)->bool:
        if not self.create_ref_plane_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Create Reference Frame Service not available!')
            return False
        
        response = self.create_ref_plane_client.call(request)

        return response.success

    def create_assembly_instructions(self, request: ami_srv.CreateAssemblyInstructions.Request)->bool:
        if not self.create_assembly_instructions_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Create Assembly Instruction Service not available!')
            return False

        response = self.create_assembly_instructions_client.call(request)

        return response.success

    def is_gazebo_running(self):
        """Check if the Gazebo node is active."""
        node_names = self.get_node_names()
        if 'gazebo' in node_names:
            return True
        return False

    def is_unity_running(self)->bool:
        """Check if the Unity node is active."""
        node_names = self.get_node_names()
        if 'ROS2UnityCam1Publisher' in node_names:
            return True
        return False
    
def main(args=None):
    rclpy.init(args=args)
    node = AssemblyManagerNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
