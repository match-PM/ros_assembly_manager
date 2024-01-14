#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, StaticTransformBroadcaster

import assembly_manager_interfaces.srv as ami_srv
import assembly_manager_interfaces.msg as ami_msg

from threading import Event
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from threading import Thread
import os
import json
import ast
from functools import partial

class InvalidInputDict(Exception):
    pass

def findkeys(node, kv):
    if isinstance(node, list):
        for i in node:
            for x in findkeys(i, kv):
               yield x
    elif isinstance(node, dict):
        if kv in node:
            yield node[kv]
        for j in node.values():
            for x in findkeys(j, kv):
                yield x

class AssemblyManagerNode(Node):
    def __init__(self):
        super().__init__("assembly_manager")

        # Callbacks run simutaniously
        self.callback_group_re = ReentrantCallbackGroup()
        # Callbacks can not run simutaniously
        self.callback_group_mu_ex = MutuallyExclusiveCallbackGroup()

        # create service for obj
        self.object_topic_publisher_srv_spawn = self.create_service(ami_srv.SpawnObject,'assembly_manager/spawn_object',self.spawn_object_callback,callback_group=self.callback_group_re)
        self.object_topic_publisher_srv_destroy = self.create_service(ami_srv.DestroyObject,'assembly_manager/destroy_object',self.destroy_object_callback,callback_group=self.callback_group_re)
        
        # create client for publisher node
        self.object_topic_publisher_client_spawn = self.create_client(ami_srv.SpawnObject,'assembly_scene_publisher/spawn_object',callback_group=self.callback_group_re) 
        self.object_topic_publisher_client_destroy = self.create_client(ami_srv.DestroyObject,'assembly_scene_publisher/destroy_object',callback_group=self.callback_group_re) 

        self.moveit_object_spawner_client = self.create_client(ami_srv.SpawnObject,'moveit_component_spawner/spawn_object',callback_group=self.callback_group_re) 
        self.moveit_object_destroyer_client = self.create_client(ami_srv.DestroyObject,'moveit_component_spawner/destroy_object',callback_group=self.callback_group_re)    

        # Service for Spawning from Dictionary
        self.object_spawn_from_dict = self.create_service(ami_srv.SpawnFromDict,'assembly_manager/spawn_from_dict',self.spawn_from_dict,callback_group=self.callback_group_re)
        self.create_ref_frame_client = self.create_client(ami_srv.CreateRefFrame,'assembly_manager/create_ref_frame',callback_group=self.callback_group_re) 

        self.logger = self.get_logger()

        self.logger.info("Assembly manager started!")


    def destroy_object_callback(self, request: ami_srv.DestroyObject.Request, response: ami_srv.DestroyObject.Response):
        self.logger.info('Destroy component request received')
        moveit_destroy_executed = None
        object_destroy_executed =  None
        request_forwarding = ami_srv.DestroyObject.Request()
        request_forwarding.obj_name         = request.obj_name 

        if not self.object_topic_publisher_client_destroy.wait_for_service(timeout_sec=2.0):
            self.logger.info('Destroy component service not available')
            object_destroy_executed =  False
        
        if object_destroy_executed is None:
            # Spawning part in topic publisher
            result = self.object_topic_publisher_client_destroy.call(request_forwarding)
            #rclpy.spin_until_future_complete(self, future)
            #result = self.future.result()
            object_destroy_executed =result.success
            #object_publish_executed = bool (result.success)

        self.logger.info('test')

        # spawning part in moveit
        if not self.moveit_object_destroyer_client.wait_for_service(timeout_sec=2.0):
            self.logger.info('Spawn Service not available')
            moveit_destroy_executed =  False
        
        if moveit_destroy_executed is None:
            result = self.moveit_object_destroyer_client.call(request_forwarding)
            moveit_destroy_executed = result.success

        response.success = object_destroy_executed and moveit_destroy_executed

        return response
    
    async def spawn_object_callback(self, request: ami_srv.SpawnObject.Request, response: ami_srv.SpawnObject.Response):

        self.logger.info('Spawn Object Service received!')
        
        # depricated
        # if not self.object_topic_publisher_client_spawn.wait_for_service(timeout_sec=2.0):
        #     self.logger.info('Spawn Service not available')
        #     object_publish_executed =  False
        
        # if object_publish_executed is None:
        #     # Spawning part in topic publisher
        #     result = self.object_topic_publisher_client_spawn.call(request_forwarding)
        #     #rclpy.spin_until_future_complete(self, future)
        #     #result = self.future.result()
        #     object_publish_success=result.success
        #     #object_publish_executed = bool (result.success)

        # # spawning part in moveit
        # if object_publish_success:
        #     if not self.moveit_object_spawner_client.wait_for_service(timeout_sec=2.0):
        #         self.logger.info('Spawn Service not available')
        #         moveit_spawner_executed =  False
            
        #     if moveit_spawner_executed is None:
        #         result = self.moveit_object_spawner_client.call(request_forwarding)
        #         moveit_spawner_success = result.success
        
        # # Destroy object from publisher if spawn in moveit failed
        # if not moveit_spawner_success:
        #     request_destroy = DestroyObject.Request()
        #     request_destroy.obj_name=request.obj_name
        #     result_destory = self.object_topic_publisher_client_destroy.call(request_destroy)
        #     if (result_destory.success):
        #         self.logger.error('Object was spawned in publisher, but failed to spawn in Moveit. Object was deleted from publisher! Service call ignored!')
            

        response.success = self.spawn_object_function (request)

        return response

    def spawn_object_function(self, SpwnRequest: ami_srv.SpawnObject.Request)->bool:

        self.logger.info('Spawn Object Service received!')
        object_publish_executed =  None
        moveit_spawner_executed =  None
        object_publish_success = False
        moveit_spawner_success = False

        if not self.object_topic_publisher_client_spawn.wait_for_service(timeout_sec=2.0):
            self.logger.info('Spawn Service not available')
            object_publish_executed =  False
        
        if object_publish_executed is None:
            self.logger.info('Object handler called')
            # Spawning part in topic publisher
            result = self.object_topic_publisher_client_spawn.call(SpwnRequest)
            #rclpy.spin_until_future_complete(self, future)
            #result = self.future.result()
            object_publish_success=result.success
            #object_publish_executed = bool (result.success)

        # spawning part in moveit
        if object_publish_success:
            if not self.moveit_object_spawner_client.wait_for_service(timeout_sec=2.0):
                self.logger.info('Spawn Service not available')
                moveit_spawner_executed =  False
            
            if moveit_spawner_executed is None:
                self.logger.info('Moveit Spawn called')
                result = self.moveit_object_spawner_client.call(SpwnRequest)
                moveit_spawner_success = result.success

        # Destroy object from publisher if spawn in moveit failed
        if not moveit_spawner_success:
            request_destroy = ami_srv.DestroyObject.Request()
            request_destroy.obj_name=SpwnRequest.obj_name
            result_destory = self.object_topic_publisher_client_destroy.call(request_destroy)
            if (result_destory.success):
                self.logger.error('Object was spawned in publisher, but failed to spawn in Moveit. Object was deleted from publisher! Service call ignored!')
        
        return (object_publish_success and moveit_spawner_success)

    def spawn_from_dict(self, request: ami_srv.SpawnFromDict.Request, response: ami_srv.SpawnFromDict.Response):
        dictionarys_to_spawn = self.process_input_dict(request.dict)
        objects_to_spawn, ref_frames_to_spawn= self.convert_dict_list_to_msg_list(dictionarys_to_spawn)

        for req in objects_to_spawn:

            SpawnObject_Req = ami_srv.SpawnObject.Request()
            SpawnObject_Res = ami_srv.SpawnObject.Response()
            SpawnObject_Req.obj_name = req['obj_name']
            SpawnObject_Req.parent_frame = req['spawning_frame']
            SpawnObject_Req.cad_data = req['cad_data']
            try:
                SpawnObject_Req.translation.x = 0.0
                SpawnObject_Req.translation.y = 0.0
                SpawnObject_Req.translation.z = 0.0
                SpawnObject_Req.rotation.w = 1.0
                SpawnObject_Req.rotation.x = 0.0
                SpawnObject_Req.rotation.y = 0.0
                SpawnObject_Req.rotation.z = 0.0
            except:
                self.logger.info('translation and rotation not specified in dictionary')
                SpawnObject_Req.translation.x = 0.0
                SpawnObject_Req.translation.y = 0.0
                SpawnObject_Req.translation.z = 0.0
                SpawnObject_Req.rotation.w = 1.0
                SpawnObject_Req.rotation.x = 0.0
                SpawnObject_Req.rotation.y = 0.0
                SpawnObject_Req.rotation.z = 0.0

            success = self.spawn_object_function(SpawnObject_Req)
            self.logger.warn(str(success))
            SpawnObject_Res.success = success
        
        for req in ref_frames_to_spawn:

            CreateRefFrame_Req = ami_srv.CreateRefFrame.Request()
            CreateRefFrame_Req.frame_name = req['frame_name']
            CreateRefFrame_Req.parent_frame = req['parent_frame']
            CreateRefFrame_Req.pose.position.x = req['pose']['x']
            CreateRefFrame_Req.pose.position.y = req['pose']['y']
            CreateRefFrame_Req.pose.position.z = req['pose']['z']
            CreateRefFrame_Req.pose.orientation.w= req['pose']['qw']
            CreateRefFrame_Req.pose.orientation.x= req['pose']['qw']
            CreateRefFrame_Req.pose.orientation.y= req['pose']['qw']
            CreateRefFrame_Req.pose.orientation.z= req['pose']['qw']
            
            if not self.create_ref_frame_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().error('Create Reference Frame Service not available')
                response.success = False
                return response
            
            future = self.create_ref_frame_client.call_async(CreateRefFrame_Req)
            future.add_done_callback(partial(self.callback_set))

        response.success = True

        return response
        
    def process_input_dict(self,input_data: str) -> list[dict]:
        try:
            if not input_data:
                return []
                
            if os.path.isfile(input_data):
                try:
                    with open(input_data, 'r') as f:
                        input_data = json.load(f)
                    self.logger.info("json file read successfully!")
                except:
                    self.logger.error("json file could not be read! Syntax error in given file!")
                    raise InvalidInputDict
            
            if isinstance(input_data,str):
                input_data = ast.literal_eval(input_data)

            if isinstance(input_data, dict):
                input_data = [input_data]  # Convert a dictionary to a list containing that dictionary

            if not isinstance(input_data, list):
                raise InvalidInputDict ("Input is not a list or dictionary")
            
            self.logger.info("Dictionary processed successfully!")
            return input_data
        
        except InvalidInputDict as t:
            self.logger.error("Spawning dictionary aborted! Error (1) in Dict Input!")
            self.logger.error(str(t))
            return []
        except BaseException as e:
            #print(str(e))
            self.logger.error("Error (2) in dict input! Opening file failed or conversiion to dict/list failed!")
            return []

    def convert_dict_list_to_msg_list(self, list_data: list[dict]):
        objects_to_spawn_msg_list = []

        ref_frames_to_spawn_msg_list = []

        # it is assumed if the input is not a list that the input is not valid
        # If an error accures in the previous function the p. function will not return a list
        if not isinstance(list_data, list):
            return [],[]
        
        try:
            for index, entry in enumerate(list_data):
                object_name = None
                # Processing objects
                try:
                    new_object={'obj_name': entry['Spawn']['object_name'],
                                        'spawning_frame':entry['Spawn']['spawning_frame'],
                                        'cad_data':entry['Spawn']['cad_data']
                                        }
                    objects_to_spawn_msg_list.append(new_object)

                    object_name = entry['Spawn']['object_name']
                except:
                    self.logger.info("Dict only contains ref_frames or error in spawn object definition!")
                
                # Processing ref_frames 
                ref_frames = list(findkeys(entry,'ref_frames'))

                if len(ref_frames):
                    for frame in ref_frames[0].items():    
                        key,value = frame

                        # Check if parent frame is given for ref_frame
                        try:
                            ref_frame_parent = value['parent_frame']
                        except:
                            if object_name is None:
                                self.logger.error("Ref_frame is missing a parent frame!")
                                raise Exception
                            self.logger.info("Parent frame for ref frame not given, assuming object as parent")
                            ref_frame_parent = object_name

                        new_ref_frame={'frame_name': key,
                                                'parent_frame': ref_frame_parent,
                                                'pose':{
                                                    'x':float(value['pose']['x']),
                                                    'y':float(value['pose']['y']),
                                                    'z':float(value['pose']['z']),
                                                    'qw':float(value['pose']['qw']),
                                                    'qx':float(value['pose']['qx']),
                                                    'qy':float(value['pose']['qy']),
                                                    'qz':float(value['pose']['qz'])}
                                                }
                        
                        ref_frames_to_spawn_msg_list.append(new_ref_frame)
            
            return objects_to_spawn_msg_list, ref_frames_to_spawn_msg_list
    
        except Exception as e:
            self.logger.error(e)
            self.logger.error("Error occured! Error in syntax for ref frames!")


    def callback_set(self,future):
        try:
            response = future.result()
            self.get_logger().info("Service call successfull.")
        except Exception as e:
            self.get_logger().error("error")





def main(args=None):
    rclpy.init(args=args)
    node = AssemblyManagerNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
