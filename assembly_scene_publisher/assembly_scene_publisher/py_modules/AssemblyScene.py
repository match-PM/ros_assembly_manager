import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, StaticTransformBroadcaster
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
import numpy as np
import json
import assembly_manager_interfaces.msg as ami_msg
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Vector3, Quaternion
import sympy as sp
from typing import Union
from ament_index_python.packages import get_package_share_directory
import os
# import plt
import matplotlib.pyplot as plt
from assembly_scene_publisher.py_modules.AssemblySceneAnalyzer import AssemblySceneAnalyzer
from scipy.optimize import minimize, least_squares
from assembly_scene_publisher.py_modules.frame_constraints import (FrameConstraintsHandler, 
                                                                    update_ref_frame_by_constraint, 
                                                                    build_frame_reference_tree,
                                                                    calculate_constraints_for_component,
                                                                    calculate_frame_contrains_for_frame_list,
                                                                    calculate_constraints_for_scene,
                                                                    get_identification_order)
from copy import deepcopy,copy
from assembly_scene_publisher.py_modules.scene_functions import (get_parent_frame_for_ref_frame,
                                                                 get_ref_frame_by_name,
                                                                 is_frame_from_scene,
                                                                 get_axis_from_scene,
                                                                 get_plane_from_scene,
                                                                 get_component_for_frame_name,
                                                                 get_frames_for_plane,
                                                                 get_frame_names_from_list,
                                                                 get_frames_for_axis,
                                                                 get_frames_for_planes_of_component,
                                                                 get_component_by_name,
                                                                 get_plane_intersection_from_scene_num)


from assembly_scene_publisher.py_modules.geometry_functions import (get_point_of_plane_intersection, 
                                                                    get_euler_rotation_matrix, 
                                                                    quaternion_multiply, 
                                                                    matrix_multiply_vector, 
                                                                    norm_vec_direction, 
                                                                    calc_angle_between_vectors)

from assembly_scene_publisher.py_modules.geometry_type_functions import (vector3_to_matrix1x3,
                                                                            point3D_to_vector3,
                                                                            check_and_return_quaternion,
                                                                            get_point_from_ros_obj,
                                                                            get_transform_matrix_from_tf,
                                                                            get_rotation_matrix_from_tf,
                                                                            transform_matrix_to_pose,
                                                                            euler_to_quaternion,
                                                                            get_euler_angles_from_roatation_matrix,
                                                                            SCALE_FACTOR)

from assembly_scene_publisher.py_modules.tf_functions import (adapt_transform_for_new_parent_frame,
                                                              get_transform_for_frame_in_world,
                                                              get_transform_for_frame,
                                                                get_plane_from_frame_names,
                                                                get_line3d_from_frame_names,
                                                                publish_transform_tf_static,
                                                                transform_vector3_to_world,
                                                                substract_vectors)

import datetime
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.set_message import set_message_fields
import json
import os

def vec_to_um(v)-> str:
    return f"(x={v.x * 1e6:.3f}, y={v.y * 1e6:.3f}, z={v.z * 1e6:.3f}) µm"

class AssemblyManagerScene():
    UNUSED_FRAME_CONST = 'unused_frame'
    def __init__(self, node: Node):
        self.scene:ami_msg.ObjectScene = ami_msg.ObjectScene()
        self.assembly_scene_analyzer = AssemblySceneAnalyzer(self.scene, node.get_logger())
        self.node = node
        self.callback_group_pub = ReentrantCallbackGroup()
        self._scene_publisher = node.create_publisher(ami_msg.ObjectScene,'/assembly_manager/scene',10,callback_group=self.callback_group_pub)
        self.timer = node.create_timer(5.0, self.publish_information,callback_group=self.callback_group_pub)
        self.logger = node.get_logger()
        self.tf_broadcaster = StaticTransformBroadcaster(node)
        #self.tf_broadcaster = TransformBroadcaster(node)
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, node,spin_thread=True)
    
    def publish_information(self):
        self.publish_scene()
        self.publish_to_tf()

    def publish_scene(self):
        #self.logger.warn("Object Scene has been published")
        self._scene_publisher.publish(self.scene)
        #self.logger.info("Object Scene has been published")

    def add_obj_to_scene(self, new_obj:ami_msg.Object)-> bool:

        if new_obj.obj_name == "":
            self.logger.error(f"Name of the component should not be empty. Aboarted!")
            return False
        
        name_conflict, _ = self.check_ref_frame_exists(new_obj.obj_name)

        if name_conflict:
            self.logger.error(f'Object can not have the same name as an existing reference frame!')
            return False

        parent_frame_exists = self.check_if_frame_exists(new_obj.parent_frame)
            
        if not parent_frame_exists:
            self.logger.warn(f"Tried to spawn object {new_obj.obj_name}, but parent frame '{new_obj.parent_frame}' does not exist!")
            return False

        obj_existend = self.check_object_exists(new_obj.obj_name)

        if not obj_existend:
            self.scene.objects_in_scene.append(new_obj)
            self.publish_scene()
            self.publish_to_tf()
            return True

        # if exists updated values
        else:
            for index, obj in enumerate(self.scene.objects_in_scene):
                obj:ami_msg.Object
                if obj.obj_name == new_obj.obj_name:
                    new_obj.obj_pose.orientation = check_and_return_quaternion(new_obj.obj_pose.orientation,self.logger)
                    obj.cad_data = new_obj.cad_data
                    obj.obj_pose = new_obj.obj_pose
                    obj.parent_frame = new_obj.parent_frame

                    self.logger.warn(f'Service for spawning {new_obj.obj_name} was called, but object does already exist!')
                    self.logger.warn(f'Information for {new_obj.obj_name} updated!')
                    self.publish_scene()
                    self.publish_to_tf()
                    return True
            # eventually 
            return False

    def add_ref_frame_to_scene(self, new_ref_frame:ami_msg.RefFrame)-> bool:

        if new_ref_frame is None:
            self.node.get_logger().warn(f"Frame is None.")
            return False
        
        # checks if ref frame frame exists
        ref_frame_existend, parent_frame = self.check_ref_frame_exists(new_ref_frame.frame_name)

        # checks if object with same name exists
        name_conflict_1 = self.check_object_exists(new_ref_frame.frame_name)

        # checks if tf frame exists
        name_conflict_2 = self.check_if_frame_exists(new_ref_frame.frame_name)

        if not ref_frame_existend and (name_conflict_1 or name_conflict_2):
            self.logger.error(f"Ref frame '{new_ref_frame.frame_name}' can not have the same name as an existing reference frame or object!")
            return False

        parent_frame_exists = self.check_if_frame_exists(new_ref_frame.parent_frame)
            
        if not parent_frame_exists:
            self.logger.warn(f'Tried to spawn ref frame {new_ref_frame.frame_name}, but parent frame {new_ref_frame.parent_frame} does not exist!')
            return False

        # Check if the new ref frame should be connected to an existing object or not.
        frame_is_obj_frame = self.check_object_exists(new_ref_frame.parent_frame)

        frame_list_to_append_to = []

        if frame_is_obj_frame:
            frame_list_to_append_to = self.get_obj_by_name(new_ref_frame.parent_frame).ref_frames
        else:
            frame_list_to_append_to = self.scene.ref_frames_in_scene

        if not ref_frame_existend:
            frame_list_to_append_to.append(new_ref_frame)
        else:
            if parent_frame != new_ref_frame.parent_frame:
                if frame_is_obj_frame:
                    frame_list_to_delete = self.get_obj_by_name(parent_frame).ref_frames
                else:
                    frame_list_to_delete = self.scene.ref_frames_in_scene
                for index, frame in enumerate(frame_list_to_delete):
                    frame: ami_msg.RefFrame
                    if frame.frame_name == new_ref_frame.frame_name:
                        del frame_list_to_delete[index]
                        self.logger.warn(f'Service for creating {new_ref_frame.frame_name} was called, but frame does already exist! Information for {new_ref_frame.frame_name} updated!')
        
            for index, frame in enumerate(frame_list_to_append_to):
                frame: ami_msg.RefFrame
                if frame.frame_name == new_ref_frame.frame_name:
                    del frame_list_to_append_to[index]
                    self.logger.warn(f'Service for creating {new_ref_frame.frame_name} was called, but frame does already exist! Information for {new_ref_frame.frame_name} updated!')
                
            frame_list_to_append_to.append(new_ref_frame)

        self.publish_information()
        return True

    def add_ref_frames_to_scene_from_dict(self, ref_frames_dict:dict)-> bool:
        """
        This function adds multiple ref frames to the scene from a dictionary.
        """

        list_of_ref_frames = ref_frames_dict.get('frames', [])
        document_units = ref_frames_dict.get('document_units', 'm')
        unique_identifier = ref_frames_dict.get('unique_identifier', '')
        
        multiplicator = 1
        if document_units == 'mm':
            multiplicator = 1000
        elif document_units == 'um':
            multiplicator = 1000000
        elif document_units == 'm':
            multiplicator = 1
        else:
            self.logger.error(f"Document units '{document_units}' not supported! Frames could not be added!")
            return False

        try:
            for index, frame in enumerate(list_of_ref_frames):
                new_ref_frame = ami_msg.RefFrame()
                new_ref_frame.frame_name = f"{unique_identifier}{frame.get('name', 'NO_NAME')}"
                new_ref_frame.parent_frame = frame.get('parent_frame', "")
                x = frame["transformation"]["translation"]["X"]/multiplicator
                y = frame["transformation"]["translation"]["Y"]/multiplicator
                z = frame["transformation"]["translation"]["Z"]/multiplicator
                position = Point()
                position.x = x
                position.y = y
                position.z = z
                new_ref_frame.pose.position = position
                r_x = frame["transformation"]["rotation"]["X"]
                r_y = frame["transformation"]["rotation"]["Y"]
                r_z = frame["transformation"]["rotation"]["Z"]
                r_w = frame["transformation"]["rotation"]["W"]
                orientation = Quaternion()
                orientation.x = r_x
                orientation.y = r_y
                orientation.z = r_z
                orientation.w = r_w
                new_ref_frame.pose.orientation = orientation 
                
                constraints_dict = frame.get('constraints', {})
                
                frame_constraint_handler = FrameConstraintsHandler.return_handler_from_dict(dictionary=constraints_dict,
                                                                                            unique_identifier = unique_identifier,
                                                                                            logger = self.logger)
                
                frame_constraint_handler.unit = document_units
                msg = frame_constraint_handler.return_as_msg()
                new_ref_frame.constraints = msg

                add_success = self.add_ref_frame_to_scene(new_ref_frame)

                if not add_success:
                    self.logger.error(f"Ref frame {new_ref_frame} could not be created!")
                    return False
            
            self.update_scene_with_constraints()
            return True 
        
        except Exception as e:
            self.logger.error(f"Error while adding ref frames to scene from dictionary: {str(e)}")
            return False
    
    def destroy_object(self, obj_id:str)-> bool:
        """
        This method will delete an object from the objects list
        """

        for index, obj in enumerate(self.scene.objects_in_scene):
            obj:ami_msg.Object
            if obj.obj_name == obj_id:
                # change the parent frame for the ref frames connected to the object. This is necessary because the ref frame would reapear if a new obj with the same name would be spawned. 
                for ref_frame in obj.ref_frames:
                    ref_frame:ami_msg.RefFrame
                    ref_frame.parent_frame = self.UNUSED_FRAME_CONST
                # publish the changes from the ref frames
                self.publish_information()

                # destroy TF !!!! A static TF cant be destroyed. Instead it is detached from the world.
                publish_transform_tf_static(node= self.node,
                                            tf_broadcaster= self.tf_broadcaster,
                                            child_frame=obj.obj_name,
                                            parent_frame=self.UNUSED_FRAME_CONST,
                                            translation=obj.obj_pose.position,
                                            rotations=obj.obj_pose.orientation)
                                
                del self.scene.objects_in_scene[index]

                self.publish_information()
                self.logger.info(f'{obj_id} destroyed!!!')
                return True
            
        self.logger.error(f'Tried to destroy {obj_id}, but object does not exist!')
        return False
    
    def destroy_ref_frame(self, frame_id:str)-> bool:

        # Check if frame is refevert to an object or not.
        for index, frame in enumerate(self.scene.ref_frames_in_scene):
            frame: ami_msg.RefFrame
            if frame.frame_name == frame_id:
                publish_transform_tf_static(node= self.node,
                                            tf_broadcaster= self.tf_broadcaster,
                                            child_frame=frame.frame_name,
                                            parent_frame='unused_frame',
                                            translation=frame.pose.position,
                                            rotations=frame.pose.orientation)
                del self.scene.ref_frames_in_scene[index]
                self.logger.info(f"Frame '{frame_id}' destroyed!")
                return True
            
        for obj in self.scene.objects_in_scene:
            obj: ami_msg.Object
            for index, frame in enumerate(obj.ref_frames):
                frame: ami_msg.RefFrame
                if frame.frame_name == frame_id:
                    publish_transform_tf_static(node= self.node,
                            tf_broadcaster= self.tf_broadcaster,
                            child_frame=frame.frame_name,
                            parent_frame='unused_frame',
                            translation=frame.pose.position,
                            rotations=frame.pose.orientation)
                    del self.scene.ref_frames_in_scene[index]
                    self.logger.info(f'Frame {frame_id} destroyed!')
                    return True
        
        self.logger().error(f"Frame '{frame_id}' could not be deleted! Frame does not exist!")
        return False
 
    def get_obj_by_name(self, obj_name:str) -> ami_msg.Object:
        """
        Returns the object from the objects list by the given obj name
        """
        for obj in self.scene.objects_in_scene:
            obj:ami_msg.Object
            if obj.obj_name == obj_name:
                return obj
        return None

    def change_obj_parent_frame(self, obj_id: str, new_parent_frame:str) -> bool :
        if obj_id == new_parent_frame:
            self.logger.error(f'Parent and child frame can not be the same! parent_frame = child_frame ')
            return False
        
        new_parent_frame_exists = self.check_if_frame_exists(new_parent_frame)
        if not new_parent_frame_exists:
            self.logger.error(f"The given parent frame '{new_parent_frame}' does not exist! Frame could not be changed!")
            return False
        
        obj_to_change = self.get_obj_by_name(obj_name=obj_id)   # returns not if not found

        # if obj is not None
        if obj_to_change is not None:
            if obj_to_change.parent_frame == new_parent_frame:
                self.logger.warn(f'Parent frame is already set!')
                return True
            new_trans, new_rot = adapt_transform_for_new_parent_frame(  child_frame=obj_to_change.obj_name,
                                                                        new_parent_frame=new_parent_frame,
                                                                        tf_buffer=self.tf_buffer)
            obj_to_change.obj_pose.position.x = new_trans.x
            obj_to_change.obj_pose.position.y = new_trans.y
            obj_to_change.obj_pose.position.z = new_trans.z
            obj_to_change.obj_pose.orientation = new_rot
            obj_to_change.parent_frame = new_parent_frame
            self.logger.info(f'Parent Frame updated!') 
            self.publish_information()
            return True
        else:
            self.logger.error(f'Given obj_id is not an existing object!')
            return False
            
    def publish_to_tf(self):
        # Create a static transform
        for ref_frame in self.scene.ref_frames_in_scene:
            transform = TransformStamped()
            ref_frame:ami_msg.RefFrame
            transform.header.stamp = self.node.get_clock().now().to_msg()
            transform.header.frame_id = ref_frame.parent_frame
            transform.child_frame_id = ref_frame.frame_name
            transform.transform.rotation = ref_frame.pose.orientation
            transform.transform.translation.x=ref_frame.pose.position.x
            transform.transform.translation.y=ref_frame.pose.position.y
            transform.transform.translation.z=ref_frame.pose.position.z
            self.tf_broadcaster.sendTransform(transform)

        for obj in self.scene.objects_in_scene:
            obj: ami_msg.Object
            transform_stamped = TransformStamped()
            transform_stamped.header.stamp = self.node.get_clock().now().to_msg()
            transform_stamped.child_frame_id = obj.obj_name
            transform_stamped.header.frame_id = obj.parent_frame
            transform_stamped.transform.translation.x = obj.obj_pose.position.x
            transform_stamped.transform.translation.y = obj.obj_pose.position.y
            transform_stamped.transform.translation.z = obj.obj_pose.position.z
            transform_stamped.transform.rotation = obj.obj_pose.orientation
            self.tf_broadcaster.sendTransform(transform_stamped)

        for obj in self.scene.objects_in_scene:
            obj: ami_msg.Object
            for ref_frame in obj.ref_frames:
                transform = TransformStamped()
                ref_frame:ami_msg.RefFrame
                transform.header.stamp = self.node.get_clock().now().to_msg()
                transform.header.frame_id = ref_frame.parent_frame
                transform.child_frame_id = ref_frame.frame_name
                transform.transform.rotation = ref_frame.pose.orientation
                transform.transform.translation.x=ref_frame.pose.position.x
                transform.transform.translation.y=ref_frame.pose.position.y
                transform.transform.translation.z=ref_frame.pose.position.z
                self.tf_broadcaster.sendTransform(transform)

    def check_if_frame_exists(self, frame_id:str) -> bool:
        """ This function checks if a tf exists in the tf buffer"""
        try:
            self.tf_buffer.lookup_transform("world", frame_id, rclpy.time.Time())
            return True
        except Exception as e:
            print(e)
            return False

    def check_object_exists(self,name_new_obj:str) -> bool:
        # this function checks if an object is in the objects list
        for obj in self.scene.objects_in_scene:
            obj: ami_msg.Object
            if obj.obj_name == name_new_obj:
                return True
        return False
    
    def check_ref_frame_exists(self,name_frame:str) -> Union[bool,str]:
        """
        This function checks if an frame esixts in the frame list. If it exists it returns True and the parent frame. 
        If it does not exist it returns False and None.
        """
        # this function checks if an frame esixts in the frame list
        # iterate over ref_frames
        for ref_frame in self.scene.ref_frames_in_scene:
            ref_frame:ami_msg.RefFrame
            if ref_frame.frame_name == name_frame:
                return True, None
        
        # iterate over objects
        for obj in self.scene.objects_in_scene:
            obj: ami_msg.Object
            # iterate over ref_frames in object
            for obj_ref_frame in obj.ref_frames:
                obj_ref_frame:ami_msg.RefFrame
                if obj_ref_frame.frame_name == name_frame:
                    return True, obj.obj_name
                
        return False, None

    def create_axis(self, axis: ami_msg.Axis) -> bool:

        try:
            if axis.axis_name == "":
                self.logger.error(f"Name of the axis should not be empty. Aboarted!")
                return False
            
            if not len(axis.point_names) == 2:
                self.logger.error(f"Not enough input arguments. Plane could not be created!")
                return False
            
            parent_frame_1 = get_parent_frame_for_ref_frame(self.scene, axis.point_names[0], logger=self.logger)
            parent_frame_2 = get_parent_frame_for_ref_frame(self.scene, axis.point_names[1], logger=self.logger)

            if (not (parent_frame_1 == parent_frame_2) or 
                parent_frame_1 is None or 
                parent_frame_2 is None):
                self.logger.error(f"Given frames do not have the same parent frame or ref frame does not exist. Axis could not be created!")
                return False

            # Check if frames form a valid plane
            try:
                test_axis:sp.Line3D = get_line3d_from_frame_names(axis.point_names, tf_buffer=self.tf_buffer)
            except ValueError as e:
                self.logger.error(f"Given frames do not form a axis. Axis could not be created!")
                self.logger.error(str(e))
                return False

            list_to_append_axis= []

            # try to get parent object of ref frame 
            obj = self.get_obj_by_name(parent_frame_1)
            
            if obj is None:
                list_to_append_axis = self.scene.axis_in_scene
            else:
                list_to_append_axis = obj.ref_axis

            for ind, _axis in enumerate(list_to_append_axis):
                _axis:ami_msg.Axis
                if _axis.axis_name == axis.axis_name:
                    list_to_append_axis[ind] = axis
                    return True
            
            # if above for loop executes without returning append the plane because it does not yet excist.
            list_to_append_axis.append(axis)
            self.logger.debug(f"Axis '{axis.axis_name}' created! Axis is defined by frames 1.{axis.point_names[0]}, 2.{axis.point_names[1]}.")
            return True

        except Exception as e:
            self.logger.error(e)
            self.logger.error(f"Plane could not be created. Invalid message!")
            return False
        
    def create_ref_plane(self, plane: ami_msg.Plane) -> bool:
        try:
            if plane.ref_plane_name == "":
                self.logger.error(f"Name of the plane should not be empty. Aboarted!")
                return False

            if (plane.axis_names[0]=='' and
                plane.point_names[0]!='' and 
                plane.point_names[1]!='' and 
                plane.point_names[2]!=''):
                mode = 'PlanePPP'
                parent_frame_1 = get_parent_frame_for_ref_frame(self.scene, plane.point_names[0], logger=self.logger)
                parent_frame_2 = get_parent_frame_for_ref_frame(self.scene, plane.point_names[1], logger=self.logger)
                parent_frame_3 = get_parent_frame_for_ref_frame(self.scene, plane.point_names[2], logger=self.logger)

            elif (plane.axis_names[0]!='' and
                plane.point_names[0]!='' and 
                plane.point_names[1]=='' and 
                plane.point_names[2]==''):
                mode = 'PlaneAP'
                axis_msg = get_axis_from_scene(scene=self.scene, 
                                               axis_name=plane.axis_names[0])
                parent_frame_1 = get_parent_frame_for_ref_frame(self.scene, axis_msg.point_names[0], logger=self.logger)
                parent_frame_2 = get_parent_frame_for_ref_frame(self.scene, axis_msg.point_names[1], logger=self.logger)
                parent_frame_3 = get_parent_frame_for_ref_frame(self.scene, plane.point_names[0], logger=self.logger)
            else:
                self.logger.error(f"Invalid input for creation of reference plane. Plane should be defined by 3 x frames or by 1 x axis + 1 x frame!")
                return False
            
            if (not (parent_frame_1 == parent_frame_2 == parent_frame_3) or 
                parent_frame_1 is None or 
                parent_frame_2 is None or 
                parent_frame_3 is None):
                self.logger.error(f"Given frames do not have the same parent frame or ref frame does not exist. Plane could not be created!")
                return False

            # Check if frames form a valid plane
            try:
                if mode == 'PlanePPP':
                    test_plane: sp.Plane = get_plane_from_frame_names(plane.point_names, tf_buffer=self.tf_buffer, logger=self.logger)
                    logger_message = f"Plane '{plane.ref_plane_name}' created! Plane is defined by frames 1.{plane.point_names[0]}, 2.{plane.point_names[1]}, 3.{plane.point_names[2]}."
                else:
                    test_plane: sp.Plane = self.get_plane_from_axis_and_frame(plane.axis_names[0],plane.point_names[0])
                    logger_message = f"Plane '{plane.ref_plane_name}' created! Plane is defined by axis '{plane.axis_names[0]}' and point '{plane.point_names[0]}'."

            except ValueError as e:
                self.logger.error(f"Given frames do not form a valid plane. Plane '{plane.ref_plane_name}' could not be created! Error {e}")
                return False

            list_to_append_plane = []

            # try to get parent object of ref frame 
            obj = self.get_obj_by_name(parent_frame_1)
            
            if obj is None:
                list_to_append_plane = self.scene.planes_in_scene
            else:
                list_to_append_plane = obj.ref_planes

            for ind, _plane in enumerate(list_to_append_plane):
                _plane:ami_msg.Plane
                if _plane.ref_plane_name == plane.ref_plane_name:
                    list_to_append_plane[ind] = plane
                    return True
            
            # if above for loop executes without returning append the plane because it does not yet excist.
            list_to_append_plane.append(plane)
            self.logger.debug(logger_message)
            return True

        except Exception as e:
            self.logger.error(e)
            self.logger.error(f"Plane could not be created. Invalid message!")
            return False
        
    # def update_all_ref_frame_constraints(self):
    #     """
    #     This function updates all ref frame constraints.
    #     """
    #     self.logger.debug(f"Update all ref frame constraints")
    #     for obj in self.scene.objects_in_scene:
    #         obj: ami_msg.Object
    #         for ref_frame in obj.ref_frames:
    #             ref_frame:ami_msg.RefFrame
    #             update_ref_frame_by_constraint(scene=self.scene, ref_frame=ref_frame, component_name=obj.obj_name, logger=self.logger)
                
    #     for ref_frame in self.scene.ref_frames_in_scene:
    #         ref_frame: ami_msg.RefFrame
    #         update_ref_frame_by_constraint(scene=self.scene, ref_frame=ref_frame, logger=self.logger)
    
    def modify_pose(self,frame_obj_name:str, rel_pose: Pose)-> bool:
        # Dep
        pose_to_modify: Pose = None

        for obj in self.scene.objects_in_scene:
            obj: ami_msg.Object
            if obj.obj_name == frame_obj_name:
                pose_to_modify = obj.obj_pose
                break
            for ref_frame in obj.ref_frames:
                ref_frame:ami_msg.RefFrame
                if ref_frame.frame_name == frame_obj_name:
                    pose_to_modify = ref_frame.pose
                    break
        for ref_frame in self.scene.ref_frames_in_scene:
            ref_frame:ami_msg.RefFrame
            if ref_frame.frame_name == frame_obj_name:
                pose_to_modify = ref_frame.pose
                break
        
        if not pose_to_modify is None:
            pose_to_modify.position.x += rel_pose.position.x
            pose_to_modify.position.y += rel_pose.position.y
            pose_to_modify.position.z += rel_pose.position.z
            pose_to_modify.orientation = quaternion_multiply(pose_to_modify.orientation,rel_pose.orientation)
            self.publish_information()
            self.logger.info(f'Pose for object {frame_obj_name} updated!')
            return True
        else:
            self.logger.warn(f"Pose could not be updated. Frame '{frame_obj_name}' not found!")
            return False

    def modify_frame_relative(self,frame_name:str, 
                              translation: Point, 
                              rotation: Quaternion,
                              not_relativ_to_parent_but_child:bool = False)-> bool:
        #self.logger.warn(f"Pose of frame '{frame_name}' will be updated relative to its parent frame!")
        pose_to_modify: Pose = None

        if not self.check_if_frame_exists(frame_name):
            self.logger.error(f"Error modifing frame position. Frame '{frame_name}' does not exist")
            return False
        
        # Find the parent Frame
        parent_frame = get_parent_frame_for_ref_frame(scene=self.scene, frame_name=frame_name, logger=self.logger)
        pose_to_modify = get_ref_frame_by_name(self.scene, frame_name, logger=self.logger).pose
        
        component_name = get_component_for_frame_name(self.scene, frame_name)
        
        if parent_frame is None:
            self.logger.error(f"Pose of frame '{frame_name}' will be updated relative to the world frame!")
            return False
        
        if not_relativ_to_parent_but_child:
            # Find the parent Frame
            #self.logger.error(f"Pose of frame '{frame_name}' will be updated relative to its child frame!")
            parent__T__child = get_transform_matrix_from_tf(pose_to_modify)
            transform = Pose()
            transform.position.x = translation.x
            transform.position.y = translation.y
            transform.position.z = translation.z
            transform.orientation = rotation
            child__T__new_frame = get_transform_matrix_from_tf(transform)
            partent__T__new_frame = parent__T__child * child__T__new_frame
            new_pose = transform_matrix_to_pose(partent__T__new_frame)
            pose_to_modify.position.x = new_pose.position.x
            pose_to_modify.position.y = new_pose.position.y
            pose_to_modify.position.z = new_pose.position.z
            pose_to_modify.orientation = new_pose.orientation
        else:
            pose_to_modify.position.x += translation.x
            pose_to_modify.position.y += translation.y
            pose_to_modify.position.z += translation.z
            pose_to_modify.orientation = quaternion_multiply(pose_to_modify.orientation,rotation)
            
        if component_name is not None:
            calculate_constraints_for_component(scene=self.scene,
                                component_name=component_name,
                                logger=self.logger)
            self.publish_information()
        else:
            self.update_scene_with_constraints()
        
        self.logger.info(f'Pose for object {frame_name} updated!')
        return True


    def modify_frame_absolut(self,frame_name:str, new_world_pose: Pose)-> bool:
        # Give pose in world coordinates
        pose_to_modify: Pose = Pose()
        if not self.check_if_frame_exists(frame_name):
            self.logger.error(f"Error modifing frame position. Frame '{frame_name}' does not exist")
            return False
        
        # Find the parent Frame
        parent_frame = get_parent_frame_for_ref_frame(self.scene, frame_name , logger=self.logger)
        
        #self.logger(f"{pose_to_modify},{parent_frame}")
        if  parent_frame is not None:
        
            # Transform of point in world       
            transform_global_parent:sp.Matrix = get_transform_matrix_from_tf(get_transform_for_frame_in_world(parent_frame, self.tf_buffer, logger=self.logger))
            
            parent_frame = get_transform_for_frame_in_world(parent_frame, self.tf_buffer, logger=self.logger)
            
            #self.logger.warn(f"Parent frame: {parent_frame}")

            #self.logger.warn(f"New pose: {new_world_pose}")
            
            new_pose_frame:sp.Matrix = get_transform_matrix_from_tf(new_world_pose)

            # Compute the new transform
            new_transform:sp.Matrix = transform_global_parent.inv()*new_pose_frame

            #self.logger.warn(f"New transform: {new_transform}")

            pose_to_modify = transform_matrix_to_pose(new_transform)

            #self.logger.warn(f"Pose to modify: {pose_to_modify}")

            #self.logger.info(f'Frame {frame_name} updated!') 
            
            frame = get_ref_frame_by_name(self.scene, frame_name, logger=self.logger)

            # only modify the position !!!!
            frame.pose.position = pose_to_modify.position

            # right now there is an error with the orientation. Beside that, the question is, how we would like to handle the orientation (should it actually be modified).
            # frame.pose = pose_to_modify
            
            component_name = get_component_for_frame_name(self.scene, frame_name)
            
            if component_name is not None:
                calculate_constraints_for_component(scene=self.scene,
                                    component_name=component_name,
                                    logger=self.logger)
                self.publish_information()
            else:
                self.update_scene_with_constraints()
                
            return True
        else:
            return False

    def modify_frame_set_to_frame(self, frame_to_set: str, from_frame: str)-> bool:
        
        if not is_frame_from_scene(self.scene, frame_to_set):
            self.logger.error(f"Frame '{frame_to_set}' does not exist in the scene!")
            return False
        
        frame_parent = get_parent_frame_for_ref_frame(self.scene, frame_to_set, logger=self.logger)
        
        # Transform of point in world 
        try:      
            transform = get_transform_for_frame(from_frame, frame_parent, self.tf_buffer, logger=self.logger)
        except ValueError as e:
            self.logger.error(f"Frame '{from_frame}' does not exist in the tf buffer!")
            return False
               
        ref_frame = get_ref_frame_by_name(self.scene, frame_to_set, logger=self.logger)
        
        ref_frame.pose.position.x = transform.transform.translation.x
        ref_frame.pose.position.y = transform.transform.translation.y
        ref_frame.pose.position.z = transform.transform.translation.z
        ref_frame.pose.orientation.x = transform.transform.rotation.x
        ref_frame.pose.orientation.y = transform.transform.rotation.y
        ref_frame.pose.orientation.z = transform.transform.rotation.z
        ref_frame.pose.orientation.w = transform.transform.rotation.w
          
        component_name = get_component_for_frame_name(self.scene, frame_to_set)
        
        if component_name is not None:
            calculate_constraints_for_component(scene=self.scene,
                                component_name=component_name,
                                logger=self.logger)
            self.publish_information()
        else:
            self.update_scene_with_constraints()
            
        return True


    def create_assembly_instructions(self,instruction: ami_msg.AssemblyInstruction)->bool:
        # Get plane msgs for object 1
        if instruction.id == "":
                self.logger.error(f"ID of the instruction shoud not be empty. Aboarted!")
                return False
            
        obj1_plane1_msg = get_plane_from_scene(scene=self.scene, plane_name=instruction.plane_match_1.plane_name_component_1)
        obj1_plane2_msg = get_plane_from_scene(scene=self.scene, plane_name=instruction.plane_match_2.plane_name_component_1)
        obj1_plane3_msg = get_plane_from_scene(scene=self.scene, plane_name=instruction.plane_match_3.plane_name_component_1)

        # Get plane msgs for object 2
        obj2_plane1_msg = get_plane_from_scene(scene=self.scene, plane_name=instruction.plane_match_1.plane_name_component_2)
        obj2_plane2_msg = get_plane_from_scene(scene=self.scene, plane_name=instruction.plane_match_2.plane_name_component_2)
        obj2_plane3_msg = get_plane_from_scene(scene=self.scene, plane_name=instruction.plane_match_3.plane_name_component_2) 

        # return false if planes do not exist in scene
        create_plane_error = False
        if (obj1_plane1_msg is None):
            self.logger.error(f"Plane 1 '{instruction.plane_match_1.plane_name_component_1}' of component 1 does not exist. Invalid input!")
            create_plane_error = True
        if (obj1_plane2_msg is None):
            self.logger.error(f"Plane 2 '{instruction.plane_match_2.plane_name_component_1}' of component 1 does not exist. Invalid input!")
            create_plane_error = True
        if (obj1_plane3_msg is None):
            self.logger.error(f"Plane 3 '{instruction.plane_match_3.plane_name_component_1}' of component 1 does not exist. Invalid input!")
            create_plane_error = True
        if (obj2_plane1_msg is None):
            self.logger.error(f"Plane 1 '{instruction.plane_match_1.plane_name_component_2}' of component 2 does not exist. Invalid input!")
            create_plane_error = True
        if (obj2_plane2_msg is None):
            self.logger.error(f"Plane 2 '{instruction.plane_match_2.plane_name_component_2}' of component 2 does not exist. Invalid input!")
            create_plane_error = True
        if (obj2_plane3_msg is None):
            self.logger.error(f"Plane 3 '{instruction.plane_match_3.plane_name_component_2}' of component 2 does not exist. Invalid input!")
            create_plane_error = True

        # return false if planes do not exist in scene
        if create_plane_error:
            self.logger.error(f"Invalid input for assembly instruction. Planes do not exist in scene!")
            return False
        
        obj_1_name = get_parent_frame_for_ref_frame(self.scene, 
                                                    get_plane_from_scene(self.scene, instruction.plane_match_1.plane_name_component_1).point_names[0],
                                                    logger=self.logger)
        obj_2_name = get_parent_frame_for_ref_frame(self.scene,
                                                    get_plane_from_scene(self.scene, instruction.plane_match_1.plane_name_component_2).point_names[0],
                                                    logger=self.logger)

        # return false if planes are linked to the same component
        if obj_1_name == obj_2_name:
            self.logger.error(f"Invalid plane selection. All given planes are linked to the same component!")
            return False
        
        try:
            transfrom = self.calculate_assembly_transformation(instruction)
        except ValueError as e:
            self.logger.error(f"{str(e)}")
            return False    
        except Exception as e:
            self.logger.error(str(e))
            self.logger.error(f"Fatal Error")
            return False
        
        inst_exists=False
        for index, _inst in enumerate(self.scene.assembly_instructions):
            _inst : ami_msg.AssemblyInstruction
            if _inst.id == instruction.id:
                del self.scene.assembly_instructions[index]
                self.scene.assembly_instructions.append(instruction)
                _inst = instruction
                inst_exists=True
                break
        if not inst_exists:
            self.scene.assembly_instructions.append(instruction)

        return True
    
    def _get_plane_obj_from_scene(self, plane_name:str, parent_frame:str = None)-> sp.Plane:
        plane_msg = get_plane_from_scene(self.scene, plane_name)
        plane_msg: ami_msg.Plane
        if (plane_msg.axis_names[0]=='' and
            plane_msg.point_names[0]!='' and 
            plane_msg.point_names[1]!='' and 
            plane_msg.point_names[2]!=''):
            
            plane = get_plane_from_frame_names(frames = plane_msg.point_names , tf_buffer=self.tf_buffer, parent_frame=parent_frame)

        elif (plane_msg.axis_names[0]!='' and
            plane_msg.point_names[0]!='' and 
            plane_msg.point_names[1]=='' and 
            plane_msg.point_names[2]==''):
            plane = self.get_plane_from_axis_and_frame(plane_msg.axis_names[0], plane_msg.point_names[0],parent_frame=parent_frame)
        else:
            plane = None

        return plane
    

    def get_assembly_instruction_by_id(self, instruction_id:str)->ami_msg.AssemblyInstruction:
        for instruction in self.scene.assembly_instructions:
            instruction: ami_msg.AssemblyInstruction
            if instruction.id == instruction_id:
                return instruction
        return None
    
    def get_assembly_transformation_by_id(self, instruction_id:str)->Pose:
        instruction = self.get_assembly_instruction_by_id(instruction_id)
        if instruction is None:
            return None
        else:
            assembly_transform = self.calculate_assembly_transformation(instruction)
            return assembly_transform
    
    def calculate_plane_intersections(self, instruction: ami_msg.AssemblyInstruction)-> tuple[Vector3, Vector3]:
        """
        This function calculates the intersection point of the given planes and returns the intersection point as a Vector3.

        """
        component_1 = get_parent_frame_for_ref_frame(self.scene,
                                                     get_plane_from_scene(self.scene, instruction.plane_match_1.plane_name_component_1).point_names[0],
                                                     logger=self.logger)
        component_2 = get_parent_frame_for_ref_frame(self.scene,
                                                     get_plane_from_scene(self.scene, instruction.plane_match_1.plane_name_component_2).point_names[0],
                                                     logger=self.logger)

        obj_1_plane_1 = self._get_plane_obj_from_scene(instruction.plane_match_1.plane_name_component_1, parent_frame=component_1)
        obj_1_plane_2 = self._get_plane_obj_from_scene(instruction.plane_match_2.plane_name_component_1, parent_frame=component_1)
        obj_1_plane_3 = self._get_plane_obj_from_scene(instruction.plane_match_3.plane_name_component_1, parent_frame=component_1)
        
        obj_2_plane_1 = self._get_plane_obj_from_scene(instruction.plane_match_1.plane_name_component_2, parent_frame=component_2)
        obj_2_plane_2 = self._get_plane_obj_from_scene(instruction.plane_match_2.plane_name_component_2, parent_frame=component_2)
        obj_2_plane_3 = self._get_plane_obj_from_scene(instruction.plane_match_3.plane_name_component_2, parent_frame=component_2)


        comp_1_mate_plane_intersection: Vector3 = point3D_to_vector3(get_point_of_plane_intersection(obj_1_plane_1, obj_1_plane_2, obj_1_plane_3))

        comp_2_mate_plane_intersection: Vector3 = point3D_to_vector3(get_point_of_plane_intersection(obj_2_plane_1, obj_2_plane_2, obj_2_plane_3))

        comp_1_mate_plane_intersection.x = comp_1_mate_plane_intersection.x/SCALE_FACTOR
        comp_1_mate_plane_intersection.y = comp_1_mate_plane_intersection.y/SCALE_FACTOR
        comp_1_mate_plane_intersection.z = comp_1_mate_plane_intersection.z/SCALE_FACTOR

        comp_2_mate_plane_intersection.x = comp_2_mate_plane_intersection.x/SCALE_FACTOR
        comp_2_mate_plane_intersection.y = comp_2_mate_plane_intersection.y/SCALE_FACTOR
        comp_2_mate_plane_intersection.z = comp_2_mate_plane_intersection.z/SCALE_FACTOR

        transform_component_1 = get_transform_for_frame_in_world(component_1, self.tf_buffer)

        transform_component_2 = get_transform_for_frame_in_world(component_2, self.tf_buffer)

        # Transform the intersection points to the world frame
        comp_1_mate_plane_intersection = matrix_multiply_vector(get_rotation_matrix_from_tf(transform_component_1), vector3_to_matrix1x3(comp_1_mate_plane_intersection))
        comp_2_mate_plane_intersection = matrix_multiply_vector(get_rotation_matrix_from_tf(transform_component_2), vector3_to_matrix1x3(comp_2_mate_plane_intersection))

        comp_1_mate_plane_intersection.x += transform_component_1.transform.translation.x
        comp_1_mate_plane_intersection.y += transform_component_1.transform.translation.y
        comp_1_mate_plane_intersection.z += transform_component_1.transform.translation.z

        comp_2_mate_plane_intersection.x += transform_component_2.transform.translation.x
        comp_2_mate_plane_intersection.y += transform_component_2.transform.translation.y
        comp_2_mate_plane_intersection.z += transform_component_2.transform.translation.z

        self.logger.debug(f"Debug-Info: Plane intersection component 1: {comp_1_mate_plane_intersection}")
        self.logger.debug(f"Debug-Info: Plane intersection component 2: {comp_2_mate_plane_intersection}")

        return (comp_1_mate_plane_intersection, comp_2_mate_plane_intersection)
    
    def calculate_assembly_transformation(self, instruction:ami_msg.AssemblyInstruction)->Pose:
        
        # calculate according to numpy
        intersection_obj_1 = get_plane_intersection_from_scene_num(self.scene,
                                                                    instruction.plane_match_1.plane_name_component_1, 
                                                                    instruction.plane_match_2.plane_name_component_1,
                                                                    instruction.plane_match_3.plane_name_component_1,
                                                                    self.logger)
        
        intersection_obj_2 = get_plane_intersection_from_scene_num(self.scene,
                                                                        instruction.plane_match_1.plane_name_component_2, 
                                                                        instruction.plane_match_2.plane_name_component_2,
                                                                        instruction.plane_match_3.plane_name_component_2)
        
        intersection_obj_1_world = transform_vector3_to_world(intersection_obj_1, self.tf_buffer, original_parent_frame=instruction.component_1)
        intersection_obj_2_world = transform_vector3_to_world(intersection_obj_2, self.tf_buffer, original_parent_frame=instruction.component_2)

        #self.logger.error(f"Numpy: {str(intersection_obj_1_world)}")
        #self.logger.error(f"Numpy: {str(intersection_obj_2_world)}")

        # calculate with sympy
        obj_1_plane_1 = self._get_plane_obj_from_scene(instruction.plane_match_1.plane_name_component_1)
        obj_1_plane_2 = self._get_plane_obj_from_scene(instruction.plane_match_2.plane_name_component_1)
        obj_1_plane_3 = self._get_plane_obj_from_scene(instruction.plane_match_3.plane_name_component_1)

        obj_1_name = get_parent_frame_for_ref_frame(self.scene,
                                                    get_plane_from_scene(self.scene, instruction.plane_match_1.plane_name_component_1).point_names[0],
                                                    logger=self.logger)
        obj_2_name = get_parent_frame_for_ref_frame(self.scene,
                                                    get_plane_from_scene(self.scene, instruction.plane_match_1.plane_name_component_2).point_names[0],
                                                    logger=self.logger)
        
        #obj_1_mate_plane_intersection: Vector3 = point3D_to_vector3(get_point_of_plane_intersection(obj_1_plane_1, obj_1_plane_2, obj_1_plane_3))
        
        obj_2_plane_1 = self._get_plane_obj_from_scene(instruction.plane_match_1.plane_name_component_2)
        obj_2_plane_2 = self._get_plane_obj_from_scene(instruction.plane_match_2.plane_name_component_2)
        obj_2_plane_3 = self._get_plane_obj_from_scene(instruction.plane_match_3.plane_name_component_2)

        #obj_2_mate_plane_intersection: Vector3 = point3D_to_vector3(get_point_of_plane_intersection(obj_2_plane_1, obj_2_plane_2, obj_2_plane_3))
        try:
            obj_1_mate_plane_intersection, obj_2_mate_plane_intersection = self.calculate_plane_intersections(instruction)
            #comparison of plane intersections
            #self.logger.error(f"Sympy: {str(obj_1_mate_plane_intersection)}")
            #self.logger.error(f"Sympy: {str(obj_2_mate_plane_intersection)}")
            result_diff_1 = substract_vectors(intersection_obj_1_world, obj_1_mate_plane_intersection)
            result_diff_2 = substract_vectors(intersection_obj_2_world, obj_2_mate_plane_intersection)
            
            threshold = 1e-8  # Define a small threshold value
            if ((abs(result_diff_1.x) > threshold or abs(result_diff_1.y) > threshold or abs(result_diff_1.z) > threshold) or 
                (abs(result_diff_2.x) > threshold or abs(result_diff_2.y) > threshold or abs(result_diff_2.z) > threshold)):
                self.logger.error(f"SEVERE DIFFERENCE IN INTERSECTION CALCULATION!! NOTIFY MAINTAINER.")
                self.logger.error(f"Difference in intersection calculation: {vec_to_um(result_diff_1)}, {vec_to_um(result_diff_2)}")

            obj_1_mate_plane_intersection = intersection_obj_1_world
            obj_2_mate_plane_intersection = intersection_obj_2_world

        except ValueError as e:
            self.logger.warn(f"Error in plane intersection calculation with sympy. Using numpy calculation. Error: '{str(e)}'")
            obj_1_mate_plane_intersection = intersection_obj_1_world
            obj_2_mate_plane_intersection = intersection_obj_2_world

        assembly_transform = Pose()

        if instruction.component_1_is_moving_part:
            moving_component = obj_1_name
            static_component = obj_2_name

        else:
            moving_component = obj_2_name
            static_component = obj_1_name

        self.logger.warn(f"\nMoving component: '{moving_component}'\nStatic component: '{static_component}'")
        
        # Get the ideal norm vectors from the plane messages
        comp_1_plane_1_ideal_norm_vector = get_plane_from_scene(self.scene, instruction.plane_match_1.plane_name_component_1).ideal_norm_vector
        comp_1_plane_2_ideal_norm_vector = get_plane_from_scene(self.scene, instruction.plane_match_2.plane_name_component_1).ideal_norm_vector
        comp_1_plane_3_ideal_norm_vector = get_plane_from_scene(self.scene, instruction.plane_match_3.plane_name_component_1).ideal_norm_vector

        comp_2_plane_1_ideal_norm_vector = get_plane_from_scene(self.scene, instruction.plane_match_1.plane_name_component_2).ideal_norm_vector
        comp_2_plane_2_ideal_norm_vector = get_plane_from_scene(self.scene, instruction.plane_match_2.plane_name_component_2).ideal_norm_vector
        comp_2_plane_3_ideal_norm_vector = get_plane_from_scene(self.scene, instruction.plane_match_3.plane_name_component_2).ideal_norm_vector

        # Transform the ideal norm vectors to the world frame
        # for component 1
        obj_1_rot_matrix = get_rotation_matrix_from_tf(self.tf_buffer.lookup_transform('world', obj_1_name,rclpy.time.Time()))
        comp_1_plane_1_ideal_norm_vector = vector3_to_matrix1x3(comp_1_plane_1_ideal_norm_vector)
        comp_1_plane_1_ideal_norm_vector = matrix_multiply_vector(matrix=obj_1_rot_matrix, vector = comp_1_plane_1_ideal_norm_vector)

        comp_1_plane_2_ideal_norm_vector = vector3_to_matrix1x3(comp_1_plane_2_ideal_norm_vector)
        comp_1_plane_2_ideal_norm_vector = matrix_multiply_vector(matrix=obj_1_rot_matrix, vector = comp_1_plane_2_ideal_norm_vector)

        comp_1_plane_3_ideal_norm_vector = vector3_to_matrix1x3(comp_1_plane_3_ideal_norm_vector)
        comp_1_plane_3_ideal_norm_vector = matrix_multiply_vector(matrix=obj_1_rot_matrix, vector = comp_1_plane_3_ideal_norm_vector)

        # for component 2
        obj_2_rot_matrix = get_rotation_matrix_from_tf(self.tf_buffer.lookup_transform('world', obj_2_name,rclpy.time.Time()))
        comp_2_plane_1_ideal_norm_vector = vector3_to_matrix1x3(comp_2_plane_1_ideal_norm_vector)
        comp_2_plane_1_ideal_norm_vector = matrix_multiply_vector(matrix=obj_2_rot_matrix, vector = comp_2_plane_1_ideal_norm_vector)

        comp_2_plane_2_ideal_norm_vector = vector3_to_matrix1x3(comp_2_plane_2_ideal_norm_vector)
        comp_2_plane_2_ideal_norm_vector = matrix_multiply_vector(matrix=obj_2_rot_matrix, vector = comp_2_plane_2_ideal_norm_vector)

        comp_2_plane_3_ideal_norm_vector = vector3_to_matrix1x3(comp_2_plane_3_ideal_norm_vector)
        comp_2_plane_3_ideal_norm_vector = matrix_multiply_vector(matrix=obj_2_rot_matrix, vector = comp_2_plane_3_ideal_norm_vector)

        #self.logger.warn(f"Ideal norm vectors ob1: {comp_1_plane_1_ideal_norm_vector}, {comp_1_plane_2_ideal_norm_vector}, {comp_1_plane_3_ideal_norm_vector}")
        #self.logger.warn(f"Ideal norm vectors obj2: {comp_2_plane_1_ideal_norm_vector}, {comp_2_plane_2_ideal_norm_vector}, {comp_2_plane_3_ideal_norm_vector}")

        # Get the normal vectors from the plane actual plane in the world frame
        bvec_obj_1_1:sp.Matrix = sp.Matrix(obj_1_plane_1.normal_vector).normalized().evalf()
        bvec_obj_1_2:sp.Matrix = sp.Matrix(obj_1_plane_2.normal_vector).normalized().evalf()
        bvec_obj_1_3:sp.Matrix = sp.Matrix(obj_1_plane_3.normal_vector).normalized().evalf()

        bvec_obj_2_1=sp.Matrix(obj_2_plane_1.normal_vector).normalized().evalf()
        bvec_obj_2_2=sp.Matrix(obj_2_plane_2.normal_vector).normalized().evalf()
        bvec_obj_2_3=sp.Matrix(obj_2_plane_3.normal_vector).normalized().evalf()

        self.logger.debug(f"All normal vectors obj 1 are: {bvec_obj_1_1.evalf()}, {bvec_obj_1_2.evalf()}, {bvec_obj_1_3.evalf}")
        self.logger.debug(f"All normal vectors obj2 are: {bvec_obj_2_1.evalf()}, {bvec_obj_2_2.evalf()}, {bvec_obj_2_3.evalf}")
        self.logger.debug(f"All normal vectors obj2 are: {comp_2_plane_1_ideal_norm_vector}, {comp_2_plane_2_ideal_norm_vector}, {comp_2_plane_3_ideal_norm_vector}")
        
        # get the multiplicator for the normal vectors calculated from the planes and match their direction to the ideal normal vectors

        mult_1 = norm_vec_direction(bvec_obj_1_1,comp_1_plane_1_ideal_norm_vector, logger=self.logger)
        mult_2 = norm_vec_direction(bvec_obj_1_2,comp_1_plane_2_ideal_norm_vector, logger=self.logger)
        mult_3 = norm_vec_direction(bvec_obj_1_3,comp_1_plane_3_ideal_norm_vector, logger=self.logger)
        mult_4 = norm_vec_direction(bvec_obj_2_1,comp_2_plane_1_ideal_norm_vector, logger=self.logger)
        mult_5 = norm_vec_direction(bvec_obj_2_2,comp_2_plane_2_ideal_norm_vector, logger=self.logger)
        mult_6 = norm_vec_direction(bvec_obj_2_3,comp_2_plane_3_ideal_norm_vector, logger=self.logger)
        
        # Check if the normal vectors should be inverted
        if instruction.plane_match_1.inv_normal_vector:
            mult_4 = -mult_4
            #self.logger.debug("Inverted normal vector for plane 1")
        if instruction.plane_match_2.inv_normal_vector:
            mult_5 = -mult_5
            #self.logger.debug("Inverted normal vector for plane 2")
        if instruction.plane_match_3.inv_normal_vector:
            mult_6 = -mult_6
            #self.logger.debug("Inverted normal vector for plane 3")

        # Multiply the normal vectors with the multiplicator
        bvec_obj_1_1 : sp.Matrix = bvec_obj_1_1 * mult_1
        bvec_obj_1_2 : sp.Matrix = bvec_obj_1_2 * mult_2
        bvec_obj_1_3 : sp.Matrix = bvec_obj_1_3 * mult_3
        bvec_obj_2_1 : sp.Matrix = bvec_obj_2_1 * mult_4
        bvec_obj_2_2 : sp.Matrix = bvec_obj_2_2 * mult_5
        bvec_obj_2_3 : sp.Matrix = bvec_obj_2_3 * mult_6

        # Stack all normal vectors together to a single basis
        basis_obj_1: sp.Matrix = sp.Matrix.hstack(bvec_obj_1_1, bvec_obj_1_2, bvec_obj_1_3)
        basis_obj_2: sp.Matrix = sp.Matrix.hstack(bvec_obj_2_1, bvec_obj_2_2, bvec_obj_2_3)         
        
        #self.logger.warn(f"Basis obj 1: {str(obj_1_mate_plane_intersection.y)}")
        # Add the translation to the assembly transformation
        obj_1_mate_plane_intersection.x += float(bvec_obj_1_1[0]*instruction.plane_match_1.plane_offset)
        obj_1_mate_plane_intersection.y += float(bvec_obj_1_1[1]*instruction.plane_match_1.plane_offset)
        obj_1_mate_plane_intersection.z += float(bvec_obj_1_1[2]*instruction.plane_match_1.plane_offset)

        obj_1_mate_plane_intersection.x += float(bvec_obj_1_2[0]*instruction.plane_match_2.plane_offset)
        obj_1_mate_plane_intersection.y += float(bvec_obj_1_2[1]*instruction.plane_match_2.plane_offset)
        obj_1_mate_plane_intersection.z += float(bvec_obj_1_2[2]*instruction.plane_match_2.plane_offset)

        obj_1_mate_plane_intersection.x += float(bvec_obj_1_3[0]*instruction.plane_match_3.plane_offset)
        obj_1_mate_plane_intersection.y += float(bvec_obj_1_3[1]*instruction.plane_match_3.plane_offset)
        obj_1_mate_plane_intersection.z += float(bvec_obj_1_3[2]*instruction.plane_match_3.plane_offset)
        #self.logger.warn(f"Basis obj 1: {str(obj_1_mate_plane_intersection.y)}")

        if instruction.component_1_is_moving_part:
            moving_component_plane_intersection = obj_1_mate_plane_intersection
            static_component_plane_intersection = obj_2_mate_plane_intersection
        else:
            moving_component_plane_intersection = obj_2_mate_plane_intersection
            static_component_plane_intersection = obj_1_mate_plane_intersection

        # Calculate the translation
        assembly_transform.position.x = float(static_component_plane_intersection.x - moving_component_plane_intersection.x)
        assembly_transform.position.y = float(static_component_plane_intersection.y - moving_component_plane_intersection.y)
        assembly_transform.position.z = float(static_component_plane_intersection.z - moving_component_plane_intersection.z)

        # Calculate the 'unideal' rotationmatrix
        if instruction.component_1_is_moving_part:
            rot_matrix = basis_obj_2 * basis_obj_1.inv()
        else:
            rot_matrix = basis_obj_1 * basis_obj_2.inv()

        det_rot_matrix= rot_matrix.det()

        #self.logger.warn(f"Rot obj 2 to 1: {rot_matrix.evalf()}")

        #self.logger.warn(f"Eigenvalues Rot: {rot_matrix.eigenvals()}")
        #self.logger.warn(f"Det Rot: {det_rot_matrix}")
               
        #if not round(det_rot_matrix, 9) == 1.0:
            #self.logger.warn(f"Invalid plane selection")
            #return False

        # Calculate the approx quaternion for rotation in euclidean space
        # get timestamp
        timestamp_before = self.node.get_clock().now()
        assembly_transform.orientation = self.calc_approx_quat_from_matrix(rot_matrix)
        # get timestamp
        timestamp_after = self.node.get_clock().now()
        time_diff = timestamp_after - timestamp_before
        #self.logger.info(f"Assembly transformation is: {assembly_transform.__str__()}")
        self.logger.info(f"""Assembly transformation is: \n
                         x: {assembly_transform.position.x},\n
                         y: {assembly_transform.position.y},\n
                         z: {assembly_transform.position.z},\n
                         w: {assembly_transform.orientation.w},\n
                         x: {assembly_transform.orientation.x},\n
                         y: {assembly_transform.orientation.y},\n
                         z: {assembly_transform.orientation.z}""")
        self.logger.info(f"Time for quaternion calculation: {time_diff.nanoseconds/1e6} ms")

        add_success = self.add_assembly_frames_to_scene(instruction.id,
                                                        moving_component,
                                                        static_component,
                                                        moving_component_plane_intersection,
                                                        static_component_plane_intersection,
                                                        assembly_transform)
        if not add_success:
            raise ValueError(f"Could not add assembly frames to scene. Aborting!")
        
        return assembly_transform
    
    def add_assembly_frames_to_scene(   self,
                                        instruction_id:str,
                                        moving_component: str, 
                                        static_component:str, 
                                        moving_component_plane_intersection:sp.Point3D, 
                                        static_component_plane_intersection:sp.Point3D, 
                                        assembly_transform: Pose)-> bool:
        assembly_frame = ami_msg.RefFrame()
        assembly_frame.frame_name = f"assembly_frame_{instruction_id}"
        assembly_frame.parent_frame = moving_component
        assembly_frame.constraints_dict = str({})
        assembly_frame.pose.position.x = float(moving_component_plane_intersection.x)
        assembly_frame.pose.position.y = float(moving_component_plane_intersection.y)
        assembly_frame.pose.position.z = float(moving_component_plane_intersection.z)
        moving_component_world_transform = get_transform_for_frame_in_world(moving_component, self.tf_buffer)
        if moving_component_world_transform is None:
            return False
        assembly_frame.pose.orientation = moving_component_world_transform.transform.rotation
        assembly_frame_matrix = get_transform_matrix_from_tf(assembly_frame.pose)
        moving_component_matrix = get_transform_matrix_from_tf(moving_component_world_transform)

        helper_pose = transform_matrix_to_pose(moving_component_matrix.inv()*assembly_frame_matrix)
        assembly_frame.pose = helper_pose
        add_assembly_frame_success = self.add_ref_frame_to_scene(assembly_frame)

        # Create frame for the static component
        target_frame = ami_msg.RefFrame()
        target_frame.frame_name = f"target_frame_{instruction_id}"
        target_frame.parent_frame = static_component
        target_frame.pose.position.x = float(static_component_plane_intersection.x)
        target_frame.pose.position.y = float(static_component_plane_intersection.y)
        target_frame.pose.position.z = float(static_component_plane_intersection.z)
        target_frame.constraints_dict = str({})
        #target_frame.pose.orientation = quaternion_multiply(assembly_transfrom.orientation, target_frame.pose.orientation)
        target_frame.pose.orientation = quaternion_multiply(assembly_transform.orientation, moving_component_world_transform.transform.rotation)
        static_component_world_transform = get_transform_for_frame_in_world(static_component, self.tf_buffer)
        if static_component_world_transform is None:    
            return False
        
        target_frame_matrix = get_transform_matrix_from_tf(target_frame.pose)
        static_component_matrix = get_transform_matrix_from_tf(static_component_world_transform)
        helper_pose_2 = transform_matrix_to_pose(static_component_matrix.inv()*target_frame_matrix, logger=self.logger)
        target_frame.pose = helper_pose_2
        add_target_frame_success = self.add_ref_frame_to_scene(target_frame)
        result = add_assembly_frame_success and add_target_frame_success

        return result
        
    def calc_approx_quat_from_matrix(self, rot_mat: sp.Matrix) -> Quaternion:
        initial_guess = np.array([0, 0, 0])
        list_of_cost  = []
        i_roll, i_pith_, i_yaw = get_euler_angles_from_roatation_matrix(rot_mat)
        
        def cost_function(params, target_matrix):
            alpha, beta, gamma = params
            rotation_matrix = get_euler_rotation_matrix(alpha,beta,gamma)
            difference = (rotation_matrix - target_matrix).norm()
            #self.logger.error(f"Difference: {difference}")
            list_of_cost.append(difference)
            return difference
        
        self.logger.warn(f"Started to calculate the assembly transformation. This could take a while...")
        max_iter = 1000

        #result = minimize(cost_function, initial_guess, args=(rot_obj2_to_obj1),  method='L-BFGS-B', tol = 1e-10, options={'maxiter': 1000})
        #tolerance = 1e-20
        tolerance = 1e-10
        result = minimize(cost_function, initial_guess, 
                          args=(rot_mat),  
                          method='Nelder-Mead', 
                          tol = tolerance, 
                          options={'maxiter': max_iter})
        
        if max_iter == result.nit:
            self.logger.info(f"Max iterations reached ({result.nit}/{max_iter}). Measurement inacurate.")
        else:
            self.logger.info(f"Iterations ran: {result.nit}")
        if result.fun >1.0:
            self.logger.warn(f"Residual error of rotation calculation is very high: {result.fun}. You should not assemble the components.")
        elif result.fun >0.5:
            self.logger.warn(f"Residual error of rotation calculation is high: {result.fun}. Make sure you identified all necessary assembly frames.")
        else:
            self.logger.info(f"Calculation has low residual error: {result.fun}! This is a good sign. You can assemble the components.")
        
        self.logger.info(f"Result (deg) is: \nalpha: {result.x[0]*(180/np.pi)}, \nbeta: {result.x[1]*(180/np.pi)}, \ngamma:{result.x[2]*(180/np.pi)}")

        roll   = result.x[2]
        pitch  = result.x[1]
        yaw    = result.x[0]
                                            
        quaternion = euler_to_quaternion(   roll   = roll,
                                            pitch  = pitch,
                                            yaw    = yaw)

        roll_diff = abs(i_roll - roll) * (180/np.pi)
        pitch_diff = abs(i_pith_ - pitch) * (180/np.pi)
        yaw_diff = abs(i_yaw - yaw) * (180/np.pi)
        
        self.logger.warn(f"Roll diff: {roll_diff}, Pitch diff: {pitch_diff}, Yaw diff: {yaw_diff}")
        
        # create a plot of the list of cost
        fig, ax = plt.subplots()
        ax.plot(list_of_cost)
        ax.set(xlabel='iteration', ylabel='cost',
               title='Cost function over iterations')
        ax.grid()
        #increase resolution of the plot
        fig.set_dpi(300)
        package_share = get_package_share_directory('assembly_scene_publisher')
        plt.savefig(f"{package_share}/cost_{self.node.get_clock().now()}.png")
        return quaternion
            
    def log_secene(self):
        self.logger.info("Objects in scene:")
        for obj in self.scene.objects_in_scene:
            obj: ami_msg.Object
            self.logger.warn(f"Object: {obj.obj_name}")
            self.logger.warn(f"Pose: {obj.obj_pose}")
            self.logger.warn(f"Ref Frames: {obj.ref_frames}")
            #self.logger.info(f"Ref Planes: {obj.ref_planes}")
            #self.logger.info(f"Ref Axes: {obj.ref_axis}")
            #self.logger.info(f"Ref Points: {obj.ref_points}")
            #self.logger.info(f"Ref Lines: {obj.ref_lines}")
            #self.logger.info(f"Ref Planes: {obj.ref_planes}")
            #self.logger.info(f"Ref Planes: {obj.ref_planes}")


    def get_plane_from_axis_and_frame(self, axis_name: str, frame_name: str, parent_frame:str = None)-> sp.Plane:
        axis_msg = get_axis_from_scene(scene=self.scene, 
                                       axis_name=axis_name)

        line3d:sp.Line3D = get_line3d_from_frame_names(axis_msg.point_names, tf_buffer=self.tf_buffer, parent_frame=parent_frame, logger=self.logger)

        if parent_frame is None:
            t_point:TransformStamped = get_transform_for_frame_in_world(frame_name, self.tf_buffer, logger=self.logger)
        else:
            t_point:TransformStamped = get_transform_for_frame(frame_name, parent_frame, self.tf_buffer, logger=self.logger)

        point = get_point_from_ros_obj(t_point.transform.translation)

        plane = sp.Plane(point, normal_vector = line3d.direction)

        return plane
    
    def update_scene_with_constraints(self):
        calculate_constraints_for_scene(self.scene, logger=self.logger)
        self.publish_information()
    
    def destroy_all_ref_frames(self):
        for obj in self.scene.objects_in_scene:
            obj: ami_msg.Object
            for frame in obj.ref_frames:
                frame: ami_msg.RefFrame
                frame.parent_frame = self.UNUSED_FRAME_CONST
                #fr = get_ref_frame_by_name(self.scene, frame.frame_name, logger=self.logger)
                #fr.parent_frame = self.UNUSED_FRAME_CONST
        
        for frame in self.scene.ref_frames_in_scene:
            frame: ami_msg.RefFrame
            frame.parent_frame = self.UNUSED_FRAME_CONST

        self.publish_to_tf()
            
    def clear_scene(self, save_data:bool = False)->bool:
        
        if save_data:
            self.logger.info("Saving scene data before clearing...")
            self.save_scene_to_files()
        
        self.destroy_all_ref_frames()
        self.scene = ami_msg.ObjectScene()
        self.publish_information()
        
        return True
        
    def get_core_frames_for_component(self, component_name:str)->list[str]:
        if not self.check_object_exists(component_name):
            self.logger.error("Object does not exist")
            return []
        
        # frames = get_frames_for_planes_of_component(self.scene, 
        #                                   component_name,
        #                                   self.logger)
        
        # dicti =  build_frame_reference_tree(self.scene, frames, self.logger)
        
        # #self.logger.warn(f"Dicti: {dicti}")

        # final_list = get_frame_names_from_list(frames)

        # suc = calculate_frame_contrains_for_frame_list(scene=self.scene, 
        #                                                frame_list=frames, 
        #                                                logger=self.logger)

        #suc = calculate_constraints_for_scene(self.scene, logger=self.logger)
        #self.publish_information()
        final_list = []
        
        component = get_component_by_name(self.scene, component_name)
        
        list_res = get_identification_order(self.scene, component.ref_frames, self.logger)
        
        # for frame in frames:
        #     frame: ami_msg.RefFrame
        #     constraints_handler = FrameConstraintsHandler()
        #     constraints_handler.set_from_msg(frame.constraints)
            
        #     fri = constraints_handler.get_frame_references()
        #     if len(fri) > 0:
        #         final_list.extend(fri)
        #     else:
        #         final_list.append(frame.frame_name)
        
        # self.logger.error(f"Addition list: {final_list}")
        
        # # frames_str = get_frame_names_from_list(frames)        
        # # delete doupliates
        
        # final_list = list(dict.fromkeys(final_list))


        
        return final_list

    def save_scene_to_file(self, file_path: str) -> bool:
        file_dict = {}
        # ensure file path exists

        if not os.path.exists(os.path.dirname(file_path)):
            return False

        file_name = f"Scene_Save_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        ordered_dict = message_to_ordereddict(self.scene)
        file_dict["scene"] = json.loads(json.dumps(ordered_dict))
        file_dict["save_time"] = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')

        self.logger.info(f"Saving scene to file {file_path}...")
        with open(os.path.join(file_path, file_name), 'w') as file:
            json.dump(file_dict, file, indent=4)

        return True
    
    def load_scene_from_file(self, file_path: str) -> bool:
        if not os.path.exists(file_path):
            self.logger.error(f"File path {file_path} does not exist!")
            return False
        
        with open(file_path, 'r') as file:
            file_dict = json.load(file)
        
        if "scene" not in file_dict:
            self.logger.error("File does not contain scene data!")
            return False
        
        scene_dict = file_dict["scene"]
        self.logger.info(f"Scene data: {scene_dict}")

        try:
            new_scene = ami_msg.ObjectScene()
            set_message_fields(new_scene, scene_dict)  # modifies in place
            self.scene = new_scene
            self.publish_information()
            self.logger.info(f"Loaded scene from file {file_path} successfully!")
            return True
        except Exception as e:
            self.logger.error(f"Failed to load scene: {e}")
            return False


    def save_scene_to_files(self):
        for obj in self.scene.objects_in_scene:
            obj: ami_msg.Object
            self.logger.info(f"Saving object {obj.obj_name} to file...")
            self.save_object_to_file(obj)
        
    def save_object_to_file(self, obj:ami_msg.Object):
        filename = f"{obj.guid}"
        folder_name = f"{obj.obj_name}"
        
        if filename is None or filename == "":
            filename = obj.obj_name + ".json"
        else:
            filename = filename + ".json"
        
        # strip '-X' endings from folder name
        folder_name = folder_name.rstrip('-0123456789')
        
        file_dir = obj.cad_data
        #strip everything after the last '/'
        file_dir = os.path.dirname(file_dir)
        file_path = os.path.join(file_dir, f"logs_{folder_name}", filename)
        
        obj_dict = {}
        
        # create folder if it does not exist
        if not os.path.exists(os.path.dirname(file_path)):
            os.makedirs(os.path.dirname(file_path))
        
        # save to json file
        with open(file_path, 'w') as file:
            json.dump(obj_dict, file, indent=4)
            