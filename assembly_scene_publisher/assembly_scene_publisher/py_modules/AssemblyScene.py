import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, StaticTransformBroadcaster
import numpy as np
import json
import assembly_manager_interfaces.msg as ami_msg
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Vector3, Quaternion
import sympy as sp
from typing import Union
from scipy.optimize import minimize, least_squares

from assembly_scene_publisher.py_modules.geometry_functions import (get_point_of_plane_intersection, 
            get_euler_rotation_matrix, quaternion_multiply, matrix_multiply_vector, norm_vec_direction)

from assembly_scene_publisher.py_modules.geometry_type_functions import (vector3_to_matrix1x3,
                                                                            point3D_to_vector3,
                                                                            check_and_return_quaternion,
                                                                            get_point_from_ros_obj,
                                                                            get_transform_matrix_from_tf,
                                                                            get_rotation_matrix_from_tf,
                                                                            transform_matrix_to_pose,
                                                                            euler_to_quaternion,
                                                                            SCALE_FACTOR)

from assembly_scene_publisher.py_modules.tf_functions import (adapt_transform_for_new_parent_frame,
                                                              get_transform_for_frame_in_world,
                                                              get_transform_for_frame,
                                                                get_plane_from_frame_names,
                                                                get_line3d_from_frame_names,
                                                                publish_transform_tf_static)

class AssemblyManagerScene():
    UNUSED_FRAME_CONST = 'unused_frame'
    def __init__(self, node: Node):
        self.scene = ami_msg.ObjectScene()
        self.node = node

        self._scene_publisher = node.create_publisher(ami_msg.ObjectScene,'/assembly_manager/scene',10)
        self.logger = node.get_logger()
        self.tf_broadcaster = StaticTransformBroadcaster(node)
        #self.tf_broadcaster = TransformBroadcaster(node)
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, node,spin_thread=True)
        self.timer = node.create_timer(5.0, self.publish_scene)

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
        self.update_all_ref_frame_constraints()
        self.publish_information()
        return True

    def add_ref_frames_to_scene_from_dict(self, ref_frames_dict:dict)-> bool:
        """
        This function adds multiple ref frames to the scene from a dictionary.
        """

        list_of_ref_frames = ref_frames_dict.get('frames', [])
        document_units = ref_frames_dict.get('document_units', 'm')
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
                new_ref_frame.frame_name = frame.get('name', "")
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
                add_success = self.add_ref_frame_to_scene(new_ref_frame)
                if not add_success:
                    self.logger.error(f"Ref frame {new_ref_frame.frame_name} could not be created!")
                    return False
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
            
            parent_frame_1 = self.get_parent_frame_for_ref_frame(axis.point_names[0])
            parent_frame_2 = self.get_parent_frame_for_ref_frame(axis.point_names[1])

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
            self.logger.info(f"Axis '{axis.axis_name}' created! Axis is defined by frames 1.{axis.point_names[0]}, 2.{axis.point_names[1]}.")
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
                parent_frame_1 = self.get_parent_frame_for_ref_frame(plane.point_names[0])
                parent_frame_2 = self.get_parent_frame_for_ref_frame(plane.point_names[1])
                parent_frame_3 = self.get_parent_frame_for_ref_frame(plane.point_names[2])

            elif (plane.axis_names[0]!='' and
                plane.point_names[0]!='' and 
                plane.point_names[1]=='' and 
                plane.point_names[2]==''):
                mode = 'PlaneAP'
                axis_msg = self.get_axis_from_scene(plane.axis_names[0])
                parent_frame_1 = self.get_parent_frame_for_ref_frame(axis_msg.point_names[0])
                parent_frame_2 = self.get_parent_frame_for_ref_frame(axis_msg.point_names[1])
                parent_frame_3 = self.get_parent_frame_for_ref_frame(plane.point_names[0])
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
            self.logger.info(logger_message)
            return True

        except Exception as e:
            self.logger.error(e)
            self.logger.error(f"Plane could not be created. Invalid message!")
            return False
        
    def update_all_ref_frame_constraints(self):
        """
        This function updates all ref frame constraints.
        """
        self.logger.debug(f"Update all ref frame constraints")
        for obj in self.scene.objects_in_scene:
            obj: ami_msg.Object
            for ref_frame in obj.ref_frames:
                ref_frame:ami_msg.RefFrame
                self.update_ref_frame_constraint(ref_frame, obj.obj_name)
        for ref_frame in self.scene.ref_frames_in_scene:
            ref_frame: ami_msg.RefFrame
            self.update_ref_frame_constraint(ref_frame, None)

    def update_ref_frame_constraint(self, ref_frame:ami_msg.RefFrame,component_name:str)-> bool:
        try:
            default_dicts = [{},{"constraints": {"units": "m"}}, {"constraints": {"units": "mm"}}, {"constraints": {"units": "um"}}, ""]         
            if ref_frame.constraints_dict is None:
                self.logger.debug(f"For ref frame '{ref_frame.frame_name}' no constraints are given (None).")
                return True
            if ref_frame.constraints_dict == "":
                self.logger.debug(f"For ref frame '{ref_frame.frame_name}' no constraints are given (EmptyString).")
                return True
            
            constraint_dict = eval(ref_frame.constraints_dict)

            if constraint_dict in default_dicts:
                self.logger.debug(f"No constraints for ref frame '{ref_frame.frame_name}' given.")
                return True

            if constraint_dict.get("constraints", []).get("centroid", []).get("refFrameNames", []) == []:
                self.logger.debug(f"No constraints for ref frame '{ref_frame.frame_name}' given.")
                return True
            
            try:

                ref_frames:list[str] = constraint_dict["constraints"]["centroid"]['refFrameNames']
                dim:str = constraint_dict["constraints"]["centroid"]['dim']
                offset_values:list[float] = constraint_dict["constraints"]["centroid"]['offsetValues']
                unit:str = constraint_dict["constraints"]['units']

                if unit == 'mm':
                    offset_values = [x/1000 for x in offset_values]
                if unit == 'um':
                    offset_values = [x/1000000 for x in offset_values]

                self.logger.debug(f"Centroid constraint for '{ref_frame.frame_name}' is given!")
                frame_names_list = []
                if component_name is not None:
                    for frame in ref_frames:
                        frame_names_list.append(f"{component_name}_{frame}")
                else:
                    frame_names_list = ref_frames
                self.logger.debug(f"Frame names list: {str(frame_names_list)}")
                if not self.check_ref_frames_for_same_parent_frame(frame_names_list):
                    self.logger.error(f"Tried to create constraint for ref frame '{ref_frame.frame_name}', but given ref frames do not have the same parent frame. Constraint could not be created!")
                    self.logger.debug(f"{frame_names_list}")
                    return False

                if ref_frame.parent_frame != self.get_parent_frame_for_ref_frame(frame_names_list[0]):
                    self.logger.error(f"Tried to create constraint for ref frame '{ref_frame.frame_name}', but given ref frames do not have the same parent frame as the ref frame. Constraint could not be created!")
                    self.logger.error(f"Parent frame of ref frame '{ref_frame.frame_name}': {ref_frame.parent_frame}")
                    self.logger.error(f"Parent frame of ref frame '{frame_names_list[0]}': {self.get_parent_frame_for_ref_frame(ref_frames[0])}")
                    return False
                
                ref_frame.pose = self.caluclate_frame_centroid(ref_frames=frame_names_list, 
                                                               dim=dim, 
                                                               offset_values=offset_values,
                                                               initial_pose=ref_frame.pose)
                self.logger.debug(f"Centroid constraint for ref frame '{ref_frame.frame_name}' created!")
            except KeyError as e:
                self.logger.error(f"KeyError: {str(e)}")

            return True
        
        
        except Exception as e:
            self.logger.error(str(e))
            self.logger.error(f"Unknown Error. Constraint could not be updated!")
            return False

    def caluclate_frame_centroid(self, ref_frames: list[str], dim: str, offset_values: list[float], initial_pose: Pose) -> Pose:
        """
        This function calculates the centroid of the given ref frames and returns the pose of the centroid.
        """
        centroid_pose = Pose()
        for index, frame in enumerate(ref_frames):
            fr: ami_msg.RefFrame = self.get_ref_frame_by_name(frame)
            frame_pose = fr.pose
            if 'x' in dim:
                centroid_pose.position.x += frame_pose.position.x
            if 'y' in dim:
                centroid_pose.position.y += frame_pose.position.y
            if 'z' in dim:
                centroid_pose.position.z += frame_pose.position.z
        centroid_pose.position.x = centroid_pose.position.x/len(ref_frames)
        centroid_pose.position.y = centroid_pose.position.y/len(ref_frames)
        centroid_pose.position.z = centroid_pose.position.z/len(ref_frames)
        if offset_values[0] is not None:
            centroid_pose.position.x += offset_values[0]
        if offset_values[1] is not None:
            centroid_pose.position.y += offset_values[1]
        if offset_values[2] is not None:
            centroid_pose.position.z += offset_values[2]
        # if unit == 'mm':
        #     centroid_pose.position.x = centroid_pose.position.x/1000
        #     centroid_pose.position.y = centroid_pose.position.y/1000
        #     centroid_pose.position.z = centroid_pose.position.z/1000
        # if unit == 'um':
        #     centroid_pose.position.x = centroid_pose.position.x/1000000
        #     centroid_pose.position.y = centroid_pose.position.y/1000000
        #     centroid_pose.position.z = centroid_pose.position.z/1000000
        _pose = Pose()
        if 'x' in dim:
            _pose.position.x = centroid_pose.position.x
        else:
            _pose.position.x = initial_pose.position.x
        if 'y' in dim:
            _pose.position.y = centroid_pose.position.y
        else:
            _pose.position.y = initial_pose.position.y
        if 'z' in dim:
            _pose.position.z = centroid_pose.position.z
        else:
            _pose.position.z = initial_pose.position.z

        _pose.orientation = initial_pose.orientation

        self.logger.debug(f"Centroid pose: {str(_pose)}")
        return _pose

    def get_ref_frame_by_name(self, frame_name:str) -> ami_msg.RefFrame:
        """
        Returns the ref frame from the ref frames list by the given frame name.
        """
        for obj in self.scene.objects_in_scene:
            obj: ami_msg.Object
            for ref_frame in obj.ref_frames:
                ref_frame:ami_msg.RefFrame
                if ref_frame.frame_name == frame_name:
                    return ref_frame
        for ref_frame in self.scene.ref_frames_in_scene:
            ref_frame:ami_msg.RefFrame
            if ref_frame.frame_name == frame_name:
                return ref_frame
            
    def get_parent_frame_for_ref_frame(self,frame_name:str)->str:
        """
        This function returns the parent frame for the given ref frame. 
        If the ref frame does not exist the function returns None.
        """
        for obj in self.scene.objects_in_scene:
            obj: ami_msg.Object
            for ref_frame in obj.ref_frames:
                ref_frame: ami_msg.RefFrame
                if ref_frame.frame_name == frame_name:
                    return ref_frame.parent_frame
                
        for ref_frame in self.scene.ref_frames_in_scene:
            ref_frame: ami_msg.RefFrame
            if ref_frame.frame_name == frame_name:
                return ref_frame.parent_frame    

        return None 

    def check_ref_frames_for_same_parent_frame(self,frame_names:list[str])->bool:
        """
        This function checks if all given ref frames have the same parent frame.
        If this is the case the function returns True. If not the function returns False.
        Parameters:
        - frame_names: list of strings with the names of the ref frames to check
        """
        parent_frame = self.get_parent_frame_for_ref_frame(frame_names[0])
        self.logger.debug(f"Parent frame: {str(parent_frame)}")
        for frame in frame_names:
            if parent_frame != self.get_parent_frame_for_ref_frame(frame) or parent_frame is None:
                return False
        return True

    
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

    def modify_frame_relative(self,frame_name:str, translation: Point, rotation: Quaternion)-> bool:
        self.logger.warn(f"Pose of frame '{frame_name}' will be updated relative to its parent frame!")
        pose_to_modify: Pose = None

        if not self.check_if_frame_exists(frame_name):
            self.logger.error("Frame does not exist")
            return False
        
        # Find the parent Frame
        parent_frame = self.get_parent_frame_for_ref_frame(frame_name)
        pose_to_modify = self.get_ref_frame_by_name(frame_name).pose
        
        if parent_frame is not None:
            pose_to_modify.position.x += translation.x
            pose_to_modify.position.y += translation.y
            pose_to_modify.position.z += translation.z
            pose_to_modify.orientation = quaternion_multiply(pose_to_modify.orientation,rotation)
            self.update_all_ref_frame_constraints()
            self.publish_information()
            self.logger.info(f'Pose for object {frame_name} updated!')
            return True
        else:
            self.logger.warn(f"Pose could not be updated. Frame '{frame_name}' not found!")
            return False   

    def modify_frame_absolut(self,frame_name:str, new_world_pose: Pose)-> bool:
        # Give pose in world coordinates
        pose_to_modify: Pose = Pose()
        if not self.check_if_frame_exists(frame_name):
            self.logger.error("Frame does not exist")
            return False
        
        # Find the parent Frame
        parent_frame = self.get_parent_frame_for_ref_frame(frame_name)
        
        #self.logger(f"{pose_to_modify},{parent_frame}")
        if  parent_frame is not None:
        
            # Transform of point in world       
            transform_global_parent:sp.Matrix = get_transform_matrix_from_tf(get_transform_for_frame_in_world(parent_frame, self.tf_buffer, logger=self.logger))
            
            parent_frame = get_transform_for_frame_in_world(parent_frame, self.tf_buffer, logger=self.logger)
            
            self.logger.warn(f"Parent frame: {parent_frame}")

            self.logger.warn(f"New pose: {new_world_pose}")
            
            new_pose_frame:sp.Matrix = get_transform_matrix_from_tf(new_world_pose)

            new_transform:sp.Matrix = transform_global_parent.inv()*new_pose_frame

            self.logger.warn(f"New transform: {new_transform}")

            pose_to_modify = transform_matrix_to_pose(new_transform)

            self.logger.warn(f"Pose to modify: {pose_to_modify}")

            self.logger.info(f'Frame {frame_name} updated!') 
            
            frame = self.get_ref_frame_by_name(frame_name)
            # only modify the position !!!!
            frame.pose.position = pose_to_modify.position

            # right now there is an error with the orientation. Beside that, the question is, how we would like to handle the orientation (should it actually be modified).
            # frame.pose = pose_to_modify

            self.update_all_ref_frame_constraints()

            self.publish_information()
            return True
        else:
            return False


    def create_assembly_instructions(self,instruction: ami_msg.AssemblyInstruction)->bool:
        # Get plane msgs for object 1
        if instruction.id == "":
                self.logger.error(f"ID of the instruction shoud not be empty. Aboarted!")
                return False
            
        obj1_plane1_msg = self.get_plane_from_scene(instruction.plane_match_1.plane_name_component_1)
        obj1_plane2_msg = self.get_plane_from_scene(instruction.plane_match_2.plane_name_component_1)
        obj1_plane3_msg = self.get_plane_from_scene(instruction.plane_match_3.plane_name_component_1)

        # Get plane msgs for object 2
        obj2_plane1_msg = self.get_plane_from_scene(instruction.plane_match_1.plane_name_component_2)
        obj2_plane2_msg = self.get_plane_from_scene(instruction.plane_match_2.plane_name_component_2)
        obj2_plane3_msg = self.get_plane_from_scene(instruction.plane_match_3.plane_name_component_2) 

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
        
        obj_1_name = self.get_parent_frame_for_ref_frame(self.get_plane_from_scene(instruction.plane_match_1.plane_name_component_1).point_names[0])
        obj_2_name = self.get_parent_frame_for_ref_frame(self.get_plane_from_scene(instruction.plane_match_1.plane_name_component_2).point_names[0])

        # return false if planes are linked to the same component
        if obj_1_name == obj_2_name:
            self.logger.error(f"Invalid plane selection. All given planes are linked to the same component!")
            return False
        
        try:
            transfrom = self.calculate_assembly_transformation(instruction)
        except ValueError as e:
            self.logger.error(str(e))
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
        plane_msg = self.get_plane_from_scene(plane_name)
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
        component_1 = self.get_parent_frame_for_ref_frame(self.get_plane_from_scene(instruction.plane_match_1.plane_name_component_1).point_names[0])
        component_2 = self.get_parent_frame_for_ref_frame(self.get_plane_from_scene(instruction.plane_match_1.plane_name_component_2).point_names[0])

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
        obj_1_plane_1 = self._get_plane_obj_from_scene(instruction.plane_match_1.plane_name_component_1)
        obj_1_plane_2 = self._get_plane_obj_from_scene(instruction.plane_match_2.plane_name_component_1)
        obj_1_plane_3 = self._get_plane_obj_from_scene(instruction.plane_match_3.plane_name_component_1)
        obj_1_name = self.get_parent_frame_for_ref_frame(self.get_plane_from_scene(instruction.plane_match_1.plane_name_component_1).point_names[0])
        obj_2_name = self.get_parent_frame_for_ref_frame(self.get_plane_from_scene(instruction.plane_match_1.plane_name_component_2).point_names[0])
        
        #obj_1_mate_plane_intersection: Vector3 = point3D_to_vector3(get_point_of_plane_intersection(obj_1_plane_1, obj_1_plane_2, obj_1_plane_3))

        obj_2_plane_1 = self._get_plane_obj_from_scene(instruction.plane_match_1.plane_name_component_2)
        obj_2_plane_2 = self._get_plane_obj_from_scene(instruction.plane_match_2.plane_name_component_2)
        obj_2_plane_3 = self._get_plane_obj_from_scene(instruction.plane_match_3.plane_name_component_2)

        self.logger.debug(f"Glas - Plane 1: {obj_1_plane_1}")
        self.logger.debug(f"Glas - Plane 2: {obj_1_plane_2}")
        self.logger.debug(f"Glas - Plane 3: {obj_1_plane_3}")

        self.logger.debug(f"UFC - Plane 1: {obj_2_plane_1}")
        self.logger.debug(f"UFC - Plane 2: {obj_2_plane_2}")
        self.logger.debug(f"UFC - Plane 3: {obj_2_plane_3}")

        #obj_2_mate_plane_intersection: Vector3 = point3D_to_vector3(get_point_of_plane_intersection(obj_2_plane_1, obj_2_plane_2, obj_2_plane_3))

        obj_1_mate_plane_intersection, obj_2_mate_plane_intersection = self.calculate_plane_intersections(instruction)

        assembly_transform = Pose()

        if instruction.component_1_is_moving_part:
            moving_component = obj_1_name
            static_component = obj_2_name

        else:
            moving_component = obj_2_name
            static_component = obj_1_name

        self.logger.warn(f"\nMoving component: '{moving_component}'\nStatic component: '{static_component}'")
        
        # Get the ideal norm vectors from the plane messages
        comp_1_plane_1_ideal_norm_vector = self.get_plane_from_scene(instruction.plane_match_1.plane_name_component_1).ideal_norm_vector
        comp_1_plane_2_ideal_norm_vector = self.get_plane_from_scene(instruction.plane_match_2.plane_name_component_1).ideal_norm_vector
        comp_1_plane_3_ideal_norm_vector = self.get_plane_from_scene(instruction.plane_match_3.plane_name_component_1).ideal_norm_vector

        comp_2_plane_1_ideal_norm_vector = self.get_plane_from_scene(instruction.plane_match_1.plane_name_component_2).ideal_norm_vector
        comp_2_plane_2_ideal_norm_vector = self.get_plane_from_scene(instruction.plane_match_2.plane_name_component_2).ideal_norm_vector
        comp_2_plane_3_ideal_norm_vector = self.get_plane_from_scene(instruction.plane_match_3.plane_name_component_2).ideal_norm_vector

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
        self.logger.warn(f"Norm vec obj 1_1: {instruction.plane_match_1.plane_name_component_1}")
        self.logger.warn(f"Norm vec obj 1_2: {instruction.plane_match_2.plane_name_component_1}")
        self.logger.warn(f"Norm vec obj 1_3: {instruction.plane_match_3.plane_name_component_1}")
        self.logger.warn(f"Norm vec obj 2_1: {instruction.plane_match_1.plane_name_component_2}")
        self.logger.warn(f"Norm vec obj 2_2: {instruction.plane_match_2.plane_name_component_2}")
        self.logger.warn(f"Norm vec obj 2_3: {instruction.plane_match_3.plane_name_component_2}")
        mult_1 = norm_vec_direction(bvec_obj_1_1,comp_1_plane_1_ideal_norm_vector, logger=self.logger)
        mult_2 = norm_vec_direction(bvec_obj_1_2,comp_1_plane_2_ideal_norm_vector, logger=self.logger)
        mult_3 = norm_vec_direction(bvec_obj_1_3,comp_1_plane_3_ideal_norm_vector, logger=self.logger)
        mult_4 = norm_vec_direction(bvec_obj_2_1,comp_2_plane_1_ideal_norm_vector, logger=self.logger)
        mult_5 = norm_vec_direction(bvec_obj_2_2,comp_2_plane_2_ideal_norm_vector, logger=self.logger)
        mult_6 = norm_vec_direction(bvec_obj_2_3,comp_2_plane_3_ideal_norm_vector, logger=self.logger)

        self.logger.debug(f"Mults: {mult_1}, {mult_2}, {mult_3}, {mult_4}, {mult_5}, {mult_6}")
        
        # Check if the normal vectors should be inverted
        if instruction.plane_match_1.inv_normal_vector:
            mult_4 = -mult_4
            self.logger.debug("Inverted normal vector for plane 1")
        if instruction.plane_match_2.inv_normal_vector:
            mult_5 = -mult_5
            self.logger.debug("Inverted normal vector for plane 2")
        if instruction.plane_match_3.inv_normal_vector:
            mult_6 = -mult_6
            self.logger.debug("Inverted normal vector for plane 3")

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
        assembly_transform.orientation = self.calc_approx_quat_from_matrix(rot_matrix)

        #self.logger.info(f"Assembly transformation is: {assembly_transform.__str__()}")
        self.logger.info(f"""Assembly transformation is: \n
                         x: {assembly_transform.position.x},\n
                         y: {assembly_transform.position.y},\n
                         z: {assembly_transform.position.z},\n
                         w: {assembly_transform.orientation.w},\n
                         x: {assembly_transform.orientation.x},\n
                         y: {assembly_transform.orientation.y},\n
                         z: {assembly_transform.orientation.z}""")

        add_success = self.add_assembly_frames_to_scene(instruction.id,
                                                        moving_component,
                                                        static_component,
                                                        moving_component_plane_intersection,
                                                        static_component_plane_intersection,
                                                        assembly_transform)
        if not add_success:
            raise Exception
        
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
        
        def cost_function(params, target_matrix):
            alpha, beta, gamma = params
            rotation_matrix = get_euler_rotation_matrix(alpha,beta,gamma)
            difference = (rotation_matrix - target_matrix).norm()
            return difference
        
        self.logger.warn(f"Starting calculating the transformation...")
        max_iter = 1000

        #result = minimize(cost_function, initial_guess, args=(rot_obj2_to_obj1),  method='L-BFGS-B', tol = 1e-10, options={'maxiter': 1000})
        result = minimize(cost_function, initial_guess, args=(rot_mat),  method='Nelder-Mead', tol = 1e-20, options={'maxiter': max_iter})

        if max_iter == result.nit:
            self.logger.info(f"Max iterations reached ({max_iter}). Measurement inacurate.")
        else:
            self.logger.info(f"Iterations ran: {result.nit}")
        if result.fun >1.0:
            self.logger.warn(f"Residual error of rotation calculation is very high: {result.fun}. You should not assemble the components.")
        elif result.fun >0.5:
            self.logger.warn(f"Residual error of rotation calculation is high: {result.fun}. Make sure you identified all necessary assembly frames.")
        else:
            self.logger.info(f"Calculation has low residual error: {result.fun}! This is a good sign. You can assemble the components.")
        
        self.logger.info(f"Result (deg) is: \nalpha: {result.x[0]*(180/np.pi)}, \nbeta: {result.x[1]*(180/np.pi)}, \ngamma:{result.x[2]*(180/np.pi)}")

        quaternion = euler_to_quaternion(   roll   = result.x[2],
                                            pitch  = result.x[1],
                                            yaw    = result.x[0])
        return quaternion
    
    def get_plane_from_scene(self, plane_name:str)-> ami_msg.Plane:
        plane_msg = None
        for plane in self.scene.planes_in_scene:
            plane: ami_msg.Plane
            if plane_name == plane.ref_plane_name:
                plane_msg = plane
                break

        for obj in self.scene.objects_in_scene:
            obj:ami_msg.Object
            for plane in obj.ref_planes:
                plane: ami_msg.Plane
                if plane_name == plane.ref_plane_name:
                    plane_msg = plane
                    break

        return plane_msg
    
    def get_axis_from_scene(self, axis_name:str)-> ami_msg.Axis:
        axis_msg = None
        for axis in self.scene.axis_in_scene:
            axis: ami_msg.Axis
            if axis_name == axis.axis_name:
                axis_msg = axis
                break

        for obj in self.scene.objects_in_scene:
            obj:ami_msg.Object
            for axis in obj.ref_axis:
                axis: ami_msg.Axis
                if axis_name == axis.axis_name:
                    axis_msg = axis
                    break
        return axis_msg
    
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
        axis_msg = self.get_axis_from_scene(axis_name)

        line3d:sp.Line3D = get_line3d_from_frame_names(axis_msg.point_names, tf_buffer=self.tf_buffer, parent_frame=parent_frame, logger=self.logger)

        if parent_frame is None:
            t_point:TransformStamped = get_transform_for_frame_in_world(frame_name, self.tf_buffer, logger=self.logger)
        else:
            t_point:TransformStamped = get_transform_for_frame(frame_name, parent_frame, self.tf_buffer, logger=self.logger)

        point = get_point_from_ros_obj(t_point.transform.translation)

        plane = sp.Plane(point, normal_vector = line3d.direction)

        return plane
    
    @staticmethod
    def is_frame_from_scene( scene:ami_msg.ObjectScene, frame_name:str)->tuple[Union[str,None], Union[str,None]]:
        """
        If associated with an object returns
        (object_name, frame_name)
        If associated with a ref_frame returns
        (None, frame_name)
        """

        for obj in scene.objects_in_scene:
            obj:ami_msg.Object
            
            for ref_frame in obj.ref_frames:
                ref_frame:ami_msg.RefFrame
                if f"{obj.obj_name}_{ref_frame.frame_name}" == frame_name:
                    return (obj.obj_name, ref_frame.frame_name)

        for ref_frame in scene.ref_frames_in_scene:
            ref_frame:ami_msg.RefFrame
            if ref_frame.frame_name == frame_name:
                return None, ref_frame.frame_name
            
        return None, None
    

