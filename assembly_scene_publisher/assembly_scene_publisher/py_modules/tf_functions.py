
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, StaticTransformBroadcaster
import rclpy
from rclpy.node import Node

import sympy as sp
from geometry_msgs.msg import Vector3, Quaternion
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
import numpy as np
from typing import Union

from assembly_scene_publisher.py_modules.geometry_type_functions import get_point_from_ros_obj

ROUND_FACTOR = 9


def adapt_transform_for_new_parent_frame(child_frame, new_parent_frame, tf_buffer: Buffer):
    # this function adapts the tf for parent_frame changes
    t:TransformStamped = tf_buffer.lookup_transform(child_frame, new_parent_frame,rclpy.time.Time())
    trans = Vector3()
    rot = Quaternion()
    trans.x = -t.transform.translation.x
    trans.y = -t.transform.translation.y
    trans.z = -t.transform.translation.z

    rot.x = t.transform.rotation.x
    rot.y = t.transform.rotation.y
    rot.z = t.transform.rotation.z
    rot.w = t.transform.rotation.w

    return trans, rot

def get_transform_for_frame_in_world(frame_name: str, tf_buffer: Buffer, logger = None) -> TransformStamped:
    # this function adapts the tf for parent_frame changes
    #transform:TransformStamped = tf_buffer.lookup_transform(frame_name, 'world',rclpy.time.Time())
    try:
        
        transform:TransformStamped = tf_buffer.lookup_transform('world', frame_name, rclpy.time.Time(),rclpy.duration.Duration(seconds=1.0))
        if logger is not None:
            logger.debug(f"Frame '{frame_name}' found in TF!")
    except Exception as e:
        transform = None
        raise ValueError(f"Frame '{frame_name}' does not exist in TF! {str(e)}")
    return transform

def get_transform_for_frame(frame_name: str, parent_frame:str, tf_buffer: Buffer, logger = None) -> TransformStamped:
    # this function adapts the tf for parent_frame changes
    #transform:TransformStamped = tf_buffer.lookup_transform(frame_name, 'world',rclpy.time.Time())
    try:
        
        transform:TransformStamped = tf_buffer.lookup_transform(parent_frame, frame_name, rclpy.time.Time(),rclpy.duration.Duration(seconds=1.0))
        if logger is not None:
            logger.debug(f"Frame '{frame_name}' found in TF!")
    except Exception as e:
        transform = None
        raise ValueError(f"Frame '{frame_name}' does not exist in TF! {str(e)}")
    return transform

def get_plane_from_frame_names(frames: list[str], tf_buffer: Buffer, parent_frame:str = None, logger = None)-> sp.Plane:

    if len(frames)!=3:
        raise ValueError
    
    if parent_frame is not None:
        t1:TransformStamped = get_transform_for_frame(frames[0], parent_frame, tf_buffer)
        t2:TransformStamped = get_transform_for_frame(frames[1], parent_frame, tf_buffer)
        t3:TransformStamped = get_transform_for_frame(frames[2], parent_frame, tf_buffer)
    else:
        t1:TransformStamped = get_transform_for_frame_in_world(frames[0], tf_buffer)
        t2:TransformStamped = get_transform_for_frame_in_world(frames[1], tf_buffer)
        t3:TransformStamped = get_transform_for_frame_in_world(frames[2], tf_buffer)
    
    if logger is not None:  
        logger.debug(f"t1: {t1}")
        logger.debug(f"t2: {t2}")
        logger.debug(f"t3: {t3}")

    if t1 is None or t2 is None or t3 is None:
        raise ValueError
    t_rounded_1 = TransformStamped()
    t_rounded_1.header = t1.header
    t_rounded_1.child_frame_id = t1.child_frame_id
    t_rounded_1.transform.translation.x = round(t1.transform.translation.x, ROUND_FACTOR)
    t_rounded_1.transform.translation.y = round(t1.transform.translation.y, ROUND_FACTOR)
    t_rounded_1.transform.translation.z = round(t1.transform.translation.z, ROUND_FACTOR)
    
    t_rounded_2 = TransformStamped()
    t_rounded_2.header = t2.header
    t_rounded_2.child_frame_id = t2.child_frame_id
    t_rounded_2.transform.translation.x = round(t2.transform.translation.x, ROUND_FACTOR)
    t_rounded_2.transform.translation.y = round(t2.transform.translation.y, ROUND_FACTOR)
    t_rounded_2.transform.translation.z = round(t2.transform.translation.z, ROUND_FACTOR)


    t_rounded_3 = TransformStamped()
    t_rounded_3.header = t3.header
    t_rounded_3.child_frame_id = t3.child_frame_id
    t_rounded_3.transform.translation.x = round(t3.transform.translation.x, ROUND_FACTOR)
    t_rounded_3.transform.translation.y = round(t3.transform.translation.y, ROUND_FACTOR)
    t_rounded_3.transform.translation.z = round(t3.transform.translation.z, ROUND_FACTOR)

    p1 = get_point_from_ros_obj(t_rounded_1.transform.translation)
    p2 = get_point_from_ros_obj(t_rounded_2.transform.translation)
    p3 = get_point_from_ros_obj(t_rounded_3.transform.translation)

    plane = sp.Plane(p1,p2,p3)

    return plane

def get_line3d_from_frame_names(frames: list[str],  tf_buffer: Buffer, parent_frame:str = None, logger=None)-> sp.Line3D:

    if len(frames)!=2:
        raise ValueError
    if parent_frame is not None:
        t1:TransformStamped = get_transform_for_frame(frames[0], parent_frame, tf_buffer, logger)
        t2:TransformStamped = get_transform_for_frame(frames[1], parent_frame, tf_buffer, logger)
    else:
        t1:TransformStamped = get_transform_for_frame_in_world(frames[0], tf_buffer, logger)
        t2:TransformStamped = get_transform_for_frame_in_world(frames[1], tf_buffer, logger)
    if t1 is None or t2 is None:
        raise ValueError
    
    t_rounded_1 = TransformStamped()
    t_rounded_1.header = t1.header
    t_rounded_1.child_frame_id = t1.child_frame_id
    t_rounded_1.transform.translation.x = round(t1.transform.translation.x, ROUND_FACTOR)
    t_rounded_1.transform.translation.y = round(t1.transform.translation.y, ROUND_FACTOR)
    t_rounded_1.transform.translation.z = round(t1.transform.translation.z, ROUND_FACTOR)
    
    t_rounded_2 = TransformStamped()
    t_rounded_2.header = t2.header
    t_rounded_2.child_frame_id = t2.child_frame_id
    t_rounded_2.transform.translation.x = round(t2.transform.translation.x, ROUND_FACTOR)
    t_rounded_2.transform.translation.y = round(t2.transform.translation.y, ROUND_FACTOR)
    t_rounded_2.transform.translation.z = round(t2.transform.translation.z, ROUND_FACTOR)

    p1 = get_point_from_ros_obj(t_rounded_1.transform.translation)
    p2 = get_point_from_ros_obj(t_rounded_2.transform.translation)

    line = sp.Line3D(p1,p2)

    return line

def publish_transform_tf_static(node: Node, 
                                tf_broadcaster:StaticTransformBroadcaster, 
                                child_frame:str, 
                                parent_frame:str, 
                                translation:Vector3, 
                                rotations:Quaternion):
    # Create a static transform
    transform_stamped = TransformStamped()
    transform_stamped.header.stamp = node.get_clock().now().to_msg()  # Use current timestamp
    transform_stamped.header.frame_id = parent_frame
    transform_stamped.child_frame_id = child_frame

    # # Set the translation
    transform_stamped.transform.translation.x = float(translation.x)
    transform_stamped.transform.translation.y = float(translation.y)
    transform_stamped.transform.translation.z = float(translation.z)

    # Set the rotation (quaternion)
    transform_stamped.transform.rotation.x = float(rotations.x)
    transform_stamped.transform.rotation.y = float(rotations.y)
    transform_stamped.transform.rotation.z = float(rotations.z)
    transform_stamped.transform.rotation.w = float(rotations.w)

    # Publish the static transform
    tf_broadcaster.sendTransform(transform_stamped)
