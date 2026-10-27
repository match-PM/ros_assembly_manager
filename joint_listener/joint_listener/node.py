"""Listen for robot motion and invalidate downstream component measurements."""

import math
from xml.etree.ElementTree import ParseError

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
import yaml

from assembly_manager_interfaces.msg import ObjectScene
from assembly_manager_interfaces.srv import InvalidateFrameMeasurements
from joint_listener.tracking import (
    IncompleteHierarchyError, JointTracker, downstream_component_frames, robot_links,
)


class JointListener(Node):
    def __init__(self, **kwargs):
        super().__init__('joint_listener', **kwargs)
        joint_names = self.declare_parameter('joints', ['']).value
        self.trackers = {}
        self.child_overrides = {}
        threshold_labels = {}
        for name in joint_names:
            if not name:
                continue
            threshold = self.declare_parameter(f'joints.{name}.threshold', 0.0).value
            unit = self.declare_parameter(f'joints.{name}.threshold_unit', 'rad').value
            if unit not in ('deg', 'rad', 'm'):
                raise ValueError(f'Invalid threshold unit for {name}: {unit!r}; use deg, rad, or m.')
            threshold = float(threshold)
            self.trackers[name] = JointTracker(
                math.radians(threshold) if unit == 'deg' else threshold)
            threshold_labels[name] = f'{threshold} {unit}'
            self.child_overrides[name] = self.declare_parameter(
                f'joints.{name}.child_frame', '').value
        joint_topic = self.declare_parameter('joint_states_topic', '/joint_states').value
        scene_topic = self.declare_parameter('scene_topic', '/assembly_manager/scene').value
        description_topic = self.declare_parameter(
            'robot_description_topic', '/robot_description').value
        service = self.declare_parameter(
            'invalidate_service', '/assembly_manager/invalidate_frame_measurements').value
        self.scene = None
        self.joint_children = {}
        self.robot_parents = {}
        self.pending = {}
        self.generation = 0
        self.in_flight = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.client = self.create_client(InvalidateFrameMeasurements, service)
        self.joint_subscription = self.create_subscription(
            JointState, joint_topic, self.on_joints, qos_profile_sensor_data)
        self.scene_subscription = self.create_subscription(
            ObjectScene, scene_topic, self.on_scene, 10)
        self.description_subscription = self.create_subscription(
            String, description_topic, self.on_description,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.retry_timer = self.create_timer(0.5, self.flush_pending)
        if not self.trackers:
            self.get_logger().warn('No joints configured; set the joints parameter in YAML.')
        for name in self.trackers:
            self.get_logger().info(f'Monitoring {name}, threshold {threshold_labels[name]}.')

    def on_description(self, message):
        try:
            self.joint_children, self.robot_parents = robot_links(message.data)
        except (ParseError, KeyError) as error:
            self.get_logger().error(f'Invalid robot description: {error}')
            return
        self.flush_pending()

    def on_scene(self, scene):
        self.scene = scene
        self.flush_pending()

    def on_joints(self, message):
        for name, position in zip(message.name, message.position):
            tracker = self.trackers.get(name)
            if tracker is not None and tracker.update(position):
                self.generation += 1
                self.pending[name] = self.generation
                self.get_logger().info(f'{name} exceeded its movement threshold.')
        self.flush_pending()

    def flush_pending(self):
        if self.in_flight or not self.pending or self.scene is None:
            return
        parents = dict(self.robot_parents)
        tf_frames = yaml.safe_load(self.tf_buffer.all_frames_as_yaml()) or {}
        parents.update({child: info['parent'] for child, info in tf_frames.items()})
        resolved, frames = {}, set()
        for name, generation in self.pending.items():
            root = self.child_overrides[name] or self.joint_children.get(name)
            if not root:
                self.get_logger().warn(
                    f'Cannot resolve child link for {name}; waiting for robot_description '
                    'or configure its child_frame.', throttle_duration_sec=10.0)
                continue
            try:
                downstream = downstream_component_frames(self.scene, root, parents)
            except IncompleteHierarchyError as error:
                self.get_logger().warn(str(error), throttle_duration_sec=10.0)
                continue
            resolved[name] = generation
            frames.update(downstream)
        if not resolved:
            return
        measured_frames = {
            frame.frame_name
            for component in self.scene.objects_in_scene
            for frame in component.ref_frames
            if (frame.properties.vision_frame_properties.has_been_measured
                or frame.properties.laser_frame_properties.has_been_measured)
        }
        frames.intersection_update(measured_frames)
        if not frames:
            self._acknowledge(resolved)
            return
        if not self.client.service_is_ready():
            return
        request = InvalidateFrameMeasurements.Request(frame_names=sorted(frames))
        self.in_flight = True
        future = self.client.call_async(request)
        future.add_done_callback(lambda result: self._completed(result, resolved))

    def _acknowledge(self, resolved):
        # Keep motion that arrived while a previous request was in flight.
        for name, generation in resolved.items():
            if self.pending.get(name) == generation:
                del self.pending[name]

    def _completed(self, future, resolved):
        self.in_flight = False
        try:
            response = future.result()
            if response.success:
                self._acknowledge(resolved)
            else:
                self.get_logger().warn(f'Measurement invalidation failed: {response.message}')
        except Exception as error:
            self.get_logger().warn(f'Measurement invalidation request failed: {error}')
        # Failures remain pending for the timer to retry with the latest scene.


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = JointListener()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
