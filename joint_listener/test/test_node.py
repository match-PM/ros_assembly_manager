"""Exercise ROS subscription configuration and asynchronous invalidation requests."""

import math
from time import monotonic
from uuid import uuid4

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from assembly_manager_interfaces.msg import Object, ObjectScene, RefFrame
from assembly_manager_interfaces.srv import InvalidateFrameMeasurements
from joint_listener.node import JointListener


def spin_until(executor, predicate, timeout=5.0):
    deadline = monotonic() + timeout
    while not predicate() and monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)
    assert predicate(), 'Timed out waiting for ROS callbacks'


def test_motion_queues_until_description_arrives_and_failed_service_is_retried():
    rclpy.init()
    executor = SingleThreadedExecutor()
    server = Node('joint_listener_test_server')
    listener = None
    prefix = '/joint_listener_test_' + uuid4().hex
    calls = []

    def invalidate(request, response):
        calls.append(list(request.frame_names))
        # Exercise retry on failure, then let the queued motion complete.
        response.success = len(calls) > 1
        return response

    try:
        server.create_service(
            InvalidateFrameMeasurements, prefix + '/invalidate', invalidate)
        listener = JointListener(parameter_overrides=[
            Parameter('joints', value=['axis']),
            Parameter('joints.axis.threshold', value=math.degrees(0.125)),
            Parameter('joints.axis.threshold_unit', value='deg'),
            Parameter('joint_states_topic', value=prefix + '/joints'),
            Parameter('scene_topic', value=prefix + '/scene'),
            Parameter('robot_description_topic', value=prefix + '/description'),
            Parameter('invalidate_service', value=prefix + '/invalidate'),
        ])
        executor.add_node(server)
        executor.add_node(listener)
        publisher = server.create_publisher(JointState, prefix + '/joints', 10)
        spin_until(executor, lambda: publisher.get_subscription_count() > 0)
        publisher.publish(JointState(name=['axis'], position=[1.0]))
        spin_until(executor, lambda: listener.trackers['axis'].reference is not None)
        publisher.publish(JointState(name=['axis'], position=[1.125]))
        publisher.publish(JointState(name=['axis'], position=[1.25]))
        spin_until(executor, lambda: bool(listener.pending))
        assert calls == []
        scene = ObjectScene(objects_in_scene=[
            Object(obj_name='part', parent_frame='tool', ref_frames=[
                RefFrame(frame_name='part_measure', parent_frame='part'),
                RefFrame(frame_name='part_laser', parent_frame='part'),
                RefFrame(frame_name='part_unmeasured', parent_frame='part')]),
            Object(obj_name='fixed', parent_frame='world', ref_frames=[
                RefFrame(frame_name='fixed_measure', parent_frame='fixed')]),
        ])
        measured = scene.objects_in_scene[0].ref_frames
        measured[0].properties.vision_frame_properties.has_been_measured = True
        measured[1].properties.laser_frame_properties.has_been_measured = True
        scene.objects_in_scene[1].ref_frames[0].properties.vision_frame_properties.has_been_measured = True
        listener.on_scene(scene)
        assert listener.pending
        listener.on_description(String(data='''<robot name="test">
          <joint name="axis" type="prismatic">
            <parent link="world"/><child link="tool"/>
          </joint>
        </robot>'''))
        spin_until(executor, lambda: len(calls) >= 2 and not listener.pending)
        assert calls == [['part_laser', 'part_measure'], ['part_laser', 'part_measure']]
        assert listener.trackers['axis'].reference == 1.25

        # The next scene reports the cleared flags. Another threshold crossing
        # needs no request, even though the unrelated branch is still measured.
        measured[0].properties.vision_frame_properties.has_been_measured = False
        measured[1].properties.laser_frame_properties.has_been_measured = False
        listener.on_scene(scene)
        listener.on_joints(JointState(name=['axis'], position=[1.5]))
        assert not listener.pending
        assert not listener.in_flight
        assert len(calls) == 2

        # A new measurement becomes invalid again on the next movement.
        measured[1].properties.laser_frame_properties.has_been_measured = True
        listener.on_scene(scene)
        listener.on_joints(JointState(name=['axis'], position=[1.75]))
        spin_until(executor, lambda: len(calls) == 3 and not listener.pending)
        assert calls[-1] == ['part_laser']
    finally:
        executor.shutdown()
        if listener is not None:
            listener.destroy_node()
        server.destroy_node()
        rclpy.shutdown()
