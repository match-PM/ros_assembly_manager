"""Pose modifications support component-owned and standalone scene frames."""

import importlib
from unittest.mock import Mock

import assembly_manager_interfaces.msg as msg
import assembly_manager_interfaces.srv as srv
from geometry_msgs.msg import TransformStamped
import pytest

from assembly_scene_publisher.py_modules.AssemblyScene import AssemblyManagerScene
from assembly_scene_publisher.py_modules.AssemblySceneAnalyzer import AssemblySceneAnalyzer


@pytest.mark.parametrize('component_owned', [False, True])
@pytest.mark.parametrize('method', [
    'modify_frame_absolut', 'modify_frame_relative', 'modify_frame_set_to_frame',
])
def test_modify_frame_updates_pose_measurements_and_constraints(
        monkeypatch, component_owned, method):
    scene = msg.ObjectScene()
    frame = msg.RefFrame(frame_name='laser', parent_frame='world')
    frame.pose.orientation.w = 1.0
    if component_owned:
        component = msg.Object(obj_name='part')
        component.ref_frames.append(frame)
        scene.objects_in_scene.append(component)
    else:
        scene.ref_frames_in_scene.append(frame)

    manager = AssemblyManagerScene.__new__(AssemblyManagerScene)
    manager.scene = scene
    manager.assembly_scene_analyzer = AssemblySceneAnalyzer(scene)
    manager.logger = Mock()
    manager.tf_buffer = Mock()
    manager.check_if_frame_exists = manager.assembly_scene_analyzer.is_frame_from_scene
    manager.publish_information = Mock()
    manager.update_scene_with_constraints = Mock()

    module = importlib.import_module(AssemblyManagerScene.__module__)
    constraints = Mock()
    monkeypatch.setattr(module, 'calculate_constraints_for_component', constraints)
    parent_transform = TransformStamped()
    parent_transform.transform.rotation.w = 1.0
    monkeypatch.setattr(module, 'get_transform_for_frame_in_world',
                        Mock(return_value=parent_transform))
    source_transform = TransformStamped()
    source_transform.transform.rotation.w = 1.0
    source_transform.transform.translation.x = 2.0
    monkeypatch.setattr(module, 'get_transform_for_frame',
                        Mock(return_value=source_transform))

    if method == 'modify_frame_absolut':
        request = srv.ModifyPoseAbsolut.Request(frame_name='laser')
        request.pose.position.x = 2.0
        request.pose.orientation.w = 1.0
    elif method == 'modify_frame_relative':
        request = srv.ModifyPoseRelative.Request(frame_name='laser')
        request.rel_position.x = 2.0
        request.rel_rotation.w = 1.0
    else:
        request = srv.ModifyPoseFromFrame.Request(
            frame_to_set='laser', from_frame='source')
    request.set_laser_measured = True
    request.set_vision_measured = True

    assert getattr(manager, method)(request)
    assert frame.pose.position.x == pytest.approx(2.0)
    assert frame.properties.laser_frame_properties.is_laser_frame
    assert frame.properties.laser_frame_properties.has_been_measured
    assert frame.properties.vision_frame_properties.is_vision_frame
    assert frame.properties.vision_frame_properties.has_been_measured
    if component_owned:
        constraints.assert_called_once_with(
            scene=scene, component_name='part', logger=manager.logger)
        manager.publish_information.assert_called_once_with()
        manager.update_scene_with_constraints.assert_not_called()
    else:
        constraints.assert_not_called()
        manager.publish_information.assert_not_called()
        manager.update_scene_with_constraints.assert_called_once_with()
