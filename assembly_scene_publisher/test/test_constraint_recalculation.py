"""Fast dependency-based frame constraint recalculation."""

from copy import deepcopy as real_deepcopy
from unittest.mock import Mock

import assembly_manager_interfaces.msg as msg
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose
import pytest

from assembly_scene_publisher.py_modules.AssemblyScene import AssemblyManagerScene
import assembly_scene_publisher.py_modules.frame_constraints as constraints


def make_frame(name, reference=None, offset=0.0):
    frame = msg.RefFrame(frame_name=name, parent_frame='world')
    frame.pose.orientation.w = 1.0
    if reference is not None:
        transform = frame.constraints.transform
        transform.is_active = True
        transform.ref_frame = reference
        transform.transform.translation.x = offset
        transform.transform.rotation.w = 1.0
    return frame


def test_changed_frame_recalculates_only_transitive_dependents():
    scene = msg.ObjectScene()
    first = msg.Object(obj_name='first')
    source = make_frame('source')
    local_dependent = make_frame('local_dependent', 'source', 1000.0)
    unrelated = make_frame('unrelated')
    unrelated.pose.position.x = 9.0
    first.ref_frames.extend((source, local_dependent, unrelated))
    scene.objects_in_scene.append(first)

    second = msg.Object(obj_name='second')
    cross_component = make_frame(
        'cross_component', 'local_dependent', 2000.0)
    second.ref_frames.append(cross_component)
    scene.objects_in_scene.append(second)

    source.pose.position.x = 4.0
    constraints.calculate_constraints_for_frame_change(scene, 'source')

    assert source.pose.position.x == pytest.approx(4.0)
    assert local_dependent.pose.position.x == pytest.approx(5.0)
    assert cross_component.pose.position.x == pytest.approx(7.0)
    assert unrelated.pose.position.x == pytest.approx(9.0)


def test_dependency_dag_calculates_each_affected_frame_once(monkeypatch):
    scene = msg.ObjectScene()
    component = msg.Object(obj_name='part')
    source = make_frame('source')
    left = make_frame('left', 'source')
    right = make_frame('right', 'source')
    result = make_frame('result')
    result.constraints.centroid.is_active = True
    result.constraints.centroid.dim = 'xyz'
    result.constraints.centroid.ref_frame_names = ['left', 'right']
    component.ref_frames.extend((source, left, right, result))
    scene.objects_in_scene.append(component)

    calls = []
    original = constraints.update_ref_frame_by_constraint

    def record(*args, **kwargs):
        calls.append(args[1].frame_name)
        return original(*args, **kwargs)

    monkeypatch.setattr(constraints, 'update_ref_frame_by_constraint', record)
    constraints.calculate_constraints_for_frame_change(scene, 'source')

    assert calls[0] == 'source'
    assert calls[-1] == 'result'
    assert set(calls[1:3]) == {'left', 'right'}
    assert len(calls) == len(set(calls)) == 4


def test_multiple_changed_frames_recalculate_shared_dependents_once(monkeypatch):
    scene = msg.ObjectScene()
    component = msg.Object(obj_name='part')
    left = make_frame('left')
    right = make_frame('right')
    result = make_frame('result')
    result.constraints.centroid.is_active = True
    result.constraints.centroid.dim = 'xyz'
    result.constraints.centroid.ref_frame_names = ['left', 'right']
    component.ref_frames.extend((left, right, result))
    scene.objects_in_scene.append(component)

    calls = []
    original = constraints.update_ref_frame_by_constraint

    def record(*args, **kwargs):
        calls.append(args[1].frame_name)
        return original(*args, **kwargs)

    monkeypatch.setattr(constraints, 'update_ref_frame_by_constraint', record)
    constraints.calculate_constraints_for_frame_changes(
        scene, ['left', 'right'])

    assert calls[-1] == 'result'
    assert set(calls[:2]) == {'left', 'right'}
    assert len(calls) == len(set(calls)) == 3


def test_affected_frames_include_inactive_constraints():
    scene = msg.ObjectScene()
    component = msg.Object(obj_name='part')
    source = make_frame('source')
    dependent = make_frame('dependent', 'source')
    dependent.constraints.transform.is_active = False
    component.ref_frames.extend((source, dependent))
    scene.objects_in_scene.append(component)

    assert constraints.get_frames_affected_by_changes(
        scene, ['source']) == ['source', 'dependent']


def test_active_dependency_cycle_is_rejected():
    scene = msg.ObjectScene()
    scene.ref_frames_in_scene.extend((
        make_frame('first', 'second'),
        make_frame('second', 'first'),
    ))

    with pytest.raises(ValueError, match='Cycle detected'):
        constraints.calculate_constraints_for_frame_change(scene, 'first')


def test_constraint_update_copies_only_the_pose(monkeypatch):
    scene = msg.ObjectScene()
    frame = make_frame('frame')
    scene.ref_frames_in_scene.append(frame)
    copied_types = []

    def recording_deepcopy(value):
        copied_types.append(type(value))
        return real_deepcopy(value)

    monkeypatch.setattr(constraints, 'deepcopy', recording_deepcopy)
    assert constraints.update_ref_frame_by_constraint(scene, frame)

    assert copied_types == [Pose]


def test_publish_to_tf_batches_all_static_transforms():
    scene = msg.ObjectScene()
    scene.ref_frames_in_scene.append(make_frame('standalone'))
    component = msg.Object(obj_name='part', parent_frame='world')
    component.ref_frames.append(make_frame('owned'))
    scene.objects_in_scene.append(component)

    manager = AssemblyManagerScene.__new__(AssemblyManagerScene)
    manager.scene = scene
    manager.node = Mock()
    manager.node.get_clock.return_value.now.return_value.to_msg.return_value = Time()
    manager.tf_broadcaster = Mock()

    manager.publish_to_tf()

    manager.tf_broadcaster.sendTransform.assert_called_once()
    transforms = manager.tf_broadcaster.sendTransform.call_args.args[0]
    assert [transform.child_frame_id for transform in transforms] == [
        'standalone', 'part', 'owned']
