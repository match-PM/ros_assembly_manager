"""Regression tests for threshold crossings and component ancestry."""

from types import SimpleNamespace as NS

import pytest

from joint_listener.tracking import (
    JointTracker, downstream_component_frames, is_downstream, robot_links,
)


def test_noise_is_ignored_but_slow_accumulated_motion_crosses_threshold():
    tracker = JointTracker(0.125)
    assert not tracker.update(1.0)
    for position in (1.03125, 0.96875, 1.0625, 1.125):
        assert not tracker.update(position)
    assert tracker.update(1.15625)
    assert not tracker.update(1.1875)
    assert tracker.update(0.875)


@pytest.mark.parametrize('threshold', [-1.0, float('nan'), float('inf')])
def test_invalid_thresholds_rejected(threshold):
    with pytest.raises(ValueError):
        JointTracker(threshold)


def test_invalid_positions_do_not_change_reference():
    tracker = JointTracker(0.01)
    assert not tracker.update(float('nan'))
    assert tracker.reference is None
    assert not tracker.update(1.0)
    assert not tracker.update(float('inf'))
    assert tracker.reference == 1.0
    assert tracker.update(1.1)


def test_urdf_resolves_joint_child_links_and_fixed_descendants():
    children, parents = robot_links('''<robot name="test">
      <joint name="axis" type="prismatic">
        <parent link="world"/><child link="slide"/>
      </joint>
      <joint name="mount" type="fixed">
        <parent link="slide"/><child link="tool"/>
      </joint>
    </robot>''')
    assert children['axis'] == 'slide'
    assert is_downstream('tool', 'slide', parents)
    assert not is_downstream('world', 'slide', parents)
    assert not is_downstream('cycle', 'slide', {'cycle': 'cycle'})


def test_downstream_frames_follow_scene_reparenting_and_nested_components():
    frame = lambda name, parent: NS(frame_name=name, parent_frame=parent)
    moving = NS(obj_name='moving', parent_frame='tool', ref_frames=[
        frame('moving_measure', 'moving'), frame('moving_mount', 'moving')])
    nested = NS(obj_name='nested', parent_frame='moving_mount', ref_frames=[
        frame('nested_measure', 'nested')])
    fixed = NS(obj_name='fixed', parent_frame='world', ref_frames=[
        frame('fixed_measure', 'fixed')])
    scene = NS(objects_in_scene=[moving, nested, fixed], ref_frames_in_scene=[])
    # Scene hierarchy overrides a stale TF relationship for a reparented object.
    parents = {'slide': 'world', 'tool': 'slide', 'moving': 'world'}
    assert downstream_component_frames(scene, 'slide', parents) == [
        'moving_measure', 'moving_mount', 'nested_measure']
    moving.parent_frame = 'world'
    assert downstream_component_frames(scene, 'slide', parents) == []


def test_missing_tf_bridge_keeps_movement_unresolved():
    from joint_listener.tracking import IncompleteHierarchyError

    scene = NS(objects_in_scene=[NS(
        obj_name='part', parent_frame='tool', ref_frames=[
            NS(frame_name='measure', parent_frame='part')])], ref_frames_in_scene=[])
    with pytest.raises(IncompleteHierarchyError):
        downstream_component_frames(scene, 'slide', {'slide': 'world'})
    assert downstream_component_frames(
        scene, 'slide', {'slide': 'world', 'tool': 'slide'}) == ['measure']
