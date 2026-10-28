"""Queries follow active frame dependencies and report current process states."""

import assembly_manager_interfaces.msg as msg
import pytest

from assembly_scene_publisher.py_modules.AssemblySceneAnalyzer import (
    AssemblySceneAnalyzer,
)
from assembly_scene_publisher.py_modules.scene_errors import (
    AssemblyFrameNotFoundError,
    AssemblyInstructionNotFoundError,
    GrippingFrameNotFoundError,
    RefFrameNotFoundError,
    TargetFrameNotFoundError,
)


def add_frame(scene, name, *dependencies):
    frame = msg.RefFrame()
    frame.frame_name = name
    if dependencies:
        frame.constraints.centroid.is_active = True
        frame.constraints.centroid.ref_frame_names = list(dependencies)
    scene.ref_frames_in_scene.append(frame)
    return frame


@pytest.mark.parametrize('role', ['gripping', 'target', 'assembly'])
@pytest.mark.parametrize('sensor', ['laser', 'vision', 'measurement'])
def test_role_queries_follow_transitive_dependencies(role, sensor):
    scene = msg.ObjectScene()
    root = add_frame(scene, 'root', 'intermediate', 'vision')
    add_frame(scene, 'intermediate', 'laser', 'vision')
    laser = add_frame(scene, 'laser')
    laser.properties.laser_frame_properties.is_laser_frame = True
    laser.properties.laser_frame_properties.has_been_measured = True
    vision = add_frame(scene, 'vision')
    vision.properties.vision_frame_properties.is_vision_frame = True
    unrelated = add_frame(scene, 'unrelated')
    unrelated.properties.laser_frame_properties.is_laser_frame = True
    # Exercise a dependency stored on a different component as well.
    scene.ref_frames_in_scene.remove(laser)
    component = msg.Object()
    component.obj_name = 'other'
    component.ref_frames.append(laser)
    scene.objects_in_scene.append(component)
    group = (root.properties.gripping_frame_properties if role == 'gripping'
             else root.properties.assembly_frame_properties)
    setattr(group, f'is_{role}_frame', True)
    analyzer = AssemblySceneAnalyzer(scene)
    query = getattr(analyzer, f'get_{sensor}_frames_for_{role}_frame')
    expected = {'laser': [('laser', True)], 'vision': [('vision', False)],
                'measurement': [('laser', True), ('vision', False)]}[sensor]
    assert [(frame.frame_name, measured)
            for frame, measured in query('root')] == expected
    vision.properties.vision_frame_properties.has_been_measured = True
    assert all(measured for _, measured in query('root'))


def test_dual_role_measurements_and_input_frame_inclusion():
    scene = msg.ObjectScene()
    frame = add_frame(scene, 'both')
    frame.properties.gripping_frame_properties.is_gripping_frame = True
    frame.properties.laser_frame_properties.is_laser_frame = True
    frame.properties.vision_frame_properties.is_vision_frame = True
    frame.properties.laser_frame_properties.has_been_measured = True
    analyzer = AssemblySceneAnalyzer(scene)
    assert analyzer.get_laser_frames_for_gripping_frame('both') == [(frame, True)]
    assert analyzer.get_vision_frames_for_gripping_frame('both') == [(frame, False)]
    assert analyzer.get_measurement_frames_for_frame('both') == [(frame, False)]
    frame.properties.vision_frame_properties.has_been_measured = True
    assert analyzer.get_measurement_frames_for_frame('both') == [(frame, True)]


def test_inactive_dependencies_empty_results_missing_references_and_cycles():
    scene = msg.ObjectScene()
    root = add_frame(scene, 'root', 'missing')
    root.constraints.centroid.is_active = False
    analyzer = AssemblySceneAnalyzer(scene)
    assert analyzer.get_measurement_frames_for_frame('root') == []
    with pytest.raises(RefFrameNotFoundError):
        analyzer.get_measurement_frames_for_frame('absent')
    root.constraints.centroid.is_active = True
    with pytest.raises(RefFrameNotFoundError):
        analyzer.get_measurement_frames_for_frame('root')
    add_frame(scene, 'missing', 'root')
    with pytest.raises(ValueError, match='Cycle detected'):
        analyzer.get_measurement_frames_for_frame('root')


@pytest.mark.parametrize('role,error', [
    ('gripping', GrippingFrameNotFoundError),
    ('target', TargetFrameNotFoundError),
    ('assembly', AssemblyFrameNotFoundError),
])
def test_role_queries_reject_incorrect_roles(role, error):
    scene = msg.ObjectScene()
    add_frame(scene, 'plain')
    analyzer = AssemblySceneAnalyzer(scene)
    with pytest.raises(error):
        getattr(analyzer, f'get_measurement_frames_for_{role}_frame')('plain')


def test_glue_states_are_reported_per_frame_and_independently():
    scene = msg.ObjectScene()
    first = add_frame(scene, 'first').properties.glue_pt_frame_properties
    first.is_glue_point = True
    first.has_been_placed = True
    second = add_frame(scene, 'second').properties.glue_pt_frame_properties
    second.is_glue_point = True
    second.has_been_cured = True
    line = add_frame(scene, 'line').properties.glue_line_frame_properties
    line.is_glue_line_frame = True
    analyzer = AssemblySceneAnalyzer(scene)
    assert analyzer.get_glue_pt_frames_placed(['first', 'second']) == {
        'first': True, 'second': False}
    assert analyzer.get_glue_pt_frames_cured(['first', 'second']) == {
        'first': False, 'second': True}
    assert analyzer.get_glue_line_frames_placed(['line']) == {'line': False}
    line.has_been_placed = True
    assert analyzer.get_glue_line_frames_placed(['line']) == {'line': True}
    for query in (analyzer.get_glue_pt_frames_placed,
                  analyzer.get_glue_pt_frames_cured,
                  analyzer.get_glue_line_frames_placed):
        assert query([]) == {}
        with pytest.raises(RefFrameNotFoundError):
            query(['absent'])
    with pytest.raises(ValueError):
        analyzer.get_glue_pt_frames_cured(['line'])
    with pytest.raises(ValueError):
        analyzer.get_glue_line_frames_placed(['first'])


def test_gripper_configuration_is_a_copy_of_declared_compatibility():
    scene = msg.ObjectScene()
    frame = add_frame(scene, 'grip')
    properties = frame.properties.gripping_frame_properties
    properties.is_gripping_frame = True
    properties.compatible_grippers = ['gripper_a', 'gripper_b']
    properties.compatible_gripper_tips = ['tip_a']
    analyzer = AssemblySceneAnalyzer(scene)
    result = analyzer.get_gripper_configuration_for_gripping_frame('grip')
    assert result == properties
    result.compatible_grippers.append('other')
    result.compatible_gripper_tips.clear()
    assert list(properties.compatible_grippers) == ['gripper_a', 'gripper_b']
    assert list(properties.compatible_gripper_tips) == ['tip_a']
    properties.is_gripping_frame = False
    with pytest.raises(GrippingFrameNotFoundError):
        analyzer.get_gripper_configuration_for_gripping_frame('grip')


@pytest.mark.parametrize('kind', ['plane_first_moves', 'plane_second_moves', 'frame'])
def test_instruction_components_by_id(kind):
    scene = msg.ObjectScene()
    if kind == 'frame':
        instruction = msg.FrameAssemblyInstruction()
        instruction.frame_match.assembly_frame.component = 'moving'
        instruction.frame_match.target_frame.component = 'target'
        scene.frame_assembly_instructions.append(instruction)
    else:
        instruction = msg.AssemblyInstruction()
        instruction.component_1_is_moving_part = kind == 'plane_first_moves'
        instruction.component_1 = ('moving' if instruction.component_1_is_moving_part
                                   else 'target')
        instruction.component_2 = ('target' if instruction.component_1_is_moving_part
                                   else 'moving')
        scene.assembly_instructions.append(instruction)
    instruction.id = 'place_part'
    analyzer = AssemblySceneAnalyzer(scene)
    assert analyzer.get_assembly_component_for_instruction('place_part') == 'moving'
    assert analyzer.get_target_component_for_instruction('place_part') == 'target'
    for query in (analyzer.get_assembly_component_for_instruction,
                  analyzer.get_target_component_for_instruction):
        with pytest.raises(AssemblyInstructionNotFoundError):
            query('missing')


@pytest.mark.parametrize('first_moves', [True, False])
@pytest.mark.parametrize('name,expected', [
    ('Part-1', 'Part'),
    ('Part-A-12', 'Part-A'),
    ('Part_A-2', 'Part_A'),
    ('Part', 'Part'),
    ('Part-A', 'Part-A'),
    ('Part-', 'Part-'),
])
def test_instruction_component_types(first_moves, name, expected):
    scene = msg.ObjectScene()
    plane = msg.AssemblyInstruction()
    plane.id = 'plane'
    plane.component_1 = name if first_moves else 'Target-Type-3'
    plane.component_2 = 'Target-Type-3' if first_moves else name
    plane.component_1_is_moving_part = first_moves
    scene.assembly_instructions.append(plane)
    frame = msg.FrameAssemblyInstruction()
    frame.id = 'frame'
    frame.frame_match.assembly_frame.component = name
    frame.frame_match.target_frame.component = 'Target-Type-3'
    scene.frame_assembly_instructions.append(frame)
    analyzer = AssemblySceneAnalyzer(scene)
    for instruction_id in ('plane', 'frame'):
        assert analyzer.get_assembly_component_type_for_instruction(
            instruction_id) == expected
        assert analyzer.get_target_component_type_for_instruction(
            instruction_id) == 'Target-Type'
    for query in (analyzer.get_assembly_component_type_for_instruction,
                  analyzer.get_target_component_type_for_instruction):
        with pytest.raises(AssemblyInstructionNotFoundError):
            query('missing')
