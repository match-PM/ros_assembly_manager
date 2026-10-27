"""Regression tests for explicit properties and both assembly match formats."""

from copy import deepcopy
from unittest.mock import Mock

import numpy as np
import pytest
import assembly_manager_interfaces.msg as msg
import assembly_manager_interfaces.srv as srv
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation

from assembly_scene_publisher.py_modules.AssemblyDescriptionHandler import (
    AssemblyDescriptionHandler,
)
from assembly_scene_publisher.py_modules.AssemblyScene import AssemblyManagerScene
from assembly_scene_publisher.py_modules.AssemblySceneAnalyzer import (
    AssemblySceneAnalyzer,
)
from assembly_scene_publisher.py_modules.FrameMatchHandler import FrameMatchHandler
from assembly_scene_publisher.py_modules.frame_constraints import FrameConstraintsHandler
from assembly_scene_publisher.py_modules.frame_properties import import_frame_properties
from assembly_scene_publisher.py_modules.scene_errors import AssemblyTransformationError


def make_scene():
    scene = msg.ObjectScene()
    for component_name, frame_name in (('moving-1', 'pickup'), ('fixed-1', 'site')):
        component = msg.Object()
        component.obj_name = component_name
        frame = msg.RefFrame()
        frame.frame_name = f'{component_name}_{frame_name}'
        frame.parent_frame = component_name
        frame.pose.orientation.w = 1.0
        component.ref_frames.append(frame)
        scene.objects_in_scene.append(component)
    return scene


def frame_instruction():
    return AssemblyDescriptionHandler._build_instruction_request({
        'name': 'place',
        'type': 'FrameMatch',
        'component_1': 'moving-1',
        'component_2': 'fixed-1',
        'assemblyFrame': {'component': 'moving-1', 'frame': 'pickup'},
        'targetFrame': {'component': 'fixed-1', 'frame': 'site'},
    }, 0.001).assembly_instruction


def test_explicit_properties_and_assembly_overrides():
    properties = import_frame_properties({
        'glueLineProperties': {
            'isGlueLineFrame': True,
            'gCodeFileName': 'line.nc',
            'use_1K_dispenser': True,
        },
        'grippingProperties': {'isGrippingFrame': True},
    })
    assert properties.glue_line_frame_properties.g_code_file_name == 'line.nc'
    assert properties.glue_line_frame_properties.use_1k_dispenser
    assert not properties.glue_pt_frame_properties.is_glue_point
    import_frame_properties({
        'assemblyProperties': {
            'isAssemblyFrame': True,
            'associatedFrame': 'site',
            'associatedComponent': 'fixed-1',
        },
    }, 'moving-1', properties)
    assert properties.gripping_frame_properties.is_gripping_frame
    assert properties.assembly_frame_properties.associated_frame == 'fixed-1_site'


def test_scene_does_not_infer_roles_from_name():
    scene = make_scene()
    manager = AssemblyManagerScene.__new__(AssemblyManagerScene)
    manager.scene = scene
    manager.logger = Mock()
    manager.publish_information = Mock()
    manager.check_if_frame_exists = lambda name: name == 'moving-1'
    manager.assembly_scene_analyzer = AssemblySceneAnalyzer(scene)
    frame = msg.RefFrame()
    frame.frame_name = 'moving-1_Vision_Laser_Glue_Gripping'
    frame.parent_frame = 'moving-1'
    manager.add_ref_frame_to_scene(frame)
    assert frame.properties == msg.RefFrameProperties()


def test_sparse_constraints_and_uppercase_quaternion_are_preserved():
    data = {'transform': {
        'refFrame': 'pickup',
        'transform': {
            'translation': {'X': 10.0, 'Y': 0.0, 'Z': 0.0},
            'rotation': {'X': 0.0, 'Y': 0.0, 'Z': 0.6, 'W': 0.8},
        },
    }}
    original = deepcopy(data)
    handler = FrameConstraintsHandler.return_handler_from_dict(
        data, component_name='moving-1', logger=Mock())
    assert handler.transform.ref_frame == 'moving-1_pickup'
    assert handler.transform.transform.rotation.w == 0.8
    assert not handler.centroid.is_active
    assert not handler.orthogonal.is_active
    assert not handler.in_plane.is_active
    assert data == original
    empty = FrameConstraintsHandler.return_handler_from_dict({}, logger=Mock())
    assert empty.get_frame_references() == []


def test_plane_match_prerequisites_and_new_moving_flag():
    data = {
        'name': 'planes', 'type': 'PlaneMatch',
        'component_1': 'fixed-1', 'component_2': 'moving-1',
        'move_component_1': False,
        'description': {
            f'planeMatch_{i}': {
                'planeNameComponent_1': f'p{i}',
                'planeNameComponent_2': f'q{i}',
                'planeOffset': 4.5, 'invertNormalVector': True,
            } for i in range(1, 4)
        },
        'prerequisites': {
            'gluePrerequisites': [{'component': 'fixed-1', 'frame': 'dot'}],
            'glueLinePrerequisite': [{'component': 'fixed-1', 'frame': 'line'}],
            'componentPrerequisites': [{'component': 'previous-1'}],
        },
    }
    instruction = AssemblyDescriptionHandler._build_instruction_request(
        data, 0.001).assembly_instruction
    assert isinstance(instruction, msg.AssemblyInstruction)
    assert not hasattr(instruction, 'frame_match')
    assert not instruction.component_1_is_moving_part
    assert instruction.plane_match_1.plane_offset == pytest.approx(0.0045)
    assert instruction.prerequisites.glue_prerequisites[0].frame == 'dot'
    assert instruction.prerequisites.glue_line_prerequisites[0].frame == 'line'
    assert list(instruction.prerequisites.component_prerequisites) == ['previous-1']
    data['moveComponent_1'] = data.pop('move_component_1')
    del data['type']
    legacy = AssemblyDescriptionHandler._build_instruction_request(
        data, 0.001).assembly_instruction
    assert legacy == instruction


def test_frame_match_aligns_existing_frames(monkeypatch):
    scene = make_scene()
    analyzer = AssemblySceneAnalyzer(scene)
    instruction = frame_instruction()
    assert isinstance(instruction, msg.FrameAssemblyInstruction)
    assert not hasattr(instruction.frame_match, 'align_axes')
    assert not hasattr(instruction, 'component_1')
    assembly = scene.objects_in_scene[0].ref_frames[0]
    target = scene.objects_in_scene[1].ref_frames[0]
    assembly.pose.position.x = 0.5
    target.pose.position.y = 0.7
    target.pose.orientation.z = np.sqrt(0.5)
    target.pose.orientation.w = np.sqrt(0.5)
    initial_poses = (deepcopy(assembly.pose), deepcopy(target.pose))
    parents = {}
    for name, x in (('moving-1', 1.0), ('fixed-1', 3.0)):
        transform = TransformStamped()
        transform.transform.rotation.w = 1.0
        transform.transform.translation.x = x
        parents[name] = transform
    monkeypatch.setattr(
        'assembly_scene_publisher.py_modules.FrameMatchHandler.'
        'get_transform_for_frame_in_world',
        lambda name, _: parents[name],
    )
    handler = FrameMatchHandler(analyzer, None)
    handler.attach(instruction)
    scene.frame_assembly_instructions.append(instruction)
    assert analyzer.get_assembly_frame_for_instruction('place') is assembly
    assert analyzer.get_target_frame_for_instruction('place') is target
    assert analyzer.get_target_frame_for_assembly_frame(assembly.frame_name) == target.frame_name
    assert analyzer.get_assembly_frame_for_target_frame(target.frame_name) == assembly.frame_name
    assert analyzer.get_planes_for_instruction('place') == []
    assert [len(obj.ref_frames) for obj in scene.objects_in_scene] == [1, 1]
    assert (assembly.pose, target.pose) == initial_poses

    delta = handler.calculate(instruction)
    rotation = Rotation.from_quat([
        delta.orientation.x, delta.orientation.y,
        delta.orientation.z, delta.orientation.w,
    ])
    translated = rotation.apply([1.5, 0.0, 0.0]) + np.array([
        delta.position.x, delta.position.y, delta.position.z,
    ])
    assert np.allclose(translated, [3.0, 0.7, 0.0])
    assert np.allclose(
        rotation.as_matrix(), Rotation.from_euler('z', 90, degrees=True).as_matrix())


def test_frame_match_rejects_conflicting_associations_without_mutation():
    scene = make_scene()
    handler = FrameMatchHandler(AssemblySceneAnalyzer(scene), None)
    target_properties = (
        scene.objects_in_scene[1].ref_frames[0].properties.assembly_frame_properties)
    target_properties.associated_frame = 'other'
    with pytest.raises(AssemblyTransformationError):
        handler.attach(frame_instruction())
    assembly_properties = (
        scene.objects_in_scene[0].ref_frames[0].properties.assembly_frame_properties)
    assert not assembly_properties.is_assembly_frame


def test_prerequisite_readiness_tracks_dispensing_and_assembly():
    scene = make_scene()
    instruction = frame_instruction()
    prerequisite = msg.FrameReference(component='fixed-1', frame='site')
    instruction.prerequisites.glue_line_prerequisites = [prerequisite]
    instruction.prerequisites.component_prerequisites = ['moving-1']
    scene.frame_assembly_instructions.append(instruction)
    analyzer = AssemblySceneAnalyzer(scene)
    assert len(analyzer.check_assembly_prerequisites('place')) == 2
    glue = scene.objects_in_scene[1].ref_frames[0].properties.glue_line_frame_properties
    glue.is_glue_line_frame = True
    glue.has_been_placed = True
    scene.objects_in_scene[0].properties.is_assembled = True
    assert analyzer.check_assembly_prerequisites('place') == []


def test_separate_scene_lists_and_shared_lookup():
    from rclpy.serialization import serialize_message, deserialize_message

    scene = make_scene()
    manager = AssemblyManagerScene.__new__(AssemblyManagerScene)
    manager.scene = scene
    manager.logger = Mock()
    manager.publish_information = Mock()
    manager.assembly_scene_analyzer = AssemblySceneAnalyzer(scene)
    manager.frame_match_handler = FrameMatchHandler(
        manager.assembly_scene_analyzer, None)
    frame_instruction_msg = frame_instruction()
    assert manager.create_frame_assembly_instructions(frame_instruction_msg)
    assert not scene.assembly_instructions
    assert list(scene.frame_assembly_instructions) == [frame_instruction_msg]

    # Keep plane geometry outside this storage/lookup test.
    manager._check_create_instruction = Mock(return_value=True)
    manager.calculate_assembly_transformation = Mock()
    plane_instruction = msg.AssemblyInstruction(
        id='planes', component_1='fixed-1', component_2='moving-1',
        component_1_is_moving_part=True)
    assert manager.create_assembly_instructions(plane_instruction)
    assert list(scene.assembly_instructions) == [plane_instruction]
    assert len(scene.frame_assembly_instructions) == 1
    assert manager.get_assembly_instruction_by_id('place') is frame_instruction_msg
    assert manager.get_assembly_instruction_by_id('planes') is plane_instruction
    analyzer = manager.assembly_scene_analyzer
    assert analyzer.get_assembly_instruction_by_name('place') is frame_instruction_msg
    assert len(analyzer.get_instructions_for_component('moving-1')) == 2
    assert not analyzer.is_component_stationary('moving-1')
    assert analyzer.get_assembly_instruction('moving-1', 'fixed-1').id == 'place'

    # The new scene layout must also survive ROS serialization.
    restored = deserialize_message(serialize_message(scene), msg.ObjectScene)
    assert restored == scene


def test_ids_are_unique_across_instruction_lists():
    scene = make_scene()
    scene.assembly_instructions.append(msg.AssemblyInstruction(id='place'))
    manager = AssemblyManagerScene.__new__(AssemblyManagerScene)
    manager.scene = scene
    manager.frame_match_handler = Mock()
    with pytest.raises(ValueError, match='already used'):
        manager.create_frame_assembly_instructions(frame_instruction())
    manager.frame_match_handler.attach.assert_not_called()
    assert not scene.frame_assembly_instructions


def test_description_uses_typed_create_services(monkeypatch):
    handler = AssemblyDescriptionHandler.__new__(AssemblyDescriptionHandler)
    handler._logger = Mock()
    handler._object_scene = Mock(scene=make_scene())
    handler._scene_analyzer = AssemblySceneAnalyzer(handler._object_scene.scene)
    handler._apply_component_frame_properties = Mock()
    handler._create_assembly_instructions = Mock(return_value=(
        srv.CreateAssemblyInstructions.Response(success=True)))
    handler._create_frame_assembly_instructions = Mock(return_value=(
        srv.CreateFrameAssemblyInstructions.Response(success=True)))
    plane_request = srv.CreateAssemblyInstructions.Request()
    plane_request.assembly_instruction.id = 'plane'
    plane_request.assembly_instruction.component_1 = 'moving-1'
    plane_request.assembly_instruction.component_2 = 'fixed-1'
    frame_request = srv.CreateFrameAssemblyInstructions.Request()
    frame_request.assembly_instruction = frame_instruction()
    handler._load_json_file = Mock(return_value=({
        'type': 'Assembly',
        'mountingDescription': {'assemblyConstraints': [{}, {}]},
    }, None))
    handler._build_instruction_request = Mock(side_effect=[
        plane_request, frame_request])
    monkeypatch.setattr(
        'assembly_scene_publisher.py_modules.AssemblyDescriptionHandler.time.sleep',
        lambda _: None,
    )
    response = handler.create_assembly_instructions_callback(
        srv.CreateAssemblyInstructionFromDescription.Request(
            file_path='/test/assembly.json', spawn_components=False),
        srv.CreateAssemblyInstructionFromDescription.Response(),
    )
    assert response.success
    handler._create_assembly_instructions.assert_called_once()
    handler._create_frame_assembly_instructions.assert_called_once()
    assert plane_request.assembly_instruction.parent_file_path == '/test/assembly.json'
    assert frame_request.assembly_instruction.parent_file_path == '/test/assembly.json'


@pytest.mark.parametrize('kind', ['PlaneMatch', 'FrameMatch'])
@pytest.mark.parametrize('available', [True, False])
@pytest.mark.parametrize('missing_first', [True, False])
def test_incremental_import_includes_missing_components(
        monkeypatch, kind, available, missing_first):
    handler = AssemblyDescriptionHandler.__new__(AssemblyDescriptionHandler)
    handler._logger = Mock()
    handler._object_scene = Mock(scene=make_scene())
    handler._scene_analyzer = AssemblySceneAnalyzer(handler._object_scene.scene)
    handler._create_assembly_instructions = Mock(return_value=(
        srv.CreateAssemblyInstructions.Response(success=True)))
    handler._create_frame_assembly_instructions = Mock(return_value=(
        srv.CreateFrameAssemblyInstructions.Response(success=True)))

    def constraint(name, stationary):
        return {
            'name': name, 'type': kind,
            'component_1': 'moving-1', 'component_2': stationary,
            'assemblyFrame': {'component': 'moving-1', 'frame': 'pickup'},
            'targetFrame': {'component': stationary, 'frame': 'site'},
            'description': {
                f'planeMatch_{index}': {
                    'planeNameComponent_1': 'plane',
                    'planeNameComponent_2': 'plane',
                    'planeOffset': 0.0, 'invertNormalVector': False,
                } for index in range(1, 4)
            },
        }

    constraints = [constraint('missing', 'not_spawned-1')]
    if available:
        constraints.insert(1 if missing_first else 0,
                           constraint('ready', 'fixed-1'))
    handler._load_json_file = Mock(return_value=({
        'type': 'Assembly',
        'mountingDescription': {
            # Missing-component overrides must not abort incremental imports.
            'components': [{
                'name': 'not_spawned-1',
                'frameProperties': {'site': {
                    'assemblyProperties': {'isTargetFrame': True}}},
            }],
            'assemblyConstraints': constraints,
        },
    }, None))
    monkeypatch.setattr(
        'assembly_scene_publisher.py_modules.AssemblyDescriptionHandler.time.sleep',
        lambda _: None,
    )
    request = srv.CreateAssemblyInstructionFromDescription.Request(
        file_path='/test/assembly.json', spawn_components=False)
    response = handler.create_assembly_instructions_callback(
        request, srv.CreateAssemblyInstructionFromDescription.Response())
    assert response.success
    assert response.instruction_id == constraints[-1]['name']
    assert f'Created {len(constraints)} assembly instruction(s).' in response.message
    callback = (handler._create_assembly_instructions if kind == 'PlaneMatch'
                else handler._create_frame_assembly_instructions)
    assert callback.call_count == len(constraints)
    assert [call.args[0].assembly_instruction.id for call in callback.call_args_list] == [
        constraint['name'] for constraint in constraints]
    assert all(not call.args[0].assembly_instruction.activated
               for call in callback.call_args_list)


@pytest.mark.parametrize('kind', ['PlaneMatch', 'FrameMatch'])
def test_pending_instructions_activate_after_geometry_arrives(kind):
    from assembly_scene_publisher.py_modules.scene_errors import RefPlaneNotFoundError

    complete_scene = make_scene()
    scene = msg.ObjectScene()
    manager = AssemblyManagerScene.__new__(AssemblyManagerScene)
    manager.scene = scene
    manager.logger = Mock()
    manager.publish_information = Mock()
    manager.assembly_scene_analyzer = AssemblySceneAnalyzer(scene)
    manager.frame_match_handler = FrameMatchHandler(manager.assembly_scene_analyzer, None)
    manager._check_create_instruction = Mock(return_value=True)
    manager.calculate_assembly_transformation = Mock()
    if kind == 'FrameMatch':
        instruction = frame_instruction()
        create = manager.create_frame_assembly_instructions
    else:
        instruction = msg.AssemblyInstruction(
            id='planes', component_1='moving-1', component_2='fixed-1')
        create = manager.create_assembly_instructions

    # Ignore an activation flag supplied by the caller.
    instruction.activated = True
    assert create(instruction)
    assert not instruction.activated
    scene.objects_in_scene.append(complete_scene.objects_in_scene[0])
    manager.initialize_pending_instructions()
    assert not instruction.activated

    second = complete_scene.objects_in_scene[1]
    frames = list(second.ref_frames)
    second.ref_frames.clear()
    scene.objects_in_scene.append(second)
    manager.calculate_assembly_transformation.side_effect = RefPlaneNotFoundError('pending')
    manager.initialize_pending_instructions()
    assert not instruction.activated

    second.ref_frames.extend(frames)
    manager.calculate_assembly_transformation.side_effect = None
    manager.initialize_pending_instructions()
    assert instruction.activated
    manager.calculate_assembly_transformation.reset_mock()
    manager.initialize_pending_instructions()
    manager.calculate_assembly_transformation.assert_not_called()

    # Losing a component invalidates the match; respawning can activate it again.
    scene.objects_in_scene.pop()
    manager.initialize_pending_instructions()
    assert not instruction.activated
    scene.objects_in_scene.append(second)
    manager.initialize_pending_instructions()
    assert instruction.activated


def test_spawn_retries_instructions_after_reference_geometry(monkeypatch):
    handler = AssemblyDescriptionHandler.__new__(AssemblyDescriptionHandler)
    handler._logger = Mock()
    handler._object_scene = Mock()
    handler._load_json_file = Mock(return_value=({
        'name': 'moving-1',
        'mountingDescription': {'mountingReferences': {}},
    }, None))
    handler._build_spawn_request = Mock(return_value=srv.SpawnObject.Request())
    handler._spawn_object = Mock(return_value=srv.SpawnObject.Response(success=True))
    events = []
    for name in ('frames', 'axes', 'planes'):
        setattr(handler, f'_spawn_reference_{name}', Mock(
            side_effect=lambda *args, name=name: (events.append(name) or True, '')))
    handler._object_scene.initialize_pending_instructions.side_effect = (
        lambda: events.append('initialize'))
    monkeypatch.setattr(
        'assembly_scene_publisher.py_modules.AssemblyDescriptionHandler.time.sleep',
        lambda _: None)
    success, _ = handler.spawn_component(srv.SpawnComponentFromDescription.Request())
    assert success
    assert events == ['frames', 'axes', 'planes', 'initialize']
    handler._object_scene.publish_information.assert_called_once()


def test_measurement_invalidation_preserves_all_other_frame_properties():
    scene = make_scene()
    manager = AssemblyManagerScene.__new__(AssemblyManagerScene)
    manager.scene = scene
    manager.assembly_scene_analyzer = AssemblySceneAnalyzer(scene)
    manager.publish_information = Mock()
    frame = scene.objects_in_scene[0].ref_frames[0]
    frame.properties.vision_frame_properties.has_been_measured = True
    frame.properties.laser_frame_properties.has_been_measured = True
    frame.properties.gripping_frame_properties.is_gripping_frame = True
    frame.properties.glue_pt_frame_properties.has_been_placed = True
    untouched = scene.objects_in_scene[1].ref_frames[0]
    untouched.properties.vision_frame_properties.has_been_measured = True
    expected = deepcopy(frame)
    expected.properties.vision_frame_properties.has_been_measured = False
    expected.properties.laser_frame_properties.has_been_measured = False
    response = manager.invalidate_frame_measurements(
        srv.InvalidateFrameMeasurements.Request(frame_names=[frame.frame_name]))
    assert response.success
    assert frame == expected
    assert untouched.properties.vision_frame_properties.has_been_measured
    manager.publish_information.assert_called_once()


def test_measurement_invalidation_validates_entire_request_before_mutating():
    scene = make_scene()
    manager = AssemblyManagerScene.__new__(AssemblyManagerScene)
    manager.assembly_scene_analyzer = AssemblySceneAnalyzer(scene)
    manager.publish_information = Mock()
    frame = scene.objects_in_scene[0].ref_frames[0]
    frame.properties.vision_frame_properties.has_been_measured = True
    response = manager.invalidate_frame_measurements(
        srv.InvalidateFrameMeasurements.Request(frame_names=[frame.frame_name, 'missing']))
    assert not response.success
    assert frame.properties.vision_frame_properties.has_been_measured
    manager.publish_information.assert_not_called()
