"""Mounting completion is distinct from instruction initialization."""

from copy import deepcopy
from unittest.mock import Mock

import assembly_manager_interfaces.msg as msg
import assembly_manager_interfaces.srv as srv
import pytest

from assembly_scene_publisher.py_modules.AssemblyScene import AssemblyManagerScene
from assembly_scene_publisher.py_modules.AssemblySceneAnalyzer import AssemblySceneAnalyzer


@pytest.mark.parametrize('kind', ['plane', 'frame'])
def test_completion_can_be_set_and_reset_and_unknown_ids_fail(kind):
    scene = msg.ObjectScene()
    instruction = (msg.AssemblyInstruction() if kind == 'plane'
                   else msg.FrameAssemblyInstruction())
    instruction.id = 'mount-1'
    instructions = (scene.assembly_instructions if kind == 'plane'
                    else scene.frame_assembly_instructions)
    instructions.append(instruction)
    manager = AssemblyManagerScene.__new__(AssemblyManagerScene)
    manager.scene = scene
    manager.assembly_scene_analyzer = AssemblySceneAnalyzer(scene)
    manager.publish_information = Mock()
    request = srv.SetAssemblyInstructionCompleted.Request(instruction_id='mount-1', completed=True)
    assert not manager.set_assembly_instruction_completed(request).success
    instruction.activated = True
    assert not manager.assembly_scene_analyzer.check_assembly_instruction_completed('mount-1')
    assert manager.set_assembly_instruction_completed(request).success
    assert manager.assembly_scene_analyzer.check_assembly_instruction_completed('mount-1')
    request.completed = False
    assert manager.set_assembly_instruction_completed(request).success
    assert not instruction.completed
    request.instruction_id = 'unknown'
    assert not manager.set_assembly_instruction_completed(request).success
    assert manager.publish_information.call_count == 2


@pytest.mark.parametrize('kind', ['plane', 'frame'])
def test_reimport_preserves_completion_only_for_the_same_definition(kind):
    original = msg.AssemblyInstruction() if kind == 'plane' else msg.FrameAssemblyInstruction()
    original.id = 'mount-1'
    original.completed = True
    original.activated = True
    items = [original]
    imported = deepcopy(original)
    imported.completed, imported.activated = False, False
    AssemblyManagerScene._store_instruction(imported, items)
    assert items[0].completed
    changed = deepcopy(imported)
    changed.parent_file_path = 'different-assembly.json'
    changed.completed = False
    AssemblyManagerScene._store_instruction(changed, items)
    assert not items[0].completed


def test_missing_endpoint_invalidates_completion():
    scene = msg.ObjectScene()
    instruction = msg.AssemblyInstruction(id='mount-1', component_1='a-1', component_2='b-1',
                                          activated=True, completed=True)
    scene.assembly_instructions.append(instruction)
    manager = AssemblyManagerScene.__new__(AssemblyManagerScene)
    manager.scene = scene
    manager.assembly_scene_analyzer = AssemblySceneAnalyzer(scene)
    manager.initialize_pending_instructions()
    assert not instruction.activated
    assert not instruction.completed


def test_description_import_defaults_to_no_spawning():
    assert not srv.CreateAssemblyInstructionFromDescription.Request().spawn_components


def test_component_completion_requires_all_of_its_mountings():
    scene = msg.ObjectScene()
    component = msg.Object(obj_name='a-1')
    component.properties.is_placed = True
    scene.objects_in_scene.append(component)
    for key in ('one', 'two'):
        scene.assembly_instructions.append(msg.AssemblyInstruction(
            id=key, component_1='a-1', component_2='b-1',
            component_1_is_moving_part=True, activated=True))
    manager = AssemblyManagerScene.__new__(AssemblyManagerScene)
    manager.scene = scene
    manager.assembly_scene_analyzer = AssemblySceneAnalyzer(scene)
    manager.publish_information = Mock()
    for key in ('one', 'two'):
        response = manager.set_assembly_instruction_completed(
            srv.SetAssemblyInstructionCompleted.Request(instruction_id=key, completed=True))
        assert response.success
        assert component.properties.is_assembled == (key == 'two')
        assert component.properties.is_placed
    assert manager.set_assembly_instruction_completed(
        srv.SetAssemblyInstructionCompleted.Request(instruction_id='one', completed=False)).success
    assert not component.properties.is_assembled
