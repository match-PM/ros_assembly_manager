from copy import deepcopy

import assembly_manager_interfaces.msg as messages

from assembly_scene_viewer.py_modules.AssemblyDescriptionModifier import (
    AssemblyDescriptionModifier,
)
from assembly_scene_viewer.py_modules.scene_display import (
    frame_properties_html,
    instruction_html,
    object_html,
    scene_overview_html,
)


def valid_document():
    return {
        'type': 'Assembly',
        'name': 'Example',
        'customExporterField': {'preserve': True},
        'mountingDescription': {
            'components': [
                {'name': 'Moving-1', 'type': 'component',
                 'frameProperties': {'pickup': {'assemblyProperties': {
                     'isAssemblyFrame': True}}}},
                {'name': 'Base-1', 'type': 'component'},
            ],
            'assemblyConstraints': [{
                'name': 'place',
                'type': 'FrameMatch',
                'assemblyFrame': {'component': 'Moving-1', 'frame': 'pickup'},
                'targetFrame': {'component': 'Base-1', 'frame': 'site'},
                'prerequisites': {
                    'gluePrerequisites': [
                        {'component': 'Base-1', 'frame': 'dot'}],
                    'glueLinePrerequisite': [],
                    'componentPrerequisites': [{'component': 'Previous-1'}],
                },
            }],
            'mountingReferences': {'spawningOrigin': 'world'},
        },
    }


def test_assembly_modifier_preserves_and_edits_new_schema():
    model = AssemblyDescriptionModifier(valid_document())
    component = deepcopy(model.components[0])
    component['frameProperties']['pickup']['assemblyProperties'][
        'associatedComponent'] = 'Base-1'
    model.replace_item('components', 0, component)
    duplicate = model.duplicate_item('instructions', 0)

    result = model.to_dict()
    assert result['customExporterField'] == {'preserve': True}
    assert result['mountingDescription']['components'][0][
        'frameProperties']['pickup']['assemblyProperties'][
            'associatedComponent'] == 'Base-1'
    assert duplicate == 1
    assert result['mountingDescription']['assemblyConstraints'][1][
        'name'] == 'place_copy'


def test_assembly_modifier_supports_both_instruction_templates():
    model = AssemblyDescriptionModifier.create_new('Example')
    plane_index = model.add_instruction('PlaneMatch')
    frame_index = model.add_instruction('FrameMatch')
    assert model.instructions[plane_index]['type'] == 'PlaneMatch'
    assert len(model.instructions[plane_index]['description']) == 3
    assert model.instructions[frame_index]['type'] == 'FrameMatch'
    assert 'alignAxes' not in model.instructions[frame_index]


def test_component_description_round_trip_preserves_properties_and_sparse_constraints(tmp_path):
    data = {
        'type': 'Component', 'name': 'Part', 'documentUnits': 'mm',
        'exporterMetadata': {'revision': 7},
        'mountingDescription': {
            'mountingReferences': {
                'spawningOrigin': 'fixture',
                'ref_frames': [{
                    'name': 'dot', 'type': 'frame',
                    'properties': {'glueLineProperties': {
                        'isGlueLineFrame': True, 'gCodeFileName': 'line.nc'}},
                    'constraints': {'transform': {'refFrame': 'origin'}},
                }],
                'ref_axes': [], 'ref_planes': [],
            },
        },
    }
    model = AssemblyDescriptionModifier(data)
    assert model.to_dict() == data
    metadata = deepcopy(model.metadata)
    metadata['description'] = 'Updated component'
    model.replace_metadata(metadata)
    path = model.save(str(tmp_path / 'component.json'))
    loaded = AssemblyDescriptionModifier.from_file(str(path))
    assert loaded.metadata['type'] == 'Component'
    assert loaded.metadata['description'] == 'Updated component'
    assert loaded.mounting_references == data['mountingDescription']['mountingReferences']


def test_scene_display_covers_scene_and_object_fields():
    scene = messages.ObjectScene()
    obj = messages.Object()
    obj.obj_name = 'Moving-1'
    obj.uuid = 'instance-id'
    obj.component_type_uuid = 'type-id'
    obj.parent_file_path = '/component.json'
    obj.properties.is_gripped = True
    scene.objects_in_scene.append(obj)
    scene.occupied_spawning_frames.append('fixture')

    overview = scene_overview_html(scene)
    details = object_html(obj)
    assert 'Moving-1' in overview
    assert 'fixture' in overview
    assert 'instance-id' in details
    assert 'Gripped: yes' in details


def test_scene_display_shows_frame_roles_and_prerequisite_state():
    scene = messages.ObjectScene()
    frame = messages.RefFrame()
    frame.frame_name = 'Base-1_dot'
    frame.properties.glue_pt_frame_properties.is_glue_point = True
    frame.properties.glue_pt_frame_properties.has_been_placed = True
    frame.properties.assembly_frame_properties.is_target_frame = True
    frame.properties.assembly_frame_properties.associated_component = 'Moving-1'
    frame.properties.assembly_frame_properties.associated_frame = 'Moving-1_pickup'
    scene.ref_frames_in_scene.append(frame)

    instruction = messages.FrameAssemblyInstruction()
    instruction.id = 'place'
    instruction.frame_match.assembly_frame.component = 'Moving-1'
    instruction.frame_match.assembly_frame.frame = 'pickup'
    instruction.frame_match.target_frame.component = 'Base-1'
    instruction.frame_match.target_frame.frame = 'site'
    prerequisite = messages.FrameReference()
    prerequisite.component = 'Base-1'
    prerequisite.frame = 'dot'
    instruction.prerequisites.glue_prerequisites.append(prerequisite)

    assert 'associated component Moving-1' in frame_properties_html(frame)
    details = instruction_html(instruction, scene)
    assert 'FrameMatch' in details
    assert 'Glue point: Base-1 / dot — <b>met</b>' in details
