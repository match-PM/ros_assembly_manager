"""Exercise form edits through real Qt controls and check saved descriptions."""

from copy import deepcopy

from PyQt6.QtWidgets import QApplication, QTextEdit

from assembly_scene_viewer.py_modules.AssemblyDescriptionModifier import (
    AssemblyDescriptionModifier, IDENTITY_TRANSFORM,
)
from assembly_scene_viewer.py_modules.AssemblyJsonModifier import AssemblyModifierWidget
from assembly_scene_viewer.py_modules.DescriptionForms import ObjectForm


def test_frame_controls_preserve_metadata_and_remove_disabled_constraints(tmp_path):
    app = QApplication.instance() or QApplication([])
    model = AssemblyDescriptionModifier.create_new('Part', 'Component')
    frame = {
        'name': 'pickup', 'type': 'frame', 'transformation': deepcopy(IDENTITY_TRANSFORM),
        'constraints': {'centroid': {
            'refFrameNames': ['a', 'b'], 'dim': 'xyz', 'offsetValues': [0.0, 0.0, 0.0]}},
        'properties': {}, 'vendorMetadata': {'revision': 5},
    }
    model.mounting_references['ref_frames'] = [frame]
    widget = AssemblyModifierWidget()
    widget._load_model(model)
    widget.tree.setCurrentItem(widget.tree.topLevelItem(1).child(0).child(0))
    assert not widget.findChildren(QTextEdit)
    form = widget.form
    form.controls['transformation'].controls['translation'].controls['X'].control.setValue(1.25)
    form.controls['constraints'].groups['centroid'].setChecked(False)
    properties = form.controls['properties']
    properties.groups['glueLineProperties'].setChecked(True)
    line = properties.controls['glueLineProperties']
    line.controls['gCodeFileName'].control.setText('line.nc')
    line.controls['use_1K_dispenser'].control.setChecked(True)
    assert widget.apply_changes()
    model.save(str(tmp_path / 'component.json'))
    saved = AssemblyDescriptionModifier.from_file(str(tmp_path / 'component.json'))
    result = saved.mounting_references['ref_frames'][0]
    assert result['transformation']['translation']['X'] == 1.25
    assert result['constraints'] == {}
    assert result['vendorMetadata'] == {'revision': 5}
    assert result['properties']['glueLineProperties']['isGlueLineFrame'] is True
    assert result['properties']['glueLineProperties']['gCodeFileName'] == 'line.nc'
    assert set(result['properties']) == {'glueLineProperties'}
    widget.set_dirty(False)
    widget.close()
    app.processEvents()


def test_instruction_controls_edit_endpoints_and_add_prerequisites():
    app = QApplication.instance() or QApplication([])
    data = {
        'name': 'place', 'type': 'FrameMatch',
        'assemblyFrame': {'component': 'Part-1', 'frame': 'pickup'},
        'targetFrame': {'component': 'Base-1', 'frame': 'site'},
        'prerequisites': {},
    }
    form = ObjectForm(data, 'assemblyConstraints', {
        'components': ['Part-1', 'Base-1'], 'frames': ['pickup', 'site', 'dot']})
    assert form.value() == data
    form.controls['targetFrame'].controls['frame'].control.setCurrentText('other_site')
    prerequisites = form.controls['prerequisites']
    points = prerequisites.controls['gluePrerequisites']
    points.add_row()
    reference = points.rows[0][1]
    reference.controls['component'].control.setCurrentText('Base-1')
    reference.controls['frame'].control.setCurrentText('dot')
    result = form.value()
    assert result['targetFrame']['frame'] == 'other_site'
    assert result['prerequisites'] == {
        'gluePrerequisites': [{'component': 'Base-1', 'frame': 'dot'}]}
    form.close()
    app.processEvents()
