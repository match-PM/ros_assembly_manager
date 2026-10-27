"""Typed form controls for component and assembly descriptions."""

import re
from copy import deepcopy

from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtWidgets import (
    QCheckBox, QComboBox, QDoubleSpinBox, QFileDialog, QFormLayout, QGroupBox,
    QHBoxLayout, QInputDialog, QLabel, QLineEdit, QPushButton, QSizePolicy,
    QVBoxLayout, QWidget,
)

from assembly_scene_viewer.py_modules.AssemblyDescriptionModifier import IDENTITY_TRANSFORM


FRAME_REFERENCE = {'component': '', 'frame': ''}
# Decimal places needed to represent one 10 nm step in each document unit.
LENGTH_DECIMALS = {'m': 8, 'cm': 6, 'mm': 5, 'um': 2}
PLANE_MATCH = {
    'planeNameComponent_1': '', 'planeNameComponent_2': '',
    'planeOffset': 0.0, 'invertNormalVector': False,
}
CONSTRAINTS = {
    'centroid': {'refFrameNames': [], 'dim': 'xyz', 'offsetValues': [0.0, 0.0, 0.0]},
    'inPlane': {'refFrameNames': [], 'planeOffset': 0.0, 'normalAxis': 'z'},
    'orthogonal': {
        'frame_1': '', 'frame_2': '', 'frame_3': '', 'distance_from_f1': 0.0,
        'unit_distance_from_f1': '%', 'distance_from_f1_f2_connection': 0.0,
        'frame_normal_plane_axis': 'z', 'frame_orthogonal_connection_axis': 'x',
    },
    'transform': {'refFrame': '', 'transform': IDENTITY_TRANSFORM},
}
PROPERTIES = {
    'visionProperties': {'isVisionFrame': False, 'hasBeenMeasured': False},
    'laserProperties': {'isLaserFrame': False, 'hasBeenMeasured': False},
    'grippingProperties': {
        'isGrippingFrame': False, 'compatibleGrippers': [], 'compatibleGripperTips': [],
    },
    'gluePtProperties': {
        'isGlueFrame': False, 'hasBeenPlaced': False, 'hasBeenCured': False,
        'timeMs': 0.0, 'dispenseOffsetMm': 0.0,
    },
    'glueLineProperties': {
        'isGlueLineFrame': False, 'gCodeFileName': '',
        'use_1K_dispenser': False, 'hasBeenPlaced': False,
    },
    'assemblyProperties': {
        'isAssemblyFrame': False, 'isTargetFrame': False,
        'associatedFrame': '', 'associatedComponent': '',
    },
}


def label(key):
    """Turn exported field names into readable form labels."""
    aliases = {
        'timeMs': 'Dispensing time (ms)', 'dispenseOffsetMm': 'Dispense offset (mm)',
        'gluePtProperties': 'Glue point', 'glueLineProperties': 'Glue line',
        'visionProperties': 'Vision', 'laserProperties': 'Laser',
        'grippingProperties': 'Gripping', 'assemblyProperties': 'Assembly / target',
        'inPlane': 'In-plane', 'assemblyFrame': 'Moving assembly frame',
        'targetFrame': 'Stationary target frame', 'frameProperties': 'Frame role overrides',
        'move_component_1': 'Move component 1', 'moveComponent_1': 'Move component 1',
        'glueLinePrerequisite': 'Glue line prerequisites',
    }
    readable = re.sub(r'(?<=[a-z])(?=[A-Z])', ' ', key).replace('_', ' ').capitalize()
    return aliases.get(key, readable)


def defaults_for(key, value):
    """Supply controls for supported fields even when an optional field is absent."""
    schemas = {
        'metadata': {'name': '', 'description': '', 'documentUnits': 'mm',
                     'cadPath': '', 'cadPathCollision': ''},
        'spawning': {'spawningOrigin': '', 'spawningTransformation': IDENTITY_TRANSFORM},
        'ref_frames': {'name': '', 'type': 'frame', 'transformation': IDENTITY_TRANSFORM,
                       'constraints': {}, 'properties': {}},
        'ref_axes': {'name': '', 'refPointNames': ['', '']},
        'ref_planes': {'name': '', 'refPointNames': ['', '', ''], 'refAxisNames': [],
                       'normalVector': {'X': 0.0, 'Y': 0.0, 'Z': 1.0}},
        'components': {'name': '', 'guid': '', 'transformation': IDENTITY_TRANSFORM,
                       'frameProperties': {}},
        'constraints': CONSTRAINTS, 'properties': PROPERTIES,
        'prerequisites': {'gluePrerequisites': [], 'glueLinePrerequisite': [],
                          'componentPrerequisites': []},
        'description': {f'planeMatch_{index}': PLANE_MATCH for index in range(1, 4)},
        'assemblyFrame': FRAME_REFERENCE, 'targetFrame': FRAME_REFERENCE,
        'frame_override': PROPERTIES,
    }
    if key == 'assemblyConstraints':
        if value.get('type', 'PlaneMatch') == 'FrameMatch':
            return {'name': '', 'assemblyFrame': FRAME_REFERENCE,
                    'targetFrame': FRAME_REFERENCE, 'prerequisites': {}}
        moving_key = ('moveComponent_1' if 'moveComponent_1' in value
                      else 'move_component_1')
        return {'name': '', 'component_1': '', 'component_2': '', moving_key: False,
                'description': {}, 'prerequisites': {}}
    if key in ('transformation', 'transform', 'spawningTransformation'):
        return IDENTITY_TRANSFORM
    if key in CONSTRAINTS:
        return CONSTRAINTS[key]
    if key in PROPERTIES:
        result = deepcopy(PROPERTIES[key])
        # Existing snake-case aliases must stay editable without adding a second
        # camel-case field that would override them on the next import.
        for exported in list(result):
            snake = re.sub(r'(?<=[a-z0-9])(?=[A-Z])', '_', exported).lower()
            if snake in value:
                del result[exported]
        if 'isGluePoint' in value or 'is_glue_point' in value:
            result.pop('isGlueFrame', None)
        return result
    if key.startswith('planeMatch_'):
        return PLANE_MATCH
    return schemas.get(key, {})


class NoWheelDoubleSpinBox(QDoubleSpinBox):
    """Let the surrounding form scroll without changing the number."""

    def wheelEvent(self, event):
        event.ignore()


class NoWheelComboBox(QComboBox):
    """Keep the selected value stable while scrolling the form."""

    def wheelEvent(self, event):
        event.ignore()


class ValueControl(QWidget):
    """A checkbox, numeric input or text/choice input with a typed value."""

    changed = pyqtSignal()

    def __init__(self, key, value, choices=None, unit=None):
        super().__init__()
        self.original = deepcopy(value)
        self.modified = False
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        if isinstance(value, bool):
            self.control = QCheckBox()
            self.control.setChecked(value)
            signal = self.control.toggled
            self.read = self.control.isChecked
        elif isinstance(value, (float, int)):
            self.control = NoWheelDoubleSpinBox()
            self.control.setRange(-1e12, 1e12)
            self.control.setDecimals(9)
            self.control.setSingleStep(0.000001)
            self.control.setKeyboardTracking(False)
            self.control.setValue(float(value))
            if unit:
                self.set_unit(unit)
            signal = self.control.valueChanged
            self.read = self.control.value
        elif choices:
            self.control = NoWheelComboBox()
            self.control.setEditable(True)
            self.control.addItems(list(dict.fromkeys(['', *choices])))
            self.control.setCurrentText(str(value))
            signal = self.control.currentTextChanged
            self.read = self.control.currentText
        else:
            self.control = QLineEdit('' if value is None else str(value))
            signal = self.control.textChanged
            self.read = self.control.text
        self.control.setMinimumHeight(34)
        if not isinstance(self.control, QCheckBox):
            self.control.setMinimumWidth(180)
            self.control.setSizePolicy(
                QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        layout.addWidget(self.control, 1)
        signal.connect(self._changed)
        if key in ('cadPath', 'cadPathCollision', 'gCodeFileName', 'g_code_file_name'):
            browse = QPushButton('Browse…')
            browse.clicked.connect(self._browse)
            layout.addWidget(browse)

    def _changed(self, *args):
        self.modified = True
        self.changed.emit()

    def set_unit(self, unit):
        """Display the unit and set the corresponding useful precision."""
        decimals = LENGTH_DECIMALS.get(unit, 3 if unit in ('%', 'ms') else 9)
        self.control.setDecimals(decimals)
        self.control.setSingleStep(10 ** -decimals)
        self.control.setSuffix(f' {unit}')

    def _browse(self):
        path, _ = QFileDialog.getOpenFileName(self, 'Select file')
        if path:
            self.control.setText(path)

    def value(self):
        return self.read() if self.modified else deepcopy(self.original)


class ListControl(QWidget):
    """Editable rows for frame names, vectors and prerequisite references."""

    changed = pyqtSignal()

    def __init__(self, key, values, context):
        super().__init__()
        self.key = key
        self.context = context
        self.rows = []
        self.layout = QVBoxLayout(self)
        self.layout.setContentsMargins(0, 0, 0, 0)
        self.fixed = key == 'offsetValues'
        for value in values:
            self._add_row(value)
        if not self.fixed:
            self.add_button = QPushButton('Add ' + (
                'prerequisite' if 'Prerequisite' in key else 'entry'))
            self.add_button.clicked.connect(self.add_row)
            self.layout.addWidget(self.add_button)

    def _add_row(self, value):
        row = QWidget()
        layout = QHBoxLayout(row)
        layout.setContentsMargins(0, 0, 0, 0)
        if isinstance(value, dict):
            schema = (FRAME_REFERENCE if self.key != 'componentPrerequisites'
                      else {'component': ''})
            control = ObjectForm(value, '', self.context, schema=schema)
        else:
            choices = self.context.get('components') if self.key == 'componentPrerequisites' \
                else self.context.get('frames') if self.key in ('refFrameNames', 'refPointNames') \
                else self.context.get('axes') if self.key == 'refAxisNames' else None
            unit = self.context.get('unit', 'mm') if self.key == 'offsetValues' else None
            control = ValueControl(self.key, value, choices, unit=unit)
        control.changed.connect(self.changed.emit)
        if self.fixed:
            layout.addWidget(QLabel('XYZ'[len(self.rows)]))
        layout.addWidget(control, 1)
        if not self.fixed:
            remove = QPushButton('Remove')
            remove.clicked.connect(lambda: self._remove(row))
            layout.addWidget(remove)
        index = self.layout.count() - (1 if hasattr(self, 'add_button') else 0)
        self.layout.insertWidget(index, row)
        self.rows.append((row, control))

    def add_row(self):
        if self.key in ('gluePrerequisites', 'glueLinePrerequisite', 'glueLinePrerequisites'):
            value = deepcopy(FRAME_REFERENCE)
        elif self.key == 'componentPrerequisites':
            value = {'component': ''}
        else:
            value = ''
        self._add_row(value)
        self.changed.emit()

    def _remove(self, row):
        self.rows = [(widget, control) for widget, control in self.rows if widget is not row]
        self.layout.removeWidget(row)
        row.deleteLater()
        self.changed.emit()

    def value(self):
        return [control.value() for _, control in self.rows]


class ObjectForm(QWidget):
    """Grouped typed controls which preserve omitted and unknown fields."""

    changed = pyqtSignal()

    def __init__(self, data, kind, context=None, schema=None):
        super().__init__()
        self.original = deepcopy(data)
        self.kind = kind
        self.context = context or {}
        self.controls = {}
        self.touched = set()
        self.groups = {}
        self.form = QFormLayout(self)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        self.form.setFieldGrowthPolicy(QFormLayout.FieldGrowthPolicy.AllNonFixedFieldsGrow)
        self.form.setRowWrapPolicy(QFormLayout.RowWrapPolicy.WrapLongRows)
        self.form.setFormAlignment(Qt.AlignmentFlag.AlignTop)
        self.form.setLabelAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
        self.form.setContentsMargins(12, 12, 12, 12)
        self.form.setHorizontalSpacing(20)
        self.form.setVerticalSpacing(12)
        defaults = deepcopy(defaults_for(kind, data) if schema is None else schema)
        values = {**defaults, **data}
        # Legacy alignAxes has no effect on frame matching; preserve it on disk
        # but do not offer a misleading alignment switch.
        hidden = {'alignAxes', 'IsIdentity'}
        for key, value in values.items():
            if key in hidden:
                continue
            if isinstance(value, dict):
                control = ObjectForm(
                    value, 'frame_override' if kind == 'frameProperties' else key, self.context)
                group = QGroupBox(label(key))
                group_layout = QVBoxLayout(group)
                group_layout.addWidget(control)
                if kind in ('constraints', 'properties', 'frame_override'):
                    group.setCheckable(True)
                    group.setChecked(key in data)
                    control.setVisible(key in data)
                    group.toggled.connect(control.setVisible)
                    group.toggled.connect(lambda checked, field=key: self._touch(field))
                    self.groups[key] = group
                self.form.addRow(group)
            elif isinstance(value, list):
                control = ListControl(key, value, self.context)
                group = QGroupBox(label(key))
                QVBoxLayout(group).addWidget(control)
                self.form.addRow(group)
            else:
                unit = self._unit_for(key, values)
                control = ValueControl(key, value, self._choices(key), unit=unit)
                control.setToolTip(label(key))
                if key == 'type' and kind != 'ref_frames':
                    control.setEnabled(False)
                self.form.addRow(label(key), control)
            self.controls[key] = control
            control.changed.connect(lambda field=key: self._touch(field))

        if kind == 'frameProperties':
            add = QPushButton('Add frame override…')
            add.clicked.connect(self._add_override)
            self.form.addRow(add)
        if kind == 'frameProperties':
            for key in data:
                remove = QPushButton(f'Remove override: {key}')
                remove.clicked.connect(lambda checked, field=key: self._remove_override(field))
                self.form.addRow(remove)

        if kind in ('properties', 'frame_override'):
            role_keys = {
                'visionProperties': ('isVisionFrame', 'is_vision_frame'),
                'laserProperties': ('isLaserFrame', 'is_laser_frame'),
                'grippingProperties': ('isGrippingFrame', 'is_gripping_frame'),
                'gluePtProperties': ('isGlueFrame', 'isGluePoint', 'is_glue_point'),
                'glueLineProperties': ('isGlueLineFrame', 'is_glue_line_frame'),
            }
            for key, candidates in role_keys.items():
                if key not in self.groups:
                    continue
                child = self.controls[key]
                role = next((child.controls[field] for field in candidates
                             if field in child.controls), None)
                if role:
                    self.groups[key].toggled.connect(role.control.setChecked)

        if kind == 'assemblyProperties':
            assembly_key = ('is_assembly_frame' if 'is_assembly_frame' in data
                            else 'isAssemblyFrame')
            target_key = 'is_target_frame' if 'is_target_frame' in data else 'isTargetFrame'
            assembly = self.controls[assembly_key].control
            target = self.controls[target_key].control
            assembly.toggled.connect(lambda checked: target.setChecked(False) if checked else None)
            target.toggled.connect(lambda checked: assembly.setChecked(False) if checked else None)

        if kind == 'rotation' and all(axis in self.controls for axis in 'XYZW'):
            reset = QPushButton('Reset to identity rotation')
            reset.clicked.connect(self._identity_rotation)
            self.form.addRow(reset)

        if kind == 'orthogonal':
            self.controls['unit_distance_from_f1'].control.currentTextChanged.connect(
                lambda unit: self.controls['distance_from_f1'].set_unit(
                    '%' if unit == '%' else self.context.get('unit', 'mm')))

    def _unit_for(self, key, values):
        unit = self.context.get('unit', 'mm')
        if self.kind == 'translation' and key in 'XYZ':
            return unit
        if key in ('planeOffset', 'plane_offset', 'distance_from_f1_f2_connection'):
            return unit
        if key == 'distance_from_f1':
            return '%' if values.get('unit_distance_from_f1', '%') == '%' else unit
        if key in ('dispenseOffsetMm', 'dispense_offset_mm'):
            return 'mm'
        if key in ('timeMs', 'time_ms'):
            return 'ms'
        return None

    def _identity_rotation(self):
        for axis in 'XYZW':
            self.controls[axis].control.setValue(1.0 if axis == 'W' else 0.0)

    def _touch(self, key):
        self.touched.add(key)
        self.changed.emit()

    def _choices(self, key):
        choices = {
            'documentUnits': ['mm', 'cm', 'um', 'm'],
            'normalAxis': ['x', 'y', 'z'],
            'frame_normal_plane_axis': ['x', 'y', 'z', '-x', '-y', '-z'],
            'frame_orthogonal_connection_axis': ['x', 'y', 'z'],
            'unit_distance_from_f1': ['%', 'mm', 'cm', 'um', 'm'],
            'dim': ['xyz', 'xy', 'xz', 'yz', 'x', 'y', 'z'],
        }
        if key == 'type' and self.kind == 'ref_frames':
            return ['frame', 'point']
        if key in ('component', 'component_1', 'component_2', 'associatedComponent',
                   'associated_component'):
            return self.context.get('components', [])
        if key in ('frame', 'refFrame', 'frame_1', 'frame_2', 'frame_3',
                   'associatedFrame', 'associated_frame', 'spawningOrigin'):
            return self.context.get('frames', [])
        if key in ('planeNameComponent_1', 'planeNameComponent_2'):
            return self.context.get('planes', [])
        return choices.get(key)

    def _add_override(self):
        name, accepted = QInputDialog.getItem(
            self, 'Frame override', 'Local frame name:', self.context.get('frames', []),
            0, True)
        if not accepted or not name.strip() or name.strip() in self.controls:
            return
        key = name.strip()
        control = ObjectForm({}, 'frame_override', self.context)
        group = QGroupBox(key)
        QVBoxLayout(group).addWidget(control)
        self.form.addRow(group)
        self.controls[key] = control
        control.changed.connect(lambda: self._touch(key))
        self._touch(key)

    def _remove_override(self, key):
        control = self.controls.pop(key, None)
        if control:
            control.parentWidget().hide()
            self._touch(key)

    def value(self):
        result = deepcopy(self.original)
        for key, control in self.controls.items():
            if key in self.groups and not self.groups[key].isChecked():
                result.pop(key, None)
            elif key in self.touched:
                result[key] = control.value()
        for key in self.touched - self.controls.keys():
            result.pop(key, None)
        # IsIdentity is an exporter hint and must agree with an edited quaternion.
        if self.kind == 'rotation' and self.touched:
            result['IsIdentity'] = all(result.get(axis, 0.0) == 0.0 for axis in 'XYZ') \
                and result.get('W', 1.0) == 1.0
        return result
