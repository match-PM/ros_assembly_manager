"""Edit assembly and component descriptions in one adaptive workspace."""

import sys
from copy import deepcopy

import rclpy
from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import (
    QApplication, QFileDialog, QHBoxLayout, QInputDialog, QLabel,
    QMainWindow, QMessageBox, QPushButton, QScrollArea, QSizePolicy, QSplitter, QToolBar,
    QTreeWidget, QTreeWidgetItem, QVBoxLayout, QWidget,
)
from rclpy.node import Node

from assembly_scene_viewer.py_modules.AssemblyDescriptionModifier import (
    AssemblyDescriptionModifier, IDENTITY_TRANSFORM,
)
from assembly_scene_viewer.py_modules.DescriptionForms import NoWheelComboBox, ObjectForm


class AssemblyModifierWidget(QWidget):
    """Show the relevant contents of either description without separate tabs."""

    def __init__(self):
        super().__init__()
        self.setAcceptDrops(True)
        self.model = None
        self._path = None
        self._dirty = False
        self._form_modified = False
        self.form = None
        self._excluded = {}
        self.setStyleSheet('''
            QLineEdit, QDoubleSpinBox, QComboBox { min-height: 30px; padding: 2px 6px; }
            QPushButton { min-height: 30px; padding: 3px 10px; }
            QGroupBox { margin-top: 14px; padding-top: 12px; font-weight: bold; }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; }
            QTreeWidget::item { min-height: 30px; }
        ''')
        layout = QVBoxLayout(self)
        self.path_label = QLabel('Open or drop an assembly or component JSON file')
        self.path_label.setTextInteractionFlags(
            Qt.TextInteractionFlag.TextSelectableByMouse)
        self.path_label.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Fixed)
        layout.addWidget(self.path_label)
        self.status_label = QLabel()
        self.status_label.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        layout.addWidget(self.status_label)
        self.workspace = QSplitter(Qt.Orientation.Horizontal)
        self.workspace.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        navigation = QWidget()
        nav_layout = QVBoxLayout(navigation)
        self.tree = QTreeWidget()
        self.tree.setHeaderLabel('Description contents')
        self.tree.currentItemChanged.connect(self._select)
        nav_layout.addWidget(self.tree, 1)
        self.add_type = NoWheelComboBox()
        nav_layout.addWidget(self.add_type)
        buttons = QHBoxLayout()
        for label, callback in (
                ('Add', self.add_item), ('Duplicate', self.duplicate_item),
                ('Remove', self.remove_item)):
            button = QPushButton(label)
            button.clicked.connect(callback)
            buttons.addWidget(button)
        nav_layout.addLayout(buttons)
        self.workspace.addWidget(navigation)
        details = QWidget()
        detail_layout = QVBoxLayout(details)
        self.detail_title = QLabel('Select an item')
        self.detail_title.setStyleSheet('font-size: 14pt; font-weight: bold; padding: 8px;')
        self.detail_title.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        detail_layout.addWidget(self.detail_title)
        self.unit_hint = QLabel()
        self.unit_hint.setWordWrap(True)
        self.unit_hint.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        detail_layout.addWidget(self.unit_hint)
        self.form_scroll = QScrollArea()
        self.form_scroll.setWidgetResizable(True)
        detail_layout.addWidget(self.form_scroll, 1)
        apply_button = QPushButton('Apply changes')
        apply_button.clicked.connect(self.apply_changes)
        detail_layout.addWidget(apply_button)
        self.workspace.addWidget(details)
        self.workspace.setStretchFactor(0, 1)
        self.workspace.setStretchFactor(1, 3)
        self.workspace.setSizes([300, 850])
        layout.addWidget(self.workspace, 1)
        self.workspace.setEnabled(False)

    def _edited(self):
        self._form_modified = True
        self.status_label.setText('Unsaved changes')
        self.window().setWindowTitle('Description Editor *')

    def set_dirty(self, dirty):
        self._dirty = dirty
        self.status_label.setText('Unsaved changes' if dirty else 'Saved')
        self.window().setWindowTitle(
            'Description Editor' + (' *' if dirty else ''))

    def _data(self):
        # Draft templates may be incomplete until their required fields are edited.
        return deepcopy(self.model._data)

    @staticmethod
    def _at(data, path):
        for key in path:
            data = data[key]
        return data

    def _refresh(self, selected_path=()):
        data = self._data()
        mounting = data['mountingDescription']
        self.tree.blockSignals(True)
        self.tree.clear()
        selected = None

        def entry(parent, label, path):
            nonlocal selected
            item = QTreeWidgetItem(parent, [label])
            item.setData(0, Qt.ItemDataRole.UserRole, path)
            if path == selected_path:
                selected = item
            return item

        entry(self.tree, 'Document settings', ())
        if data['type'] == 'Assembly':
            for key, label in (
                    ('components', 'Component instances'),
                    ('assemblyConstraints', 'Assembly instructions')):
                path = ('mountingDescription', key)
                group = entry(self.tree, label, path)
                for index, value in enumerate(mounting.get(key, [])):
                    suffix = (f" [{value.get('type', 'PlaneMatch')}]"
                              if key == 'assemblyConstraints' else '')
                    entry(group, value.get('name', '<unnamed>') + suffix,
                          path + (index,))
        references_path = ('mountingDescription', 'mountingReferences')
        references = entry(self.tree, 'Spawning and reference geometry', references_path)
        for key, label in (('ref_frames', 'Reference frames'),
                           ('ref_axes', 'Axes'), ('ref_planes', 'Planes')):
            path = references_path + (key,)
            group = entry(references, label, path)
            for index, value in enumerate(mounting.get('mountingReferences', {}).get(key, [])):
                entry(group, value.get('name', '<unnamed>'), path + (index,))
        self.tree.expandAll()
        self.tree.setCurrentItem(selected or self.tree.topLevelItem(0))
        self.tree.blockSignals(False)
        self._show_item(self.tree.currentItem())

    def _show_item(self, item):
        self._path = item.data(0, Qt.ItemDataRole.UserRole)
        data = self._data()
        try:
            value = self._at(data, self._path)
        except KeyError:
            value = {} if self._path[-1] == 'mountingReferences' else []
        self.detail_title.setText(item.text(0))
        units = self.model.metadata.get('documentUnits', 'mm')
        self.unit_hint.setText(
            f'Distances in {units}; rotations as quaternion X, Y, Z, W. '
            'Distance precision: 10 nm. Enable only the constraints and properties you need.')
        self._excluded = {}
        self.form = None
        if not self._path:
            self._excluded = {'mountingDescription': value['mountingDescription']}
            value = {key: item for key, item in value.items() if key not in self._excluded}
            kind = 'metadata'
        elif self._path[-1] == 'mountingReferences':
            self._excluded = {key: item for key, item in value.items()
                              if key in ('ref_frames', 'ref_axes', 'ref_planes')}
            value = {key: item for key, item in value.items() if key not in self._excluded}
            kind = 'spawning'
        else:
            kind = self._path[-2] if isinstance(self._path[-1], int) else self._path[-1]
        if isinstance(value, dict):
            self.form = ObjectForm(value, kind, self._form_context())
            self.form.changed.connect(self._edited)
            self.form_scroll.setWidget(self.form)
        else:
            hint = QLabel('Select an item on the left to edit it.\n'
                          'Use Add to create a frame, axis, plane, component or instruction.')
            hint.setWordWrap(True)
            self.form_scroll.setWidget(hint)
        self._form_modified = False

    def _form_context(self):
        data = self._data()
        references = data['mountingDescription'].get('mountingReferences', {})
        context = {
            'frames': [item['name'] for item in references.get('ref_frames', [])],
            'axes': [item['name'] for item in references.get('ref_axes', [])],
            'planes': [item['name'] for item in references.get('ref_planes', [])],
            'components': [item['name'] for item in
                           data['mountingDescription'].get('components', [])],
        }
        if data['type'] == 'Assembly' and self.model.file_path:
            # Resolve exported component instance names using the same database
            # directory convention as the publisher's description importer.
            import re
            import json
            directory = self.model.file_path.parent.parent / 'components'
            for name in context['components']:
                path = directory / (re.sub(r'-\d+$', '', name) + '.json')
                try:
                    with path.open(encoding='utf-8') as stream:
                        component = json.load(stream)
                    refs = component['mountingDescription']['mountingReferences']
                    for target, source in (('frames', 'ref_frames'), ('axes', 'ref_axes'),
                                           ('planes', 'ref_planes')):
                        context[target].extend(item['name'] for item in refs.get(source, []))
                except (OSError, ValueError, KeyError, TypeError):
                    continue
        context = {key: list(dict.fromkeys(values)) for key, values in context.items()}
        context['unit'] = data.get('documentUnits', 'mm')
        return context

    def _select(self, item, previous):
        if item is None:
            return
        if not self._apply_pending():
            self.tree.blockSignals(True)
            self.tree.setCurrentItem(previous)
            self.tree.blockSignals(False)
            return
        self._show_item(item)

    def _apply_pending(self):
        if not self._form_modified:
            return True
        return self.apply_changes(refresh=False)

    def apply_changes(self, checked=False, refresh=True):
        if not self.model or self._path is None:
            return False
        try:
            if self.form is None:
                return True
            value = {**self.form.value(), **self._excluded}
            candidate = self._data()
            if not self._path:
                candidate = value
            else:
                parent = candidate
                for key in self._path[:-1]:
                    if isinstance(key, str):
                        parent = parent.setdefault(key, {})
                    else:
                        parent = parent[key]
                parent[self._path[-1]] = value
            if candidate.get('type') != self.model.metadata['type']:
                raise ValueError('Document type cannot be changed while editing.')
            self.model.replace_document(candidate)
            self._form_modified = False
            self.set_dirty(True)
            if refresh:
                self._refresh(self._path)
            return True
        except (ValueError, TypeError, KeyError, AttributeError) as error:
            QMessageBox.warning(self, 'Cannot apply changes', str(error))
            return False

    def _confirm_replace(self):
        if not self._dirty and not self._form_modified:
            return True
        answer = QMessageBox.question(
            self, 'Unsaved changes', 'Save changes before continuing?',
            QMessageBox.StandardButton.Save | QMessageBox.StandardButton.Discard |
            QMessageBox.StandardButton.Cancel)
        if answer == QMessageBox.StandardButton.Save:
            return bool(self.save_document())
        return answer == QMessageBox.StandardButton.Discard

    def new_document(self):
        if not self._confirm_replace():
            return
        kind, accepted = QInputDialog.getItem(
            self, 'New description', 'File type:', ['Component', 'Assembly'], 0, False)
        if not accepted:
            return
        name, accepted = QInputDialog.getText(
            self, 'New description', 'Name:', text=f'New {kind}')
        if accepted and name.strip():
            self._load_model(AssemblyDescriptionModifier.create_new(name.strip(), kind))
            self.set_dirty(True)

    def open_document(self):
        path, _ = QFileDialog.getOpenFileName(
            self, 'Open assembly or component description', '', 'JSON files (*.json)')
        if path:
            self.load_json(path)

    def load_json(self, file_path):
        if not self._confirm_replace():
            return
        try:
            self._load_model(AssemblyDescriptionModifier.from_file(file_path))
        except (ValueError, OSError) as error:
            QMessageBox.critical(self, 'Cannot open description', str(error))

    def _load_model(self, model):
        self.model = model
        self._path = None
        self._form_modified = False
        self.workspace.setEnabled(True)
        self.path_label.setText(
            f"{model.metadata['type']} — {model.file_path or 'New unsaved description'}")
        self.add_type.clear()
        self.add_type.addItems(['Reference frame', 'Axis', 'Plane'])
        if model.metadata['type'] == 'Assembly':
            self.add_type.addItems(['Component instance', 'PlaneMatch', 'FrameMatch'])
        self._refresh()
        self.set_dirty(False)

    def add_item(self):
        if not self._apply_pending():
            return
        kind = self.add_type.currentText()
        if kind == 'Component instance':
            index = self.model.add_component()
            path = ('mountingDescription', 'components', index)
        elif kind in ('PlaneMatch', 'FrameMatch'):
            index = self.model.add_instruction(kind)
            path = ('mountingDescription', 'assemblyConstraints', index)
        else:
            key = {'Reference frame': 'ref_frames', 'Axis': 'ref_axes',
                   'Plane': 'ref_planes'}[kind]
            items = self.model.mounting_references.setdefault(key, [])
            name, accepted = QInputDialog.getText(self, f'Add {kind}', 'Name:')
            if not accepted or not name.strip():
                return
            if any(item.get('name') == name.strip() for item in items):
                QMessageBox.warning(self, 'Duplicate name', 'This name already exists.')
                return
            value = {'name': name.strip()}
            if key == 'ref_frames':
                value.update(type='frame', transformation=deepcopy(IDENTITY_TRANSFORM),
                             constraints={}, properties={})
            elif key == 'ref_axes':
                value['refPointNames'] = ['', '']
            else:
                value.update(refPointNames=['', '', ''], refAxisNames=[],
                             normalVector={'X': 0.0, 'Y': 0.0, 'Z': 1.0})
            items.append(value)
            path = ('mountingDescription', 'mountingReferences', key, len(items) - 1)
        self.set_dirty(True)
        self._refresh(path)

    def duplicate_item(self):
        if not self._path or not isinstance(self._path[-1], int):
            return
        if not self._apply_pending():
            return
        data = self._data()
        items = self._at(data, self._path[:-1])
        value = deepcopy(items[self._path[-1]])
        names = {item.get('name') for item in items}
        base = value.get('name', 'item') + '_copy'
        name = base
        index = 2
        while name in names:
            name = f'{base}_{index}'
            index += 1
        value['name'] = name
        items.append(value)
        self.model._data = data
        self.set_dirty(True)
        self._refresh(self._path[:-1] + (len(items) - 1,))

    def remove_item(self):
        if not self._path or not isinstance(self._path[-1], int):
            return
        answer = QMessageBox.question(
            self, 'Remove item', 'Remove the selected item?',
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)
        if answer != QMessageBox.StandardButton.Yes:
            return
        data = self._data()
        del self._at(data, self._path[:-1])[self._path[-1]]
        self.model._data = data
        self.set_dirty(True)
        self._refresh(self._path[:-1])

    def save_document(self):
        if not self.model or not self._apply_pending():
            return False
        if not self.model.file_path:
            return self.save_document_as()
        return self._save()

    def save_document_as(self):
        if not self.model or not self._apply_pending():
            return False
        path, _ = QFileDialog.getSaveFileName(
            self, 'Save description', f"{self.model.metadata.get('name', 'description')}.json",
            'JSON files (*.json)')
        return self._save(path) if path else False

    def _save(self, path=None):
        try:
            saved = self.model.save(path)
            self.path_label.setText(f"{self.model.metadata['type']} — {saved}")
            self.set_dirty(False)
            return True
        except (ValueError, OSError) as error:
            QMessageBox.critical(self, 'Cannot save description', str(error))
            return False

    def dragEnterEvent(self, event):
        if any(url.toLocalFile().lower().endswith('.json')
               for url in event.mimeData().urls()):
            event.acceptProposedAction()

    def dropEvent(self, event):
        for url in event.mimeData().urls():
            if url.toLocalFile().lower().endswith('.json'):
                self.load_json(url.toLocalFile())
                event.acceptProposedAction()
                return


class AssemblyModifierMainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Description Editor')
        self.resize(1250, 800)
        self.editor = AssemblyModifierWidget()
        self.setCentralWidget(self.editor)
        toolbar = QToolBar('File')
        toolbar.setMovable(False)
        self.addToolBar(toolbar)
        for label, callback in (
                ('New', self.editor.new_document), ('Open', self.editor.open_document),
                ('Save', self.editor.save_document), ('Save as', self.editor.save_document_as)):
            toolbar.addAction(label).triggered.connect(callback)

    def closeEvent(self, event):
        if self.editor._confirm_replace():
            event.accept()
        else:
            event.ignore()


class AssemblyJsonModifierNode(Node):
    def __init__(self):
        super().__init__('assembly_json_modifier_node')
        self.qt_window = AssemblyModifierMainWindow()


def main(args=None):
    rclpy.init(args=args)
    app = QApplication.instance() or QApplication(sys.argv)
    node = AssemblyJsonModifierNode()
    node.qt_window.show()
    try:
        app.exec()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
