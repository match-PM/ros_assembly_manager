"""Exercise ROS-to-Qt delivery without a ROS graph or OpenGL context."""

import threading
from unittest.mock import Mock

from PyQt6.QtWidgets import QApplication, QWidget

import assembly_manager_interfaces.msg as messages
import assembly_scene_viewer.py_modules.AssemblySceneViewer as viewer_module


def test_scene_callback_updates_panels_only_on_gui_thread(monkeypatch):
    app = QApplication.instance() or QApplication([])
    # Keep the actual scene panels; only replace the unrelated OpenGL viewers.
    monkeypatch.setattr(viewer_module, 'STLViewerWidget', QWidget)
    node = Mock()
    widget = viewer_module.AssemblyScenceViewerWidget(node)
    callback = node.create_subscription.call_args.args[2]
    gui_thread = threading.get_ident()
    updates = []
    for panel in (widget.objects_panel, widget.instructions_panel,
                  widget.scene_overview_panel):
        original_update = panel.update_scene

        def record_update(msg, original_update=original_update):
            updates.append((threading.get_ident(), msg))
            original_update(msg)

        monkeypatch.setattr(panel, 'update_scene', record_update)

    scenes = [messages.ObjectScene() for _ in range(25)]
    worker = threading.Thread(target=lambda: [callback(msg) for msg in scenes])
    worker.start()
    worker.join(timeout=5)
    assert not worker.is_alive()
    # The worker must return without touching even a hidden viewer's panels.
    assert updates == []
    app.processEvents()
    assert len(updates) == 3 * len(scenes)
    assert all(thread_id == gui_thread for thread_id, _ in updates)
    assert widget._assembly_scene is scenes[-1]
    widget.close()
