"""LOS rendering is bounded and native crashes stay in the child process."""

import importlib
import json
import os
import signal
import subprocess
import sys
import threading
from unittest.mock import Mock

from geometry_msgs.msg import Pose
import pytest
import vtk

from assembly_scene_publisher.py_modules.LineOfSightChecker import LineOfSightChecker
from assembly_scene_publisher.py_modules.LineOfSightVisualizer import LineOfSightVisualizer


@pytest.fixture
def checker(tmp_path):
    cube = vtk.vtkCubeSource()
    cube.Update()
    mesh_path = tmp_path / 'cube.stl'
    writer = vtk.vtkSTLWriter()
    writer.SetFileName(str(mesh_path))
    writer.SetInputData(cube.GetOutput())
    writer.Write()
    viewpoint, target = Pose(), Pose()
    viewpoint.position.x = 2.0
    target.position.x = 3.0
    return LineOfSightChecker(str(mesh_path), viewpoint, target)


def test_geometry_and_visualization_snapshot_are_independent(checker, monkeypatch):
    assert checker.check_line_of_sight()[1] is False
    module = importlib.import_module(
        'assembly_scene_publisher.py_modules.LineOfSightChecker')
    visualizer = Mock()
    monkeypatch.setattr(module, '_visualizer', visualizer)
    checker.visualize(timeout_seconds=2)
    snapshot = visualizer.show.call_args.args[0]
    assert json.loads(json.dumps(snapshot)) == {
        'mesh_filepath': checker.mesh_filepath,
        'viewpoint': [2.0, 0.0, 0.0],
        'target_point': [3.0, 0.0, 0.0],
        'collision_points': [],
        'timeout_seconds': 2,
    }
    checker.target_point = (0.0, 0.0, 0.0)
    assert checker.check_line_of_sight()[1] is True


@pytest.mark.parametrize('crash', [False, True])
def test_child_is_bounded_reaped_and_can_restart(monkeypatch, tmp_path, crash):
    module = importlib.import_module(
        'assembly_scene_publisher.py_modules.LineOfSightVisualizer')
    real_popen = subprocess.Popen
    ready_path = tmp_path / 'ready.json'
    # Replace just the rendering entry point with a real, controllable child.
    script = (
        'import json, os, resource, signal, sys, time\n'
        'from pathlib import Path\n'
        'resource.setrlimit(resource.RLIMIT_CORE, (0, 0))\n'
        'data = json.load(sys.stdin)\n'
        f'Path({str(ready_path)!r}).write_text(json.dumps(data))\n'
        'time.sleep(30)\n'
    )

    def launch(command, **kwargs):
        assert command == [
            sys.executable, '-m',
            'assembly_scene_publisher.py_modules.LineOfSightVisualizer']
        return real_popen([sys.executable, '-c', script], **kwargs)

    monkeypatch.setattr(module.subprocess, 'Popen', launch)
    visualizer = LineOfSightVisualizer()
    # Signal completion of the pipe I/O and process reaper thread.
    finished = threading.Event()
    original_wait = visualizer._wait

    def wait(*args):
        try:
            original_wait(*args)
        finally:
            finished.set()

    monkeypatch.setattr(visualizer, '_wait', wait)
    logger = Mock()
    try:
        assert visualizer.show({'test': 'snapshot'}, logger)
        process = visualizer._process
        assert process.pid != os.getpid()
        assert not visualizer.show({'test': 'second'}, logger)
        # Wait for the child to consume the snapshot before terminating it.
        for _ in range(100):
            if ready_path.exists():
                break
            threading.Event().wait(0.02)
        assert json.loads(ready_path.read_text()) == {'test': 'snapshot'}
        if crash:
            os.kill(process.pid, signal.SIGSEGV)
        else:
            visualizer.close()
        assert finished.wait(5)
        assert visualizer._process is None
        if crash:
            assert process.returncode == -signal.SIGSEGV
            logger.warning.assert_called_once()
        assert visualizer.show({'test': 'restart'}, logger)
    finally:
        visualizer.close()


@pytest.mark.parametrize('occluded', [False, True])
def test_visualization_failure_does_not_change_service_result(monkeypatch, occluded):
    module = importlib.import_module(
        'assembly_scene_publisher.py_modules.LineOfSightManager')
    manager = module.LineOfSightManager.__new__(module.LineOfSightManager)
    manager.node = Mock()
    manager.logger = Mock()
    manager.internal_scene_analyzer = Mock()
    manager.internal_scene_analyzer.get_ref_frame_by_name.return_value.pose = Pose()
    monkeypatch.setattr(module, 'multiply_ros_transforms', Mock(return_value=Pose()))
    checker = Mock()
    checker.check_line_of_sight.return_value = (None, occluded)
    checker.visualize.side_effect = OSError('could not start child')
    monkeypatch.setattr(module, 'LineOfSightChecker', Mock(return_value=checker))
    response = manager.check_line_of_sight(Mock(frame_name='test'))
    assert response.success == (not occluded)
    manager.logger.warning.assert_called_once()
