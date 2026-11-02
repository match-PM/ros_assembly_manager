"""Run the optional VTK window in a fresh interpreter, outside the ROS process."""

import atexit
import json
import subprocess
import sys
import threading


class LineOfSightVisualizer:
    """Allow one non-blocking visualization at a time and reap its process."""

    def __init__(self):
        self._lock = threading.Lock()
        self._process = None
        atexit.register(self.close)

    def show(self, snapshot, logger=None):
        """Start a window, or skip this snapshot while a window is already open."""
        payload = json.dumps(snapshot)
        with self._lock:
            if self._process is not None and self._process.poll() is None:
                return False
            process = subprocess.Popen(
                [sys.executable, '-m',
                 'assembly_scene_publisher.py_modules.LineOfSightVisualizer'],
                stdin=subprocess.PIPE,
                text=True,
                start_new_session=True,
            )
            self._process = process
            # Only pipe I/O and process waiting happen on this thread. All VTK
            # objects and the event loop belong to the child's main thread.
            threading.Thread(
                target=self._wait, args=(process, payload, logger), daemon=True,
            ).start()
        return True

    def _wait(self, process, payload, logger):
        try:
            process.communicate(payload)
            if process.returncode and logger is not None:
                logger.warning(
                    'Line of sight visualizer exited with code '
                    f'{process.returncode}; the LOS result is unaffected.')
        finally:
            with self._lock:
                if self._process is process:
                    self._process = None

    def close(self):
        """Stop any remaining window when the publisher exits."""
        with self._lock:
            process = self._process
            if process is not None and process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()


def main():
    """Rebuild VTK data from plain coordinates and render on the main thread."""
    snapshot = json.load(sys.stdin)

    # Keep VTK and ROS message imports out of the process controller above.
    import vtk
    from geometry_msgs.msg import Pose
    from assembly_scene_publisher.py_modules.LineOfSightChecker import (
        LineOfSightChecker,
    )

    def pose_at(coordinates):
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = coordinates
        return pose

    checker = LineOfSightChecker(
        snapshot['mesh_filepath'],
        pose_at(snapshot['viewpoint']),
        pose_at(snapshot['target_point']),
    )
    checker.collision_points = vtk.vtkPoints()
    for point in snapshot['collision_points']:
        checker.collision_points.InsertNextPoint(point)
    checker._render(snapshot['timeout_seconds'])


if __name__ == '__main__':
    main()
