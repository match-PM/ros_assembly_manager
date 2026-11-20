"""Frames lie on the part surface: their visibility must not depend on which
way the face points."""

from geometry_msgs.msg import Pose
import pytest
import vtk

from assembly_scene_publisher.py_modules.LineOfSightChecker import LineOfSightChecker


@pytest.fixture
def cube(tmp_path):
    """Unit cube centred at the origin (faces at +-0.5)."""
    source = vtk.vtkCubeSource()
    triangles = vtk.vtkTriangleFilter()
    triangles.SetInputConnection(source.GetOutputPort())
    path = tmp_path / 'cube.stl'
    writer = vtk.vtkSTLWriter()
    writer.SetFileName(str(path))
    writer.SetInputConnection(triangles.GetOutputPort())
    writer.Write()
    return str(path)


def pose(x, y, z):
    result = Pose()
    result.position.x, result.position.y, result.position.z = float(x), float(y), float(z)
    return result


def blocked(mesh, viewpoint, target):
    return LineOfSightChecker(mesh, pose(*viewpoint), pose(*target)).check_line_of_sight()[1]


FACES = [(axis, sign) for axis in range(3) for sign in (1, -1)]


def on_face(axis, sign, distance):
    """Point on the face (distance 0.5) or outside it along the face normal."""
    point = [0.1, -0.2, 0.15]  # off the triangle diagonals
    point[axis] = sign * distance
    return point


@pytest.mark.parametrize('axis, sign', FACES)
def test_frame_on_any_face_is_visible_from_outside(cube, axis, sign):
    assert not blocked(cube, on_face(axis, sign, 2.0), on_face(axis, sign, 0.5))


@pytest.mark.parametrize('axis, sign', FACES)
def test_frame_on_any_face_is_hidden_behind_the_part(cube, axis, sign):
    assert blocked(cube, on_face(axis, -sign, 2.0), on_face(axis, sign, 0.5))


def test_target_inside_the_part_is_hidden(cube):
    assert blocked(cube, (0.1, -0.2, 2.0), (0.1, -0.2, 0.0))


def test_viewpoint_inside_the_part_is_hidden(cube):
    assert blocked(cube, (0.1, -0.2, 0.0), (0.1, -0.2, -0.5))
