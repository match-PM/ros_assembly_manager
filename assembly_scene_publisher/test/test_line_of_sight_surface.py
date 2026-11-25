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


@pytest.fixture
def plate(tmp_path):
    """Plate 35 x 22 x 4 mm in metres, bottom face at z = 0."""
    source = vtk.vtkCubeSource()
    source.SetBounds(-0.035, 0.0, -0.022, 0.0, 0.0, 0.004)
    triangles = vtk.vtkTriangleFilter()
    triangles.SetInputConnection(source.GetOutputPort())
    path = tmp_path / 'plate.stl'
    writer = vtk.vtkSTLWriter()
    writer.SetFileName(str(path))
    writer.SetInputConnection(triangles.GetOutputPort())
    writer.Write()
    return str(path)


@pytest.mark.parametrize('height', [0.005, 0.010, 0.090])
def test_short_ray_through_a_thin_part_is_hidden(plate, height):
    # vtkOBBTree's default tolerance (0.01 in mesh units) dropped these hits.
    assert blocked(plate, (-0.017, -0.010, height), (-0.017, -0.010, 0.0))
    assert not blocked(plate, (-0.017, -0.010, height), (-0.017, -0.010, 0.004))


# Ray bundle for sensors that need the surroundings of a frame ---------------

def bundle(mesh, viewpoint, target, radius=0.0003, tilt=0.0087):
    checker = LineOfSightChecker(mesh, pose(*viewpoint), pose(*target))
    return checker.check_line_of_sight_bundle(feature_radius=radius, tilt_tolerance=tilt)


def tilted_view(target, tilt_rad, height=0.09):
    """Viewpoint above ``target``, the part tilted by ``tilt_rad`` about y
    (in component coordinates the view leans towards +x)."""
    import math
    return (target[0] + height * math.sin(tilt_rad), target[1], target[2] + height * math.cos(tilt_rad))


def test_bundle_without_radius_and_tilt_is_the_single_ray(plate):
    corner = (0.0, -0.011, 0.0)
    view = tilted_view(corner, 0.0349)
    occluded, blocked_rays, rays = bundle(plate, view, corner, radius=0.0, tilt=0.0)
    assert rays == 1 and occluded == blocked(plate, view, corner)


def test_bundle_rejects_a_frame_on_an_edge_seen_past_the_edge(plate):
    corner = (0.0, -0.011, 0.0)  # bottom edge of the plate
    view = tilted_view(corner, 0.0349)  # 2 deg: the single ray passes outside
    assert not blocked(plate, view, corner)
    occluded, blocked_rays, rays = bundle(plate, view, corner)
    assert occluded and rays == 45 and 'center' not in blocked_rays


def test_bundle_rejects_a_view_that_needs_the_exact_tilt(plate):
    corner = (0.0, -0.011, 0.0)
    view = tilted_view(corner, 0.0035)  # 0.2 deg
    assert not bundle(plate, view, corner, radius=0.0, tilt=0.0)[0]
    assert bundle(plate, view, corner, radius=0.0)[0]


@pytest.mark.parametrize('frame', [(-0.017, -0.011, 0.004), (0.0, -0.011, 0.004),
                                   (0.0, 0.0, 0.004)])
def test_bundle_keeps_top_face_frames_visible_also_at_edges(plate, frame):
    occluded, blocked_rays, _ = bundle(plate, tilted_view(frame, 0.0349), frame)
    assert not occluded, blocked_rays
