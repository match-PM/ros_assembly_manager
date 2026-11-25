import vtk
import math
from typing import Tuple, Optional
from geometry_msgs.msg import Pose, Point
import numpy as np
from assembly_scene_publisher.py_modules.LineOfSightVisualizer import LineOfSightVisualizer


_visualizer = LineOfSightVisualizer()

class LineOfSightChecker:
    """Class for checking line of sight between viewpoint and target using STL mesh.
    
    This class provides functionality to:
    - Load STL mesh files
    - Check if line of sight between two points is occluded by mesh geometry
    - Visualize the scene with collision points, viewpoint, target, and direction vector
    - Run checks independently from visualization
    
    The class stores results internally for access after execution.
    """
    
    def __init__(self, mesh_filepath: str, 
                 viewpoint: Pose, 
                 target_point: Pose,
                 logger=None) -> None:
        """Initialize the line of sight checker.
        
        Args:
            mesh_filepath: Path to STL file to load
            viewpoint: ROS2 Pose object for viewpoint (uses position.x, position.y, position.z)
            target_point: ROS2 Pose object for target point (uses position.x, position.y, position.z)
            logger: ROS2 logger instance for logging messages
        
        Raises:
            ValueError: If viewpoint or target_point are not Pose objects
        """
        if not isinstance(viewpoint, Pose) or not isinstance(target_point, Pose):
            raise ValueError("viewpoint and target_point must be geometry_msgs.msg.Pose objects")
        
        self.logger = logger
        self.mesh_filepath: str = mesh_filepath
        self.viewpoint_pose: Pose = viewpoint
        self.target_point_pose: Pose = target_point
        
        # Extract coordinates from poses for internal use
        self.viewpoint: Tuple[float, float, float] = (
            viewpoint.position.x,
            viewpoint.position.y,
            viewpoint.position.z
        )
        self.target_point: Tuple[float, float, float] = (
            target_point.position.x,
            target_point.position.y,
            target_point.position.z
        )
        
        self.mesh: Optional[vtk.vtkPolyData] = None
        self.collision_points: Optional[vtk.vtkPoints] = None
        self.is_occluded: bool = False
        self.load_mesh()
    
    def load_mesh(self) -> vtk.vtkPolyData:
        """Load STL mesh from file.
        
        Returns:
            vtk.vtkPolyData: The loaded mesh
            
        Raises:
            RuntimeError: If mesh file cannot be read
        """
        reader = vtk.vtkSTLReader()
        reader.SetFileName(self.mesh_filepath)
        reader.Update()
        
        if reader.GetOutput().GetNumberOfCells() == 0:
            raise RuntimeError(f"Failed to load mesh from {self.mesh_filepath}")
        
        self.mesh = reader.GetOutput()
        return self.mesh
    
    def _point_inside_mesh(self, point: Tuple[float, float, float]) -> bool:
        select = vtk.vtkSelectEnclosedPoints()
        select.SetTolerance(1e-6)  # small tolerance for surface points
        polydata = vtk.vtkPolyData()
        pts = vtk.vtkPoints()
        pts.InsertNextPoint(point)
        polydata.SetPoints(pts)
        select.SetInputData(polydata)
        select.SetSurfaceData(self.mesh)
        select.Update()
        return bool(select.IsInside(0))
    
    def _obb_tree(self) -> vtk.vtkOBBTree:
        if self.mesh is None:
            raise RuntimeError("Mesh must be loaded before checking line of sight. Call load_mesh() first.")
        obbTree = vtk.vtkOBBTree()
        obbTree.SetDataSet(self.mesh)
        # The default tolerance (0.01, in mesh units: 10 mm for meshes in
        # metres) drops intersections of short rays, e.g. a ray from 10 mm
        # above through a 4 mm plate finds no hit.
        obbTree.SetTolerance(1e-6)
        obbTree.BuildLocator()
        return obbTree

    def _ray(self, obbTree, viewpoint, target) -> Tuple[list, bool]:
        """Intersections strictly between ``viewpoint`` and ``target`` and
        whether the viewpoint lies inside the mesh."""
        collision_points = vtk.vtkPoints()
        obbTree.IntersectWithLine(tuple(viewpoint), tuple(target), collision_points, None)
        ray_length = np.linalg.norm(np.array(target) - np.array(viewpoint))
        epsilon = 1e-6
        valid_intersections = []
        for i in range(collision_points.GetNumberOfPoints()):
            p = np.array(collision_points.GetPoint(i))
            t = np.linalg.norm(p - np.array(viewpoint))
            if epsilon < t < ray_length - epsilon:
                valid_intersections.append(p)
        # Only the viewpoint is tested for lying inside the mesh. The target is a
        # frame on the part surface, where the inside test is undecided (points on
        # downward-facing faces count as inside). A target buried in the part is
        # still detected: the ray from an outside viewpoint must cross the surface.
        return valid_intersections, self._point_inside_mesh(tuple(viewpoint))

    def _store(self, intersections, occluded):
        self.is_occluded = occluded
        self.collision_points = vtk.vtkPoints()
        for p in intersections:
            self.collision_points.InsertNextPoint(p)

    def check_line_of_sight(self) -> Tuple[vtk.vtkPoints, bool]:
        """One ray from the viewpoint to the target."""
        intersections, viewpoint_inside = self._ray(self._obb_tree(), self.viewpoint,
                                                    self.target_point)
        self._store(intersections, len(intersections) > 0 or viewpoint_inside)
        if self.logger and viewpoint_inside:
            self.logger.error("Viewpoint is inside the mesh!")
        return self.collision_points, self.is_occluded

    def check_line_of_sight_bundle(self, feature_radius: float = 0.0,
                                   tilt_tolerance: float = 0.0,
                                   ring_rays: int = 8) -> Tuple[bool, list, int]:
        """Rays for a sensor that needs the surroundings of the target, with
        the part's orientation known only roughly.

        Besides the centre ray, parallel rays onto a ring of ``feature_radius``
        around the target (perpendicular to the ray) are checked, each at the
        given direction and tilted by ``tilt_tolerance`` [rad] about two axes
        perpendicular to the ray. Ring rays end one radius before the ring:
        the ring is perpendicular to the ray, not to the face, so on a face
        tilted against it (up to 45 deg) its end would lie inside the part.
        The target is visible only if every ray is clear; a target on an edge
        of the part, seen past that edge, is therefore not. With both values
        0 this is :meth:`check_line_of_sight`.

        Returns ``(occluded, labels of the blocked rays, number of rays)``;
        ``collision_points`` holds the intersections of all rays.
        """
        from scipy.spatial.transform import Rotation

        target = np.array(self.target_point, dtype=float)
        ray = np.array(self.viewpoint, dtype=float) - target
        length = float(np.linalg.norm(ray))

        def perpendicular(direction):
            helper = np.array([1.0, 0.0, 0.0]) if abs(direction[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
            u = np.cross(direction, helper)
            u /= np.linalg.norm(u)
            return u, np.cross(direction, u)

        rays = []  # (label, viewpoint, target)
        if length > 0:
            u, v = perpendicular(ray / length)
            directions = [('', ray)]
            if tilt_tolerance:
                directions += [(f'tilt {name}{"+" if sign > 0 else "-"}',
                                Rotation.from_rotvec(sign * tilt_tolerance * axis).apply(ray))
                               for name, axis in (('u', u), ('v', v)) for sign in (1, -1)]
            for label, direction in directions:
                unit = direction / np.linalg.norm(direction)
                du, dv = perpendicular(unit)
                shifts = [('center', np.zeros(3))]
                if feature_radius:
                    shifts += [(f'ring {k}', feature_radius * (np.cos(angle) * du + np.sin(angle) * dv))
                               for k, angle in enumerate(np.linspace(0, 2 * np.pi, ring_rays,
                                                                     endpoint=False))]
                for name, shift in shifts:
                    end = target + shift + (unit * feature_radius if name != 'center' else 0.0)
                    rays.append((' '.join(filter(None, (name, label))), end + direction, end))
        else:
            rays.append(('center', np.array(self.viewpoint, dtype=float), target))

        obbTree = self._obb_tree()
        blocked, intersections = [], []
        for label, start, end in rays:
            hits, inside = self._ray(obbTree, start, end)
            intersections += hits
            if hits or inside:
                blocked.append(label)
        self._store(intersections, bool(blocked))
        return self.is_occluded, blocked, len(rays)

    def create_mesh_actor(self) -> vtk.vtkActor:
        """Create actor for the mesh.
        
        Returns:
            vtk.vtkActor: Actor with semi-transparent gray mesh
        """
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(self.mesh)
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetOpacity(0.6)
        actor.GetProperty().SetColor(0.8, 0.8, 0.8)
        return actor
    
    def create_collision_points_actor(self) -> Optional[vtk.vtkActor]:
        """Create actor for collision points using red spheres.
        
        Returns:
            Optional[vtk.vtkActor]: Actor with collision sphere glyphs, or None if no collisions
        """
        if self.collision_points.GetNumberOfPoints() == 0:
            return None
        
        # Create sphere glyph for each collision point
        sphereSource = vtk.vtkSphereSource()
        sphereSource.SetRadius(0.003)
        sphereSource.SetPhiResolution(16)
        sphereSource.SetThetaResolution(16)
        sphereSource.Update()
        
        # Use glyph3D to place spheres at collision points
        glyphFilter = vtk.vtkGlyph3D()
        glyphFilter.SetSourceData(sphereSource.GetOutput())
        
        collisionGeometry = vtk.vtkPolyData()
        collisionGeometry.SetPoints(self.collision_points)
        
        glyphFilter.SetInputData(collisionGeometry)
        glyphFilter.Update()
        
        collisionMapper = vtk.vtkPolyDataMapper()
        collisionMapper.SetInputData(glyphFilter.GetOutput())
        collisionActor = vtk.vtkActor()
        collisionActor.SetMapper(collisionMapper)
        collisionActor.GetProperty().SetColor(1.0, 0.0, 0.0)  # Red
        return collisionActor
    
    def create_viewpoint_actor(self) -> vtk.vtkActor:
        """Create actor for viewpoint marker (Blue sphere).
        
        Returns:
            vtk.vtkActor: Actor with blue point marker
        """
        pointMarker = vtk.vtkPoints()
        pointMarker.InsertNextPoint(self.viewpoint)
        
        markerPolyData = vtk.vtkPolyData()
        markerPolyData.SetPoints(pointMarker)
        
        glyphFilter = vtk.vtkVertexGlyphFilter()
        glyphFilter.SetInputData(markerPolyData)
        glyphFilter.Update()
        
        markerMapper = vtk.vtkPolyDataMapper()
        markerMapper.SetInputData(glyphFilter.GetOutput())
        markerActor = vtk.vtkActor()
        markerActor.SetMapper(markerMapper)
        markerActor.GetProperty().SetColor(0.0, 0.0, 1.0)  # Blue
        markerActor.GetProperty().SetPointSize(15)
        return markerActor
    
    def create_target_actor(self) -> vtk.vtkActor:
        """Create actor for target marker (Yellow sphere).
        
        Returns:
            vtk.vtkActor: Actor with yellow point marker
        """
        pointMarker = vtk.vtkPoints()
        pointMarker.InsertNextPoint(self.target_point)
        
        markerPolyData = vtk.vtkPolyData()
        markerPolyData.SetPoints(pointMarker)
        
        glyphFilter = vtk.vtkVertexGlyphFilter()
        glyphFilter.SetInputData(markerPolyData)
        glyphFilter.Update()
        
        markerMapper = vtk.vtkPolyDataMapper()
        markerMapper.SetInputData(glyphFilter.GetOutput())
        markerActor = vtk.vtkActor()
        markerActor.SetMapper(markerMapper)
        markerActor.GetProperty().SetColor(1.0, 1.0, 0.0)  # Yellow
        markerActor.GetProperty().SetPointSize(15)
        return markerActor
    
    def create_arrow_actor(self) -> Optional[vtk.vtkActor]:
        """Create arrow vector showing direction from viewpoint to target.
        
        The arrow starts at the viewpoint and points toward the target point.
        
        Returns:
            Optional[vtk.vtkActor]: Actor with cyan arrow, or None if viewpoint equals target
        """
        # Calculate vector from viewpoint to target
        vector = [
            self.target_point[0] - self.viewpoint[0],
            self.target_point[1] - self.viewpoint[1],
            self.target_point[2] - self.viewpoint[2]
        ]
        
        length = math.sqrt(vector[0]**2 + vector[1]**2 + vector[2]**2)
        
        if length < 1e-6:
            return None  # No valid arrow for zero-length vector
        
        # Normalize vector
        normalized = [vector[0]/length, vector[1]/length, vector[2]/length]
        
        # Create arrow source (points along +X by default)
        arrowSource = vtk.vtkArrowSource()
        arrowSource.SetShaftRadius(0.005)
        arrowSource.SetTipRadius(0.02)
        arrowSource.SetTipLength(0.15)
        arrowSource.Update()
        
        # Create transform
        transform = vtk.vtkTransform()
        
        # Translate to viewpoint FIRST
        transform.Translate(self.viewpoint[0], self.viewpoint[1], self.viewpoint[2])
        
        # Then rotate to align with target direction
        # Arrow source points along X-axis (1, 0, 0)
        default_dir = [1, 0, 0]
        
        # Calculate rotation using cross product and dot product
        cross = [
            default_dir[1]*normalized[2] - default_dir[2]*normalized[1],
            default_dir[2]*normalized[0] - default_dir[0]*normalized[2],
            default_dir[0]*normalized[1] - default_dir[1]*normalized[0]
        ]
        
        cross_len = math.sqrt(cross[0]**2 + cross[1]**2 + cross[2]**2)
        dot = default_dir[0]*normalized[0] + default_dir[1]*normalized[1] + default_dir[2]*normalized[2]
        
        if cross_len > 1e-6:
            angle = math.atan2(cross_len, dot) * 180.0 / math.pi
            axis = [cross[0]/cross_len, cross[1]/cross_len, cross[2]/cross_len]
            transform.RotateWXYZ(angle, axis[0], axis[1], axis[2])
        elif dot < 0:
            # Vector is opposite to default direction
            transform.RotateWXYZ(180, 0, 1, 0)
        
        # Scale to vector length
        transform.Scale(length, length, length)
        
        # Apply transform
        transformFilter = vtk.vtkTransformPolyDataFilter()
        transformFilter.SetTransform(transform)
        transformFilter.SetInputData(arrowSource.GetOutput())
        transformFilter.Update()
        
        # Create mapper and actor
        arrowMapper = vtk.vtkPolyDataMapper()
        arrowMapper.SetInputData(transformFilter.GetOutput())
        arrowActor = vtk.vtkActor()
        arrowActor.SetMapper(arrowMapper)
        arrowActor.GetProperty().SetColor(0.0, 1.0, 1.0)  # Cyan
        arrowActor.GetProperty().SetOpacity(0.8)
        return arrowActor
    
    def visualize(self, timeout_seconds: int = 10) -> bool:
        """Open an isolated window without blocking the caller.

        Return False if another LOS window is already open. Only plain data is
        passed to a fresh interpreter; VTK objects never cross process boundaries.
        The window closes automatically after timeout_seconds.
        """
        if timeout_seconds <= 0:
            raise ValueError("timeout_seconds must be positive")
        points = self.collision_points
        return _visualizer.show({
            'mesh_filepath': self.mesh_filepath,
            'viewpoint': self.viewpoint,
            'target_point': self.target_point,
            'collision_points': [points.GetPoint(i) for i in range(
                points.GetNumberOfPoints())] if points is not None else [],
            'timeout_seconds': timeout_seconds,
        }, logger=self.logger)

    def _render(self, timeout_seconds: int) -> None:
        """Render on the visualizer process's main thread."""
        # Create renderer
        renderer = vtk.vtkRenderer()
        renderer.SetBackground(0.1, 0.1, 0.1)

        # Add actors
        renderer.AddActor(self.create_mesh_actor())
        arrow_actor = self.create_arrow_actor()
        if arrow_actor:
            renderer.AddActor(arrow_actor)
        renderer.AddActor(self.create_viewpoint_actor())
        renderer.AddActor(self.create_target_actor())

        collision_actor = self.create_collision_points_actor()
        if collision_actor:
            renderer.AddActor(collision_actor)

        # Add axes (small reference frame)
        axes = vtk.vtkAxesActor()
        axes.SetAxisLabels(0)  # Disable labels to reduce visual clutter
        axes.GetXAxisShaftProperty().SetColor(1, 0, 0)
        axes.GetYAxisShaftProperty().SetColor(0, 1, 0)
        axes.GetZAxisShaftProperty().SetColor(0, 0, 1)

        # Use actor position instead of scale for better control
        axes.SetPosition(0, 0, 0)

        # Scale down using actor's property
        axes.SetUserTransform(vtk.vtkTransform())
        axes.GetUserTransform().Scale(0.2, 0.2, 0.2)
        renderer.AddActor(axes)

        # Avoid vertical bars, which VTK can interpret as a math-text table.
        legend = vtk.vtkTextActor()
        legend.SetInput("Blue: Viewpoint    Yellow: Target    Red: Collisions    Cyan: Direction")
        legend.GetTextProperty().SetFontSize(12)
        legend.GetTextProperty().SetColor(1.0, 1.0, 1.0)
        legend.SetDisplayPosition(10, 20)
        renderer.AddActor2D(legend)

        # Reset camera to fit all actors
        renderer.ResetCamera()
        camera = renderer.GetActiveCamera()
        camera.Zoom(1.2)

        # Create window and interactor
        renderWindow = vtk.vtkRenderWindow()
        renderWindow.AddRenderer(renderer)
        renderWindow.SetSize(1000, 800)
        renderWindow.SetWindowName("Line of Sight Visualization")

        interactor = vtk.vtkRenderWindowInteractor()
        interactor.SetRenderWindow(renderWindow)

        # Set trackball camera style for responsive interaction
        style = vtk.vtkInteractorStyleTrackballCamera()
        interactor.SetInteractorStyle(style)

        # Create timer callback to close window after timeout
        def on_timer(obj, event) -> None:
            interactor.TerminateApp()

        interactor.Initialize()
        renderWindow.Render()

        # Use a one-shot timer and release the window after the event loop exits.
        interactor.AddObserver('TimerEvent', on_timer)
        timer_id = interactor.CreateOneShotTimer(int(timeout_seconds * 1000))
        try:
            interactor.Start()
        finally:
            interactor.DestroyTimer(timer_id)
            renderWindow.Finalize()
            interactor.SetRenderWindow(None)
