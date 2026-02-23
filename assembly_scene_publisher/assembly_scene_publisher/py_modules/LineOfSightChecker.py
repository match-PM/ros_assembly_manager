import vtk
import threading
import time
import math
from typing import Tuple, Optional
from geometry_msgs.msg import Pose, Point
import numpy as np

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
    
    def check_line_of_sight(self) -> Tuple[vtk.vtkPoints, bool]:
        if self.mesh is None:
            raise RuntimeError("Mesh must be loaded before checking line of sight. Call load_mesh() first.")

        obbTree = vtk.vtkOBBTree()
        obbTree.SetDataSet(self.mesh)
        obbTree.BuildLocator()

        # Ray-mesh intersections
        collision_points = vtk.vtkPoints()
        obbTree.IntersectWithLine(self.viewpoint, self.target_point, collision_points, None)

        # Compute ray vector
        ray_vec = np.array(self.target_point) - np.array(self.viewpoint)
        ray_length = np.linalg.norm(ray_vec)
        epsilon = 1e-6

        # Filter intersections strictly between start and end
        valid_intersections = []
        for i in range(collision_points.GetNumberOfPoints()):
            p = np.array(collision_points.GetPoint(i))
            t = np.linalg.norm(p - np.array(self.viewpoint))
            if epsilon < t < ray_length - epsilon:
                valid_intersections.append(p)

        # Check if either point is inside the mesh
        viewpoint_inside = self._point_inside_mesh(self.viewpoint)
        target_inside = self._point_inside_mesh(self.target_point)

        # Decide occlusion rule
        self.is_occluded = len(valid_intersections) > 0 or viewpoint_inside or target_inside

        # Store filtered collision points
        self.collision_points = vtk.vtkPoints()
        for p in valid_intersections:
            self.collision_points.InsertNextPoint(p)

        if self.logger:
            #self.logger.info(f"Line of sight: {'Occluded' if self.is_occluded else 'Visible'}")
            #self.logger.info(f"  - Total intersections: {collision_points.GetNumberOfPoints()}")
            #self.logger.info(f"  - Valid intersections: {len(valid_intersections)}")
            if viewpoint_inside:
                self.logger.error("Viewpoint is inside the mesh!")
            if target_inside:
                self.logger.error("Target point is inside the mesh!")

        return self.collision_points, self.is_occluded
    
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
    
    def visualize(self, timeout_seconds: int = 10) -> None:
        """Create and display the 3D visualization in a separate thread.
        
        Displays:
        - Semi-transparent gray mesh
        - Blue sphere at viewpoint
        - Yellow sphere at target point
        - Cyan arrow from viewpoint to target
        - Red spheres at collision points (if any)
        - Small reference axes
        - Legend explaining colors
        
        Args:
            timeout_seconds: Seconds to display window before auto-closing (default 10)
        """
        def run_visualization() -> None:
            """Run the visualization in a thread."""
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
            
            # Add legend/text
            legend = vtk.vtkTextActor()
            legend.SetInput("Blue: Viewpoint  |  Yellow: Target  |  Red: Collisions  |  Cyan: Direction")
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
                renderWindow.Finalize()
                interactor.TerminateApp()
            
            interactor.Initialize()
            renderWindow.Render()
            
            # Set timer to close after timeout_seconds
            timer_id = interactor.CreateRepeatingTimer(timeout_seconds * 1000)
            interactor.AddObserver('TimerEvent', on_timer)
            
            interactor.Start()
        
        # Start visualization in separate daemon thread
        vis_thread = threading.Thread(target=run_visualization, daemon=True)
        vis_thread.start()