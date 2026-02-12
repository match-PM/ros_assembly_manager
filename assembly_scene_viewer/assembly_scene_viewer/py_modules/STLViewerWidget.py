from PyQt6.QtWidgets import QWidget, QVBoxLayout
import vtk
from vtkmodules.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
from vtkmodules.vtkRenderingOpenGL2 import vtkOpenGLRenderer

class STLViewerWidget(QWidget):

    REFERENCE_DOT_SIZE = 0.001  # Size of the reference dot for frames
    CURRENT_FRAME_DOT_SIZE = 0.0015  # Size of the dot for the currently selected frame
    CURRENT_FRAME_AXIS_SCALE = 0.03  # Scale for the axes of the currently selected frame
    CONSTRAINING_FRAME_AXIS_SCALE = 0.0  # Scale for axes of constraining frames (0 = no axes)
    def __init__(self):
        super().__init__()

        # VTK render window widget
        self.vtk_widget = QVTKRenderWindowInteractor(self)
        layout = QVBoxLayout()
        layout.addWidget(self.vtk_widget)
        self.setLayout(layout)

        # VTK Renderer
        self.renderer = vtk.vtkRenderer()
        self.vtk_widget.GetRenderWindow().AddRenderer(self.renderer)
        self.renderer.SetBackground(0.2, 0.2, 0.2)  # Dark background

        # Current STL actor
        self.stl_actor = None
        self.stl_file_path = None
        
        # Frame visualization actors
        self.frame_actors = {}  # Dictionary to store frame actors by frame name

        # Initialize interactor
        self.vtk_widget.Initialize()
        self.vtk_widget.Start()

    def load_stl(self, stl_file_path: str):
        """Load a new STL file into the viewer."""
        # Remove previous actor if exists
        if self.stl_actor:
            self.renderer.RemoveActor(self.stl_actor)
            self.stl_actor = None

        self.stl_file_path = stl_file_path
        # Load new STL
        reader = vtk.vtkSTLReader()
        reader.SetFileName(stl_file_path)
        reader.Update()

        # Mapper and actor
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(reader.GetOutputPort())

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)

        # Add to renderer
        self.renderer.AddActor(actor)
        self.stl_actor = actor

        # Reset camera to fit new STL
        self.renderer.ResetCamera()
        self.vtk_widget.GetRenderWindow().Render()

    def clear_stl(self):
        """Remove the currently loaded STL."""
        if self.stl_actor:
            self.renderer.RemoveActor(self.stl_actor)
            self.stl_actor = None
            self.vtk_widget.GetRenderWindow().Render()

    def reset_stl_file(self, stl_file_path: str):
        """Reset the STL file to a new path."""
        if stl_file_path != self.stl_file_path:
            self.stl_file_path = stl_file_path
            self.clear_stl()
            self.load_stl(stl_file_path)
    
    def display_frame(self, 
                      frame_name: str, 
                      position, 
                      orientation, 
                      scale: float = None, 
                      color_rgb: tuple = (1, 0, 0), 
                      add_dot: bool = True):
        """
        Display a coordinate frame (3D axes) at the specified position and orientation.
        
        Args:
            frame_name: Identifier for the frame
            position: geometry_msgs.msg.Point with x, y, z
            orientation: geometry_msgs.msg.Quaternion with x, y, z, w
            scale: Size of the axes visualization (0 = no axes). If None, uses CURRENT_FRAME_AXIS_SCALE
            color_rgb: Color for the axes (tuple of R, G, B). If (1,0,0) shows RGB axes, else uses custom colors
            add_dot: Whether to add a dot at the frame position
        """
        from scipy.spatial.transform import Rotation as R
        import numpy as np
        
        # Use default scale if not provided
        if scale is None:
            scale = self.CURRENT_FRAME_AXIS_SCALE
        
        # Remove existing frame if it exists
        if frame_name in self.frame_actors:
            for actor in self.frame_actors[frame_name]:
                self.renderer.RemoveActor(actor)
            del self.frame_actors[frame_name]
        
        # Convert quaternion to rotation matrix
        quat = np.array([orientation.x, orientation.y, orientation.z, orientation.w])
        rot = R.from_quat(quat)
        rot_matrix = rot.as_matrix()
        
        axes_actors = []
        
        # Create axes (X=red, Y=green, Z=blue) or use single color
        if color_rgb == (1, 0, 0):  # Red dot marker mode - show coordinate axes
            colors = [(1, 0, 0), (0, 1, 0), (0, 0, 1)]  # R, G, B
        else:  # Single color mode for constraining frames
            colors = [color_rgb, color_rgb, color_rgb]
        
        # Create axes if scale > 0
        if scale > 0:
            for axis_idx, color in enumerate(colors):
                # Create a line (axis)
                points = vtk.vtkPoints()
                points.InsertNextPoint(position.x, position.y, position.z)
                
                # Calculate end point of axis
                axis_direction = rot_matrix[:, axis_idx]
                end_point = np.array([position.x, position.y, position.z]) + axis_direction * scale
                points.InsertNextPoint(end_point[0], end_point[1], end_point[2])
                
                # Create line cells
                lines = vtk.vtkCellArray()
                line = vtk.vtkLine()
                line.GetPointIds().SetId(0, 0)
                line.GetPointIds().SetId(1, 1)
                lines.InsertNextCell(line)
                
                # Create polydata
                polydata = vtk.vtkPolyData()
                polydata.SetPoints(points)
                polydata.SetLines(lines)
                
                # Create mapper and actor
                mapper = vtk.vtkPolyDataMapper()
                mapper.SetInputData(polydata)
                
                actor = vtk.vtkActor()
                actor.SetMapper(mapper)
                actor.GetProperty().SetColor(color[0], color[1], color[2])
                actor.GetProperty().SetLineWidth(3)
                
                self.renderer.AddActor(actor)
                axes_actors.append(actor)
        
        # Add a dot at the frame position
        if add_dot:
            sphere_source = vtk.vtkSphereSource()
            sphere_source.SetCenter(position.x, position.y, position.z)
            dot_size = self.CURRENT_FRAME_DOT_SIZE if color_rgb == (1, 0, 0) else self.REFERENCE_DOT_SIZE
            sphere_source.SetRadius(dot_size)
            sphere_source.Update()
            
            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputConnection(sphere_source.GetOutputPort())
            
            actor = vtk.vtkActor()
            actor.SetMapper(mapper)
            actor.GetProperty().SetColor(color_rgb[0], color_rgb[1], color_rgb[2])
            
            self.renderer.AddActor(actor)
            axes_actors.append(actor)
        
        # Store actors for this frame
        self.frame_actors[frame_name] = axes_actors
        self.vtk_widget.GetRenderWindow().Render()
    
    def display_axis(self, axis_name: str, point_1, point_2, color_rgb: tuple = (1, 1, 0)):
        """
        Display an axis as a line connecting two points in 3D space.
        
        Args:
            axis_name: Identifier for the axis
            point_1: geometry_msgs.msg.Point with x, y, z (first point)
            point_2: geometry_msgs.msg.Point with x, y, z (second point)
            color_rgb: Color for the axis (tuple of R, G, B)
        """
        import numpy as np
        
        # Remove existing axis if it exists
        if axis_name in self.frame_actors:
            for actor in self.frame_actors[axis_name]:
                self.renderer.RemoveActor(actor)
            del self.frame_actors[axis_name]
        
        axes_actors = []
        
        # Create axis line connecting the two points
        points = vtk.vtkPoints()
        points.InsertNextPoint(point_1.x, point_1.y, point_1.z)
        points.InsertNextPoint(point_2.x, point_2.y, point_2.z)
        
        # Create line cells
        lines = vtk.vtkCellArray()
        line = vtk.vtkLine()
        line.GetPointIds().SetId(0, 0)
        line.GetPointIds().SetId(1, 1)
        lines.InsertNextCell(line)
        
        # Create polydata
        polydata = vtk.vtkPolyData()
        polydata.SetPoints(points)
        polydata.SetLines(lines)
        
        # Create mapper and actor
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(polydata)
        
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(color_rgb[0], color_rgb[1], color_rgb[2])
        actor.GetProperty().SetLineWidth(4)
        
        self.renderer.AddActor(actor)
        axes_actors.append(actor)
        
        # Add a dot at the first point
        sphere_source = vtk.vtkSphereSource()
        sphere_source.SetCenter(point_1.x, point_1.y, point_1.z)
        sphere_source.SetRadius(self.CURRENT_FRAME_DOT_SIZE)
        sphere_source.Update()
        
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(sphere_source.GetOutputPort())
        
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(color_rgb[0], color_rgb[1], color_rgb[2])
        
        self.renderer.AddActor(actor)
        axes_actors.append(actor)
        
        # Add a dot at the second point
        sphere_source = vtk.vtkSphereSource()
        sphere_source.SetCenter(point_2.x, point_2.y, point_2.z)
        sphere_source.SetRadius(self.CURRENT_FRAME_DOT_SIZE)
        sphere_source.Update()
        
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(sphere_source.GetOutputPort())
        
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(color_rgb[0], color_rgb[1], color_rgb[2])
        
        self.renderer.AddActor(actor)
        axes_actors.append(actor)
        
        # Store actors for this axis
        self.frame_actors[axis_name] = axes_actors
        self.vtk_widget.GetRenderWindow().Render()
    
    def display_plane(self, plane_name: str, 
                      plane_data: dict,
                      color_rgb: tuple = (0, 1, 1), 
                      alpha: float = 0.3):
        """
        Display a plane in 3D space.
        
        Args:
            plane_name: Identifier for the plane
            plane_data: Dictionary with two possible formats:
                - For axis + point plane: {"axis": [point1, point2], "points": [support_point]}
                - For 3-point plane: {"points": [point1, point2, point3]}
            color_rgb: Color for the plane (tuple of R, G, B)
            alpha: Transparency (0=transparent, 1=opaque)
        """
        import numpy as np
        
        # Remove existing plane if it exists
        if plane_name in self.frame_actors:
            for actor in self.frame_actors[plane_name]:
                self.renderer.RemoveActor(actor)
            del self.frame_actors[plane_name]
        
        axes_actors = []
        
        # Check if we have axis points (axis + point plane)
        if "axis" in plane_data and plane_data["axis"]:
            axis_points = plane_data["axis"]
            # Display the axis line
            if len(axis_points) >= 2:
                points = vtk.vtkPoints()
                points.InsertNextPoint(axis_points[0].x, axis_points[0].y, axis_points[0].z)
                points.InsertNextPoint(axis_points[1].x, axis_points[1].y, axis_points[1].z)
                
                # Create line cells
                lines = vtk.vtkCellArray()
                line = vtk.vtkLine()
                line.GetPointIds().SetId(0, 0)
                line.GetPointIds().SetId(1, 1)
                lines.InsertNextCell(line)
                
                # Create polydata
                polydata = vtk.vtkPolyData()
                polydata.SetPoints(points)
                polydata.SetLines(lines)
                
                # Create mapper and actor for axis line
                mapper = vtk.vtkPolyDataMapper()
                mapper.SetInputData(polydata)
                
                actor = vtk.vtkActor()
                actor.SetMapper(mapper)
                actor.GetProperty().SetColor(color_rgb[0], color_rgb[1], color_rgb[2])
                actor.GetProperty().SetLineWidth(2)
                
                self.renderer.AddActor(actor)
                axes_actors.append(actor)
                
                # Add dots at axis points
                for axis_pt in axis_points:
                    sphere_source = vtk.vtkSphereSource()
                    sphere_source.SetCenter(axis_pt.x, axis_pt.y, axis_pt.z)
                    sphere_source.SetRadius(self.CURRENT_FRAME_DOT_SIZE)
                    sphere_source.Update()
                    
                    mapper = vtk.vtkPolyDataMapper()
                    mapper.SetInputConnection(sphere_source.GetOutputPort())
                    
                    actor = vtk.vtkActor()
                    actor.SetMapper(mapper)
                    actor.GetProperty().SetColor(color_rgb[0], color_rgb[1], color_rgb[2])
                    
                    self.renderer.AddActor(actor)
                    axes_actors.append(actor)
            
            # Get all points for plane (axis points + support points)
            all_points = axis_points + plane_data.get("points", [])
        else:
            # Three-point plane
            all_points = plane_data.get("points", [])
        
        # Display plane if we have at least 3 points
        if len(all_points) >= 3:
            p1 = np.array([all_points[0].x, all_points[0].y, all_points[0].z])
            p2 = np.array([all_points[1].x, all_points[1].y, all_points[1].z])
            p3 = np.array([all_points[2].x, all_points[2].y, all_points[2].z])
            
            # For 3-point plane, use the points as corners directly
            if "axis" not in plane_data or not plane_data["axis"]:
                # Use the three points as the three corners of the plane
                # Create a fourth point to complete the parallelogram
                v1 = p2 - p1
                v2 = p3 - p1
                p4 = p2 + v2  # or p3 + v1, creates parallelogram
                
                corners = [p1, p2, p4, p3]
            else:
                # For axis + point plane, compute normal and create plane square
                v1 = p2 - p1  # First axis direction
                v2 = p3 - p1  # Direction from first axis point to support point
                normal_vec = np.cross(v1, v2)
                normal_norm = np.linalg.norm(normal_vec)
                if normal_norm > 0:
                    normal_vec = normal_vec / normal_norm
                
                # Create perpendicular vectors
                u = v1 / np.linalg.norm(v1)
                v = np.cross(normal_vec, u)
                v = v / np.linalg.norm(v)
                
                center = (p1 + p2 + p3) / 3
                scale = max(np.linalg.norm(p2 - p1), np.linalg.norm(p3 - p1)) * 0.6
                half_scale = scale / 2
                
                corners = [
                    center - u * half_scale - v * half_scale,
                    center + u * half_scale - v * half_scale,
                    center + u * half_scale + v * half_scale,
                    center - u * half_scale + v * half_scale,
                ]
            
            # Create plane quad
            points = vtk.vtkPoints()
            for corner in corners:
                points.InsertNextPoint(corner[0], corner[1], corner[2])
            
            quads = vtk.vtkCellArray()
            quad = vtk.vtkQuad()
            for i in range(4):
                quad.GetPointIds().SetId(i, i)
            quads.InsertNextCell(quad)
            
            polydata = vtk.vtkPolyData()
            polydata.SetPoints(points)
            polydata.SetPolys(quads)
            
            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputData(polydata)
            
            actor = vtk.vtkActor()
            actor.SetMapper(mapper)
            actor.GetProperty().SetColor(color_rgb[0], color_rgb[1], color_rgb[2])
            actor.GetProperty().SetOpacity(alpha)
            
            self.renderer.AddActor(actor)
            axes_actors.append(actor)
        
        # Add dots at all input points
        for point in all_points:
            sphere_source = vtk.vtkSphereSource()
            sphere_source.SetCenter(point.x, point.y, point.z)
            sphere_source.SetRadius(self.CURRENT_FRAME_DOT_SIZE)
            sphere_source.Update()
            
            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputConnection(sphere_source.GetOutputPort())
            
            actor = vtk.vtkActor()
            actor.SetMapper(mapper)
            actor.GetProperty().SetColor(color_rgb[0], color_rgb[1], color_rgb[2])
            
            self.renderer.AddActor(actor)
            axes_actors.append(actor)
        
        # Store actors for this plane
        self.frame_actors[plane_name] = axes_actors
        self.vtk_widget.GetRenderWindow().Render()
        
        self.renderer.AddActor(actor)
        axes_actors.append(actor)
        
        # Store actors for this plane
        self.frame_actors[plane_name] = axes_actors
        self.vtk_widget.GetRenderWindow().Render()
    
    def display_plane_normal(self, plane_name: str, center, normal_vec, scale: float = 0.05, color_rgb: tuple = (1, 0, 0)):
        """
        Display the normal vector of a plane as an arrow.
        
        Args:
            plane_name: Identifier for the plane
            center: geometry_msgs.msg.Point at the center of the plane
            normal_vec: geometry_msgs.msg.Vector3 or numpy array representing the normal
            scale: Length of the normal arrow
            color_rgb: Color for the arrow (tuple of R, G, B)
        """
        import numpy as np
        
        # Convert normal_vec to numpy array (handle both Vector3 and numpy array inputs)
        if isinstance(normal_vec, np.ndarray):
            normal = normal_vec.copy()
        else:
            normal = np.array([normal_vec.x, normal_vec.y, normal_vec.z])
        
        # Normalize the normal vector
        normal_norm = np.linalg.norm(normal)
        if normal_norm > 0:
            normal = normal / normal_norm
        else:
            return
        
        center_pt = np.array([center.x, center.y, center.z])
        end_pt = center_pt + normal * scale
        
        # Create line for the normal arrow
        points = vtk.vtkPoints()
        points.InsertNextPoint(center_pt[0], center_pt[1], center_pt[2])
        points.InsertNextPoint(end_pt[0], end_pt[1], end_pt[2])
        
        lines = vtk.vtkCellArray()
        line = vtk.vtkLine()
        line.GetPointIds().SetId(0, 0)
        line.GetPointIds().SetId(1, 1)
        lines.InsertNextCell(line)
        
        polydata = vtk.vtkPolyData()
        polydata.SetPoints(points)
        polydata.SetLines(lines)
        
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(polydata)
        
        line_actor = vtk.vtkActor()
        line_actor.SetMapper(mapper)
        line_actor.GetProperty().SetColor(color_rgb[0], color_rgb[1], color_rgb[2])
        line_actor.GetProperty().SetLineWidth(3)
        
        self.renderer.AddActor(line_actor)
        
        # Add a cone at the end to indicate direction
        cone_source = vtk.vtkConeSource()
        cone_source.SetCenter(end_pt[0], end_pt[1], end_pt[2])
        cone_source.SetHeight(scale * 0.3)
        cone_source.SetRadius(scale * 0.1)
        cone_source.SetDirection(normal[0], normal[1], normal[2])
        cone_source.Update()
        
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(cone_source.GetOutputPort())
        
        cone_actor = vtk.vtkActor()
        cone_actor.SetMapper(mapper)
        cone_actor.GetProperty().SetColor(color_rgb[0], color_rgb[1], color_rgb[2])
        
        self.renderer.AddActor(cone_actor)
        
        # Store both line and cone actors with a special key
        normal_key = f"{plane_name}_normal"
        if normal_key in self.frame_actors:
            for old_actor in self.frame_actors[normal_key]:
                self.renderer.RemoveActor(old_actor)
        
        self.frame_actors[normal_key] = [line_actor, cone_actor]
        self.vtk_widget.GetRenderWindow().Render()
    
    def clear_frame(self, frame_name: str):
        """Remove the visualization of a specific frame."""
        if frame_name in self.frame_actors:
            for actor in self.frame_actors[frame_name]:
                self.renderer.RemoveActor(actor)
            del self.frame_actors[frame_name]
            self.vtk_widget.GetRenderWindow().Render()
    
    def clear_all_frames(self):
        """Remove all frame visualizations."""
        for frame_name in list(self.frame_actors.keys()):
            self.clear_frame(frame_name)
    
    def display_constraining_frames(self, frame_names: list, positions: dict, orientations: dict):
        """
        Display constraining frames with orange dots (no axes).
        
        Args:
            frame_names: List of frame names to display
            positions: Dict mapping frame_name to position
            orientations: Dict mapping frame_name to orientation
        """
        for frame_name in frame_names:
            if frame_name in positions and frame_name in orientations:
                # Display with orange color, no axes, just a dot
                self.display_frame(
                    f"{frame_name}_constraining",
                    positions[frame_name],
                    orientations[frame_name],
                    scale=self.CONSTRAINING_FRAME_AXIS_SCALE,  # No axes
                    color_rgb=(1, 0.647, 0),  # Orange
                    add_dot=True
                )