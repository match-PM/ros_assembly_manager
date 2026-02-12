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