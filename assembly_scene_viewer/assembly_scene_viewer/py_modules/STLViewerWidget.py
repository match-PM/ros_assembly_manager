from PyQt6.QtWidgets import QWidget, QVBoxLayout
import vtk
from vtkmodules.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
from vtkmodules.vtkRenderingOpenGL2 import vtkOpenGLRenderer

class STLViewerWidget(QWidget):
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