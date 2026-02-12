from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QThread, pyqtSlot, QRunnable, QObject, QThreadPool
from PyQt6.QtWidgets import (QScrollArea, 
                             QMessageBox, 
                             QDialog, 
                             QHBoxLayout, 
                             QDialog, 
                             QInputDialog, 
                             QTreeWidget, 
                             QTreeWidgetItem, 
                             QApplication, 
                             QGridLayout, 
                             QFrame, 
                             QMainWindow, 
                             QListWidget, 
                             QListWidgetItem, 
                             QDoubleSpinBox, 
                             QWidget, 
                             QVBoxLayout, 
                             QPushButton, 
                             QCheckBox, 
                             QLineEdit, 
                             QComboBox, 
                             QTextEdit,
                             QLabel,
                             QSlider, 
                             QSpinBox, 
                             QFontDialog, 
                             QFileDialog,
                             QTableWidgetItem,
                            QTableWidget,
                            QGroupBox,
                            )

from scipy.spatial.transform import Rotation as R
from PyQt6.QtGui import QColor, QTextCursor, QFont, QAction
from functools import partial
from rclpy.node import Node

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import LookupException

from geometry_msgs.msg import TransformStamped, Transform
import assembly_manager_interfaces.msg as am_msgs
import rclpy
import yaml
from assembly_scene_publisher.py_modules.AssemblySceneAnalyzerAdv import AssemblySceneAnalyzerAdv
from assembly_scene_publisher.py_modules.AssemblySceneAnalyzer import UnInitializedScene
from assembly_scene_viewer.py_modules.STLViewerWidget import STLViewerWidget
# import vtk
# from vtk.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor

class AssemblySceneViewerMainWindow(QMainWindow):
    def __init__(self, ros_node:Node):
        super().__init__()
        self.ros_node = ros_node
        self.setWindowTitle("Assembly Scene Viewer")
        self.setGeometry(100, 100, 800, 600)
        self._main_widget = AssemblyScenceViewerWidget(ros_node)
        self.setCentralWidget(self._main_widget)

# class STLViewerWidget(QWidget):
#     def __init__(self, stl_file_path: str):
#         super().__init__()

#         # VTK render window widget
#         self.vtk_widget = QVTKRenderWindowInteractor(self)
#         layout = QVBoxLayout()
#         layout.addWidget(self.vtk_widget)
#         self.setLayout(layout)

#         # VTK Renderer
#         self.renderer = vtk.vtkRenderer()
#         self.vtk_widget.GetRenderWindow().AddRenderer(self.renderer)

#         # Load STL file
#         reader = vtk.vtkSTLReader()
#         reader.SetFileName(stl_file_path)
#         reader.Update()

#         # Mapper
#         mapper = vtk.vtkPolyDataMapper()
#         mapper.SetInputConnection(reader.GetOutputPort())

#         # Actor
#         actor = vtk.vtkActor()
#         actor.SetMapper(mapper)

#         # Add actor to renderer
#         self.renderer.AddActor(actor)
#         self.renderer.SetBackground(0.2, 0.2, 0.2)  # Dark background

#         # Initialize interactor
#         self.vtk_widget.Initialize()
#         self.vtk_widget.Start()


# ---------------------------------------------------------------------------
# Widget: A clean header + QListWidget panel for objects
# ---------------------------------------------------------------------------
class ObjectsListPanel(QWidget):
    def __init__(self, logger=None):
        super().__init__()
        self.logger = logger
        self.scene_message = am_msgs.ObjectScene()

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        # Header
        self.header_label = QLabel("Objects in Scene")
        self.header_label.setStyleSheet("""
            font-size: 14pt;
            font-weight: bold;
            padding: 4px;
        """)

        # List widget
        self.list_widget = QListWidget()

        layout.addWidget(self.header_label)
        layout.addWidget(self.list_widget)

    def log(self, msg):
        if self.logger:
            self.logger.warn(str(msg))

    def update_scene(self, msg: am_msgs.ObjectScene):
        self.scene_message = msg
        self.populate()

    def populate(self):
        self.list_widget.clear()
        for obj in self.scene_message.objects_in_scene:
            obj: am_msgs.Object
            item = QListWidgetItem(f"{obj.obj_name}")
            self.list_widget.addItem(item)

# ---------------------------------------------------------------------------
# Widget: Clean header + QListWidget for instructions
# ---------------------------------------------------------------------------
class InstructionListPanel(QWidget):
    def __init__(self, logger=None):
        super().__init__()
        self.logger = logger
        self.scene_message = am_msgs.ObjectScene()

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        # Header
        self.header_label = QLabel("Assembly Instructions")
        self.header_label.setStyleSheet("""
            font-size: 14pt;
            font-weight: bold;
            padding: 4px;
        """)

        # List widget
        self.list_widget = QListWidget()

        layout.addWidget(self.header_label)
        layout.addWidget(self.list_widget)

    def log(self, msg):
        if self.logger:
            self.logger.warn(str(msg))

    def update_scene(self, msg: am_msgs.ObjectScene):
        self.scene_message = msg
        self.populate()

    def populate(self):
        self.list_widget.clear()
        for ins in self.scene_message.assembly_instructions:
            ins: am_msgs.AssemblyInstruction
            item = QListWidgetItem(f"{ins.id}")
            self.list_widget.addItem(item)

class ObjectsElementPanel(QWidget):
    def __init__(self, obj: am_msgs.Object, 
                 logger=None):
        super().__init__()
        self.logger = logger
        self.obj = obj

        # ---- Main Vertical Layout ----
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.setLayout(layout)

        # ---- Object Label ----
        objects_label = QLabel(f"Object: {obj.obj_name}")
        layout.addWidget(objects_label)

        # ---- FRAMES (Tallest) ----
        frames_container = QVBoxLayout()
        frames_label = QLabel("Frames")
        frames_container.addWidget(frames_label)

        self.frames_panel = FramesElementPanel(obj, 
                                               logger=self.logger)
        frames_container.addWidget(self.frames_panel)

        frames_widget = QWidget()
        frames_widget.setLayout(frames_container)
        layout.addWidget(frames_widget)

        # ---- AXIS (Medium size) ----
        axis_container = QVBoxLayout()
        axis_label = QLabel("Axis")
        axis_container.addWidget(axis_label)

        self.axis_panel = AxisElementPanel(obj, 
                                           logger=self.logger)
        axis_container.addWidget(self.axis_panel)

        axis_widget = QWidget()
        axis_widget.setLayout(axis_container)
        layout.addWidget(axis_widget)

        # ---- PLANES (Smallest) ----
        planes_container = QVBoxLayout()
        planes_label = QLabel("Planes")
        planes_container.addWidget(planes_label)

        self.planes_panel = PlanesElementPanel(obj, 
                                             logger=self.logger)
        planes_container.addWidget(self.planes_panel)

        planes_widget = QWidget()
        planes_widget.setLayout(planes_container)
        layout.addWidget(planes_widget)

        # ---- Set Stretch Ratios ----
        # Higher number = bigger vertical space
        layout.setStretchFactor(frames_widget, 3)   # Largest
        layout.setStretchFactor(axis_widget,   1)   # Medium
        layout.setStretchFactor(planes_widget, 1)   # Smallest


class FramesElementPanel(QListWidget):
    def __init__(self, obj_message:am_msgs.Object, 
                 logger=None):
        super().__init__()
        self.logger = logger
        self.obj_message = obj_message   
        self.obj_name = obj_message.obj_name     
        self.populate()

    def populate(self):
        self.clear()
        for frame in self.obj_message.ref_frames:
            frame: am_msgs.RefFrame
            # strip the obj_name prefix if present
            #
            item_text = frame.frame_name
            if item_text.startswith(self.obj_name):
                item_text = item_text[len(self.obj_name)+1:]
            item = QListWidgetItem(f"{item_text}")
            self.addItem(item)
 

class AxisElementPanel(QListWidget):
    def __init__(self, obj_message:am_msgs.Object, logger=None):
        super().__init__()
        self.logger = logger
        self.obj_message = obj_message   
        self.obj_name = obj_message.obj_name
        self.populate()

    def populate(self):
        self.clear()
        for axis in self.obj_message.ref_axis:
            axis: am_msgs.Axis
            item_text = axis.axis_name
            if item_text.startswith(self.obj_name):
                item_text = item_text[len(self.obj_name)+1:]
            item = QListWidgetItem(f"{item_text}")
            self.addItem(item)

class PlanesElementPanel(QListWidget):
    def __init__(self, obj_message:am_msgs.Object, logger=None):
        super().__init__()
        self.logger = logger
        self.obj_message = obj_message    
        self.obj_name = obj_message.obj_name     
        self.populate()

    def populate(self):
        self.clear()
        for plane in self.obj_message.ref_planes:
            plane: am_msgs.Plane
            item_text = plane.ref_plane_name
            if item_text.startswith(self.obj_name):
                item_text = item_text[len(self.obj_name)+1:]
            item = QListWidgetItem(f"{item_text}")
            self.addItem(item)


# ---------------------------------------------------------------------------
# Main scene viewer widget
# ---------------------------------------------------------------------------
class AssemblyScenceViewerWidget(QWidget):
    def __init__(self, ros_node: Node):
        super().__init__()
        self.ros_node = ros_node

        self._current_panel = None
        self._current_item_text = None

        # Subscribe to /assembly_manager/scene
        self._scene_sub = self.ros_node.create_subscription(
            am_msgs.ObjectScene,
            "/assembly_manager/scene",
            self.scene_callback,
            10
        )

        self._assembly_scene = am_msgs.ObjectScene()

        # Main grid layout
        self.layout = QGridLayout(self)

        # ----- Panels -----
        self.objects_panel = ObjectsListPanel(logger=self.ros_node.get_logger())
        self.instructions_panel = InstructionListPanel(logger=self.ros_node.get_logger())

        self.middle_layout = QVBoxLayout()

        self.anylzer = AssemblySceneAnalyzerAdv(scene_data=self._assembly_scene,
                                                logger=self.ros_node.get_logger())

        self.stl_viewer_widget = STLViewerWidget()
        # ----- Add panels to grid -----
        #(row, column, rowSpan, columnSpan[, alignment=Qt.Alignment()])
        self.layout.addWidget(self.objects_panel, 0, 0, 3, 1)             # full height left-side
        self.layout.addLayout(self.middle_layout, 0, 1, 4, 1)      # spans columns 1–2 full height
        self.layout.addWidget(self.instructions_panel, 3, 0, 1, 1)        # full height right-side

        self.layout.addWidget(self.stl_viewer_widget, 2, 2, 2, 1)    
        # ----- Column width proportions -----
        self.layout.setColumnStretch(0, 1)    # 1/4 width
        self.layout.setColumnStretch(1, 1)    # 1/2 width shared (col1 + col2)
        self.layout.setColumnStretch(2, 1)
        self.layout.setColumnStretch(3, 1)    # 1/4 width

        # Full height for all rows
        self.layout.setRowStretch(0, 1)
        self.layout.setRowStretch(1, 1)
        # ----------------------------
        # Connect selection signals
        # ----------------------------
        self.objects_panel.list_widget.itemSelectionChanged.connect(self.on_object_selected)
        self.instructions_panel.list_widget.itemSelectionChanged.connect(self.on_instruction_selected)

    # ----------------------------------------------------------------------
    # Object selection handler
    # ----------------------------------------------------------------------
    def on_object_selected(self):
        item = self.objects_panel.list_widget.currentItem()
        if not item:
            return

        self.instructions_panel.list_widget.clearSelection()
        self.clear_middle_panel()

        obj = next(
            (o for o in self._assembly_scene.objects_in_scene if o.obj_name == item.text()),
            am_msgs.Object()
        )
        self.middle_layout.addWidget(ObjectsElementPanel(obj, logger=self.ros_node.get_logger()))

        item_text = item.text()

        comp:am_msgs.Object = self.anylzer.get_component_by_name(item_text)

        stl_path = comp.cad_data
        self.stl_viewer_widget.reset_stl_file(stl_path)
        self._current_item_text = item.text()
        self._current_panel = self.objects_panel

    # ----------------------------------------------------------------------
    # Instruction selection handler
    # ----------------------------------------------------------------------
    def on_instruction_selected(self):
        item = self.instructions_panel.list_widget.currentItem()
        if item is None:
            return

        self.ros_node.get_logger().warn(f"Selected instruction: {item.text()}")

        # Deselect object list
        self.objects_panel.list_widget.clearSelection()

        # Clear middle panel and show instruction (placeholder)
        self.clear_middle_panel()

        placeholder_panel = QLabel(f"Instruction selected: {item.text()}")
        placeholder_panel.setStyleSheet("font-size: 16pt; padding: 10px;")
        self.middle_layout.addWidget(placeholder_panel)
        
        item_text = item.text()

        inst:am_msgs.AssemblyInstruction = self.anylzer.get_assembly_instruction_by_name(item_text)

        # This is currently emphty
        stl_path = inst.parent_file_path
        #self.stl_viewer_widget.reset_stl_file(stl_path)
        
        self._current_item_text = item.text()
        self._current_panel = self.objects_panel

        # Track current selection
        self._current_item_text = item.text()
        self._current_panel = self.instructions_panel

    # ----------------------------------------------------------------------
    # Clear middle panel safely
    # ----------------------------------------------------------------------
    def clear_middle_panel(self):
        while self.middle_layout.count():
            child = self.middle_layout.takeAt(0)
            if child.widget():
                child.widget().deleteLater()

    # ----------------------------------------------------------------------
    # Scene callback from ROS
    # ----------------------------------------------------------------------

    def scene_callback(self, msg: am_msgs.ObjectScene):
        self._assembly_scene = msg
        self.anylzer.set_scene(msg)
        self.update_panels()

        # Delay restore_selection to next event loop iteration
        QTimer.singleShot(0, self.restore_selection)


    # ----------------------------------------------------------------------
    # Update objects and instructions panels
    # ----------------------------------------------------------------------
    def update_panels(self):
        self.objects_panel.update_scene(self._assembly_scene)
        self.instructions_panel.update_scene(self._assembly_scene)

    # ----------------------------------------------------------------------
    # Restore selection after scene update
    # ----------------------------------------------------------------------
    def restore_selection(self):
        # Nothing to restore
        #self.ros_node.get_logger().warn(f"Restoring selection...{self._current_item_text}")

        if self._current_panel is None or not hasattr(self, "_current_item_text"):
            return

        lw = self._current_panel.list_widget
        for i in range(lw.count()):
            item = lw.item(i)
            if item.text() == self._current_item_text:
                lw.setCurrentItem(item)
                #self.ros_node.get_logger().warn(f"Restored selection: {item.text()}")
                return

        # If no match found
        #self.ros_node.get_logger().warn("Previous selection no longer exists.")
