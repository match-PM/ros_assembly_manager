from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QThread, pyqtSlot, QRunnable, QObject, QThreadPool, QSize
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
                            QStackedWidget,
                            QStyledItemDelegate,
                            QScrollArea as QScrollAreaImport,
                            QGraphicsView,
                            QGraphicsScene,
                            QGraphicsPixmapItem,
                            )


from scipy.spatial.transform import Rotation as R
from PyQt6.QtGui import QColor, QTextCursor, QFont, QAction, QIcon, QPainter, QPixmap
from functools import partial
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import LookupException

from geometry_msgs.msg import TransformStamped, Transform
import assembly_manager_interfaces.msg as am_msgs
import rclpy
import yaml
from assembly_scene_publisher.py_modules.scene_errors import *
from assembly_scene_publisher.py_modules.AssemblySceneAnalyzerAdv import AssemblySceneAnalyzerAdv
from assembly_scene_publisher.py_modules.AssemblySceneAnalyzer import UnInitializedScene
from assembly_scene_viewer.py_modules.STLViewerWidget import STLViewerWidget
from assembly_scene_publisher.py_modules.frame_constraints import FrameConstraintsHandler
# import vtk
# from vtk.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor

# ---------------------------------------------------------------------------
# Widget: Zoomable image viewer
# ---------------------------------------------------------------------------
class ZoomableImageViewer(QGraphicsView):
    """A graphics view widget that allows zooming in/out of images"""
    
    def __init__(self, pixmap: QPixmap):
        super().__init__()
        self.zoom_level = 1.0
        self.min_zoom = 0.1
        self.max_zoom = 5.0
        self.zoom_step = 1.05  # 5% change per scroll wheel - very smooth
        
        # Create scene and add pixmap
        self.scene = QGraphicsScene()
        self.pixmap_item = QGraphicsPixmapItem(pixmap)
        self.scene.addItem(self.pixmap_item)
        
        self.setScene(self.scene)
        self.setDragMode(QGraphicsView.DragMode.ScrollHandDrag)
        
        # **PyQt6 fix for render hint**
        self.setRenderHint(QPainter.RenderHint.SmoothPixmapTransform)
        
        # Fit to view initially
        self.fitInView(self.scene.itemsBoundingRect(), Qt.AspectRatioMode.KeepAspectRatio)
        # Reset zoom level after fit
        self.zoom_level = 1.0
    
    def wheelEvent(self, event):
        """Handle mouse wheel for zooming"""
        if event.angleDelta().y() > 0:
            # Zoom in
            self.zoom_level = min(self.zoom_level * self.zoom_step, self.max_zoom)
        else:
            # Zoom out
            self.zoom_level = max(self.zoom_level / self.zoom_step, self.min_zoom)
        
        # Apply zoom by resetting transform and applying new scale
        self.resetTransform()
        self.scale(self.zoom_level, self.zoom_level)

# ---------------------------------------------------------------------------
# Dialog: Images display dialog for showing multiple images
# ---------------------------------------------------------------------------
class ImagesDialog(QDialog):
    """Dialog to display multiple images related to an object"""
    
    def __init__(self, parent=None, images=None):
        super().__init__(parent)
        self.setWindowTitle("Analysis Graphs")
        self.setGeometry(200, 200, 900, 700)
        
        # Enable maximize button in title bar with proper flags
        self.setWindowFlags(
            Qt.WindowType.Window |                      # regular window
            Qt.WindowType.WindowMinimizeButtonHint |    # enable minimize
            Qt.WindowType.WindowMaximizeButtonHint |    # enable maximize
            Qt.WindowType.WindowCloseButtonHint         # enable close
        )
        
        layout = QVBoxLayout(self)
        
        # Scroll area for images
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        
        # Container for images
        images_container = QWidget()
        images_layout = QVBoxLayout(images_container)
        images_layout.setContentsMargins(0, 0, 0, 0)
        
        # Add images
        if images and len(images) > 0:
            for title, image_data in images:
                # Create a container for each image with title
                img_frame = QWidget()
                img_frame_layout = QVBoxLayout(img_frame)
                img_frame_layout.setContentsMargins(4, 4, 4, 4)
                
                # Image title
                img_title = QLabel(title)
                img_title.setStyleSheet("""
                    font-size: 11pt;
                    font-weight: bold;
                    padding: 4px;
                    background-color: #f0f0f0;
                """)
                img_frame_layout.addWidget(img_title)
                
                # Convert image bytes to QPixmap and display
                if isinstance(image_data, bytes):
                    pixmap = QPixmap()
                    pixmap.loadFromData(image_data)
                    
                    if not pixmap.isNull():
                        # Use zoomable viewer
                        viewer = ZoomableImageViewer(pixmap)
                        viewer.setMinimumHeight(700)
                        img_frame_layout.addWidget(viewer)
                    else:
                        error_label = QLabel("Error: Could not load image data")
                        error_label.setStyleSheet("color: red; padding: 8px;")
                        img_frame_layout.addWidget(error_label)
                else:
                    error_label = QLabel("Error: Invalid image data format")
                    error_label.setStyleSheet("color: red; padding: 8px;")
                    img_frame_layout.addWidget(error_label)
                
                img_frame.setStyleSheet("border: 1px solid #ccc; margin: 4px;")
                images_layout.addWidget(img_frame)
        else:
            placeholder = QLabel("No images available")
            placeholder.setStyleSheet("padding: 8px; color: #999;")
            images_layout.addWidget(placeholder)
        
        images_layout.addStretch()
        scroll_area.setWidget(images_container)
        layout.addWidget(scroll_area)
        
        # Button layout (close button)
        button_layout = QHBoxLayout()
        button_layout.addStretch()
        
        close_button = QPushButton("Close")
        close_button.setMaximumWidth(100)
        close_button.clicked.connect(self.accept)
        button_layout.addWidget(close_button)
        button_layout.addStretch()
        layout.addLayout(button_layout)

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
    def __init__(self, analyzer:AssemblySceneAnalyzerAdv, logger=None):
        super().__init__()
        self.logger = logger
        self.analyzer = analyzer
        self.scene_message = am_msgs.ObjectScene()
        self._current_selected_object = None
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
        # Set custom delegate for rendering status indicator
        self.list_widget.setItemDelegate(ObjectListItemDelegate(self.list_widget))

        layout.addWidget(self.header_label)
        layout.addWidget(self.list_widget)

    def log(self, msg):
        if self.logger:
            self.logger.warn(str(msg))

    def get_current_selected_object(self)->str:
        return self._current_selected_object

    def set_current_selected_object(self, obj_name:str):
        self._current_selected_object = obj_name

    def update_scene(self, msg: am_msgs.ObjectScene):
        self.scene_message = msg
        self.populate()
    
    def populate(self):
        self.list_widget.clear()
        for obj in self.scene_message.objects_in_scene:
            obj: am_msgs.Object
            
            # Create item with object name
            item = QListWidgetItem(obj.obj_name)
            
            # Set icon if gripped
            if obj.properties.is_gripped:
                gripped_icon = QIcon(f"{get_package_share_directory('assembly_scene_viewer')}/media/robotic-hand.png")
                item.setIcon(gripped_icon)
            
            # Determine assembled status and store as item data for delegate to render
            try:
                is_assembled = self.analyzer.check_component_assembled_adv(obj.obj_name)
            except:
                is_assembled = obj.properties.is_assembled
            
            # Store status color in item data for delegate to render
            if is_assembled:
                item.setData(Qt.ItemDataRole.UserRole, "green")
            else:
                item.setData(Qt.ItemDataRole.UserRole, "red")
            
            self.list_widget.addItem(item)


class ObjectListItemDelegate(QStyledItemDelegate):
    """Custom delegate to render status indicator circle without affecting text styling"""
    
    def paint(self, painter: QPainter, option, index):
        # Call parent paint to render icon and text normally
        super().paint(painter, option, index)
        
        # Get status color from item data
        status_color_name = index.data(Qt.ItemDataRole.UserRole)
        if status_color_name:
            status_color = QColor(status_color_name)
            
            # Draw indicator circle on the right side of the item
            circle_radius = 5
            circle_x = option.rect.right() - circle_radius - 8
            circle_y = option.rect.center().y()
            
            painter.setBrush(status_color)
            painter.setPen(Qt.PenStyle.NoPen)
            painter.drawEllipse(int(circle_x - circle_radius), 
                              int(circle_y - circle_radius), 
                              circle_radius * 2, 
                              circle_radius * 2)

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
                 scene_message: am_msgs.ObjectScene = None,
                 logger=None):
        super().__init__()
        self.logger = logger
        self.obj = obj
        self.scene_message = scene_message or am_msgs.ObjectScene()

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
                                               scene_message=scene_message,
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
        
        # ---- Connect mutual exclusion signals ----
        self.frames_panel.itemSelectionChanged.connect(self._on_frame_selection_changed)
        self.axis_panel.itemSelectionChanged.connect(self._on_axis_selection_changed)
        self.planes_panel.itemSelectionChanged.connect(self._on_plane_selection_changed)

    def _on_frame_selection_changed(self):
        """Clear axis and plane selection when frame is selected"""
        if self.frames_panel.currentItem():
            self.axis_panel.blockSignals(True)
            self.planes_panel.blockSignals(True)
            self.axis_panel.clearSelection()
            self.planes_panel.clearSelection()
            self.axis_panel.blockSignals(False)
            self.planes_panel.blockSignals(False)

    def _on_axis_selection_changed(self):
        """Clear frame and plane selection when axis is selected"""
        if self.axis_panel.currentItem():
            self.frames_panel.blockSignals(True)
            self.planes_panel.blockSignals(True)
            self.frames_panel.clearSelection()
            self.planes_panel.clearSelection()
            self.frames_panel.blockSignals(False)
            self.planes_panel.blockSignals(False)

    def _on_plane_selection_changed(self):
        """Clear frame and axis selection when plane is selected"""
        if self.planes_panel.currentItem():
            self.frames_panel.blockSignals(True)
            self.axis_panel.blockSignals(True)
            self.frames_panel.clearSelection()
            self.axis_panel.clearSelection()
            self.frames_panel.blockSignals(False)
            self.axis_panel.blockSignals(False)

    def set_scene(self, scene_message: am_msgs.ObjectScene):
        """Update scene data for constraint lookup"""
        self.scene_message = scene_message
        self.frames_panel.set_scene(scene_message)
    
    def clear_frame_highlighting(self):
        """Clear any frame highlighting"""
        for i in range(self.frames_panel.count()):
            item = self.frames_panel.item(i)
            item.setBackground(QColor("white"))
            item.setForeground(QColor("black"))
            font = item.font()
            font.setBold(False)
            item.setFont(font)


class FramesElementPanel(QListWidget):
    # Signal to notify when a frame is selected
    frame_selected = pyqtSignal(str)  # Emits full frame name
    
    def __init__(self, obj_message:am_msgs.Object, 
                 scene_message: am_msgs.ObjectScene = None,
                 logger=None):
        super().__init__()
        self.logger = logger
        self.obj_message = obj_message   
        self.obj_name = obj_message.obj_name
        self.scene_message = scene_message or am_msgs.ObjectScene()
        
        # Set custom delegate for rendering status indicator
        self.setItemDelegate(FrameListItemDelegate(self))
        
        self.populate()
        
        # Connect to item selection signal
        self.itemSelectionChanged.connect(self._on_frame_selected)

    def set_scene(self, scene_message: am_msgs.ObjectScene):
        """Update scene data for constraint lookup"""
        self.scene_message = scene_message

    def _on_frame_selected(self):
        """Handle frame selection and emit signal with full frame name"""
        item = self.currentItem()
        if item:
            # Get the full frame name from the object's frames
            for frame in self.obj_message.ref_frames:
                display_text = frame.frame_name
                if display_text.startswith(self.obj_name):
                    display_text = display_text[len(self.obj_name)+1:]
                if display_text == item.text():
                    self.frame_selected.emit(frame.frame_name)
                    break

    def _get_constraint_status(self, frame: am_msgs.RefFrame) -> tuple:
        """
        Determine constraint status based on frame properties: (color, description)
        Only shows status for glue, vision, or laser frames.
        Green: Property action completed
        Yellow: Property action pending
        Red: Property action failed or not started
        Returns (None, None) if frame doesn't have these properties.
        """
        props = frame.properties
        
        # Check if this is a glue point frame
        if props.glue_pt_frame_properties.is_glue_point:
            if props.glue_pt_frame_properties.has_been_cured:
                return (QColor("green"), "Glue cured")
            elif props.glue_pt_frame_properties.has_been_placed:
                return (QColor("orange"), "Glue placed, curing...")
            else:
                return (QColor("red"), "Glue not placed")
        
        # Check if this is a vision frame
        if props.vision_frame_properties.is_vision_frame:
            if props.vision_frame_properties.has_been_measured:
                return (QColor("green"), "Vision measured")
            else:
                return (QColor("red"), "Vision not measured")
        
        # Check if this is a laser frame
        if props.laser_frame_properties.is_laser_frame:
            if props.laser_frame_properties.has_been_measured:
                return (QColor("green"), "Laser measured")
            else:
                return (QColor("red"), "Laser not measured")
        
        # No relevant properties
        return (None, None)

    def populate(self):
        self.clear()
        # Icon mapping for frame types
        icon_map = {
            'vision': QIcon(f"{get_package_share_directory('assembly_scene_viewer')}/media/vision.png"),
            'glue': QIcon(f"{get_package_share_directory('assembly_scene_viewer')}/media/glue.png"),
            'laser': QIcon(f"{get_package_share_directory('assembly_scene_viewer')}/media/laser.png"),
            'grip': QIcon(f"{get_package_share_directory('assembly_scene_viewer')}/media/robotic-hand.png"),
            'assembly': QIcon(f"{get_package_share_directory('assembly_scene_viewer')}/media/assembly.png"),
            'target': QIcon(f"{get_package_share_directory('assembly_scene_viewer')}/media/target.png"),
            'chain': QIcon(f"{get_package_share_directory('assembly_scene_viewer')}/media/chain.png"),
            "default": QIcon(f"{get_package_share_directory('assembly_scene_viewer')}/media/default_frame.png"),
        }
        for frame in self.obj_message.ref_frames:
            frame: am_msgs.RefFrame
            # strip the obj_name prefix if present
            item_text = frame.frame_name
            if item_text.startswith(self.obj_name):
                item_text = item_text[len(self.obj_name)+1:]
            
            # Determine frame type and select icon
            icon = None
            props = frame.properties
            if props.vision_frame_properties.is_vision_frame:
                icon = icon_map['vision']
            elif props.glue_pt_frame_properties.is_glue_point:
                icon = icon_map['glue']
            elif props.laser_frame_properties.is_laser_frame:
                icon = icon_map['laser']
            elif props.gripping_frame_properties.is_gripping_frame:
                icon = icon_map['grip']
            elif props.assembly_frame_properties.is_assembly_frame:
                icon = icon_map['assembly']
            elif props.assembly_frame_properties.is_target_frame:
                icon = icon_map['target']
            elif (frame.constraints.centroid.is_active 
                  or frame.constraints.in_plane.is_active 
                  or frame.constraints.orthogonal.is_active
                  or frame.constraints.transform.is_active):
                icon = icon_map['chain']
            else:
                icon = icon_map['default']

            # Get status indicator (color and tooltip)
            status_color, status_tooltip = self._get_constraint_status(frame)
            
            # Create item with just the text (no status circle in text)
            item = QListWidgetItem(item_text)
            if icon:
                item.setIcon(icon)
            
            # Store status color in item data for delegate to render
            if status_color:
                item.setData(Qt.ItemDataRole.UserRole, status_color.name())
                item.setToolTip(status_tooltip)
            else:
                item.setData(Qt.ItemDataRole.UserRole, None)
            
            self.addItem(item)
 

class FrameListItemDelegate(QStyledItemDelegate):
    """Custom delegate to render status indicator circle without affecting text styling"""
    
    def paint(self, painter: QPainter, option, index):
        # Call parent paint to render icon and text normally
        super().paint(painter, option, index)
        
        # Get status color from item data
        status_color_name = index.data(Qt.ItemDataRole.UserRole)
        if status_color_name:
            status_color = QColor(status_color_name)
            
            # Draw indicator circle on the right side of the item
            circle_radius = 5
            circle_x = option.rect.right() - circle_radius - 8
            circle_y = option.rect.center().y()
            
            painter.setBrush(status_color)
            painter.setPen(Qt.PenStyle.NoPen)
            painter.drawEllipse(int(circle_x - circle_radius), 
                              int(circle_y - circle_radius), 
                              circle_radius * 2, 
                              circle_radius * 2)

 

class AxisElementPanel(QListWidget):
    # Signal to notify when an axis is selected
    axis_selected = pyqtSignal(str)  # Emits full axis name
    
    def __init__(self, obj_message:am_msgs.Object, logger=None):
        super().__init__()
        self.logger = logger
        self.obj_message = obj_message   
        self.obj_name = obj_message.obj_name
        self.populate()
        
        # Connect to item selection signal
        self.itemSelectionChanged.connect(self._on_axis_selected)

    def _on_axis_selected(self):
        """Handle axis selection and emit signal with full axis name"""
        item = self.currentItem()
        if item:
            # Get the full axis name from the object's axes
            for axis in self.obj_message.ref_axis:
                display_text = axis.axis_name
                if display_text.startswith(self.obj_name):
                    display_text = display_text[len(self.obj_name)+1:]
                if display_text == item.text():
                    self.axis_selected.emit(axis.axis_name)
                    break

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
    # Signal to notify when a plane is selected
    plane_selected = pyqtSignal(str)  # Emits full plane name
    
    def __init__(self, obj_message:am_msgs.Object, logger=None):
        super().__init__()
        self.logger = logger
        self.obj_message = obj_message    
        self.obj_name = obj_message.obj_name
        self.populate()
        
        # Connect to item selection signal
        self.itemSelectionChanged.connect(self._on_plane_selected)

    def _on_plane_selected(self):
        """Handle plane selection and emit signal with full plane name"""
        item = self.currentItem()
        if item:
            # Get the full plane name from the object's planes
            for plane in self.obj_message.ref_planes:
                display_text = plane.ref_plane_name
                if display_text.startswith(self.obj_name):
                    display_text = display_text[len(self.obj_name)+1:]
                if display_text == item.text():
                    self.plane_selected.emit(plane.ref_plane_name)
                    break

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
# Widget: Instruction Detail Panel with properties
# ---------------------------------------------------------------------------
class InstructionDetailPanel(QWidget):
    def __init__(self, instruction: am_msgs.AssemblyInstruction, analyzer, logger=None):
        super().__init__()
        self.logger = logger
        self.instruction = instruction
        self.analyzer = analyzer
        
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        # Header with instruction ID
        header_label = QLabel(f"Instruction: {instruction.id}")
        header_label.setStyleSheet("""
            font-size: 14pt;
            font-weight: bold;
            padding: 4px;
            background-color: #f0f0f0;
        """)
        layout.addWidget(header_label)
        
        # ---- Properties Section ----
        properties_label = QLabel("Instruction Details")
        properties_label.setStyleSheet("""
            font-size: 12pt;
            font-weight: bold;
            padding: 4px;
        """)
        layout.addWidget(properties_label)
        
        # Properties text display
        self.properties_text = QTextEdit()
        self.properties_text.setReadOnly(True)
        self.properties_text.setStyleSheet("""
            QTextEdit {
                font-family: monospace;
                font-size: 10pt;
            }
        """)
        layout.addWidget(self.properties_text, 1)
        
        # Populate the panel
        self._display_instruction_properties()
    
    def _display_instruction_properties(self):
        """Display all instruction details in the properties text box"""
        try:
            moving_part = "Component 1" if self.instruction.component_1_is_moving_part else "Component 2"
            
            html = f"""
            <b>Instruction ID:</b> {self.instruction.id}<br>
            <br>
            <b>Components:</b><br>
            &nbsp;&nbsp;<b>Component 1:</b> {self.instruction.component_1 or 'N/A'}<br>
            &nbsp;&nbsp;<b>Component 2:</b> {self.instruction.component_2 or 'N/A'}<br>
            &nbsp;&nbsp;<b>Moving Part:</b> {moving_part}<br>
            <br>

            """
            
            self.properties_text.setHtml(html)
        except Exception as e:
            if self.logger:
                self.logger.error(f"Error displaying instruction properties: {e}")
            self.properties_text.setHtml(f"<b>Error:</b> {str(e)}")


class InstructionStlPanel(QWidget):
    """Right-side panel for instruction view: two STL viewers (component 1 and component 2)."""

    def __init__(self, logger=None):
        super().__init__()
        self.logger = logger

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        label = QLabel("Instruction Components")
        label.setStyleSheet("font-size: 12pt; font-weight: bold; padding: 4px;")
        layout.addWidget(label)

        # Reset buttons container
        buttons_layout = QHBoxLayout()
        reset_top_btn = QPushButton("Reset Top View")
        reset_top_btn.setMaximumHeight(25)
        reset_top_btn.clicked.connect(self._on_reset_top_view)
        
        reset_bottom_btn = QPushButton("Reset Bottom View")
        reset_bottom_btn.setMaximumHeight(25)
        reset_bottom_btn.clicked.connect(self._on_reset_bottom_view)
        
        buttons_layout.addWidget(reset_top_btn)
        buttons_layout.addWidget(reset_bottom_btn)
        layout.addLayout(buttons_layout)

        self.viewer_top = STLViewerWidget()
        self.viewer_bottom = STLViewerWidget()

        layout.addWidget(self.viewer_top, 1)
        layout.addWidget(self.viewer_bottom, 1)

    def _on_reset_top_view(self):
        """Reset the top STL viewer camera."""
        try:
            self.viewer_top.reset_camera()
        except Exception as e:
            if self.logger:
                self.logger.warn(f"Error resetting top view: {e}")

    def _on_reset_bottom_view(self):
        """Reset the bottom STL viewer camera."""
        try:
            self.viewer_bottom.reset_camera()
        except Exception as e:
            if self.logger:
                self.logger.warn(f"Error resetting bottom view: {e}")

    def clear(self):
        try:
            self.viewer_top.clear_all_frames()
        except Exception:
            pass
        try:
            self.viewer_bottom.clear_all_frames()
        except Exception:
            pass

    def _get_plane_data(self, plane_obj: am_msgs.Plane, analyzer: AssemblySceneAnalyzerAdv) -> dict:
        """
        Extract plane data from a Plane message.
        Returns dict with 'points' and/or 'axis' keys, or empty dict if plane cannot be constructed.
        """
        plane_data = {}
        
        # Case 1: Plane defined by three points
        if plane_obj.axis_names[0] == '' and plane_obj.point_names[0] != '':
            points = []
            for point_name in plane_obj.point_names:
                if point_name != '':
                    try:
                        frame = analyzer.get_ref_frame_by_name(point_name)
                        if frame and frame.pose:
                            points.append(frame.pose.position)
                    except Exception:
                        pass
            if len(points) >= 3:
                plane_data["points"] = points
        
        # Case 2: Plane defined by axis and one point
        elif plane_obj.axis_names[0] != '' and plane_obj.point_names[0] != '':
            try:
                axis_frames = analyzer.get_frames_for_axis(plane_obj.axis_names[0])
                if len(axis_frames) >= 2 and axis_frames[0].pose and axis_frames[1].pose:
                    plane_data["axis"] = [axis_frames[0].pose.position, axis_frames[1].pose.position]
                
                # Get the support point
                support_frame = analyzer.get_ref_frame_by_name(plane_obj.point_names[0])
                if support_frame and support_frame.pose:
                    plane_data["points"] = [support_frame.pose.position]
            except Exception:
                pass
        
        return plane_data

    def set_instruction(self, inst: am_msgs.AssemblyInstruction, analyzer: AssemblySceneAnalyzerAdv):
        """Load STL files for the two components and draw planes from all plane matches."""
        self.clear()

        comp1_name = inst.component_1
        comp2_name = inst.component_2

        if not comp1_name or not comp2_name:
            if self.logger:
                self.logger.warn(f"Instruction has missing component names: comp1={comp1_name}, comp2={comp2_name}")
            return

        try:
            if inst.component_1_is_moving_part:
                moving_component = comp1_name
                stationary_component = comp2_name
                frame_top = analyzer.get_assembly_frame_for_instruction(inst.id)
                frame_bottom = analyzer.get_target_frame_for_instruction(inst.id)
            else:
                moving_component = comp2_name
                stationary_component = comp1_name
                frame_top = analyzer.get_target_frame_for_instruction(inst.id)
                frame_bottom = analyzer.get_assembly_frame_for_instruction(inst.id)
        except Exception as e:
            if self.logger:
                self.logger.warn(f"Could not retrieve assembly/target frames for instruction {inst.id}: {e}")
            frame_top = None
            frame_bottom = None

        # Color palette for plane matches (3 distinct colors)
        plane_colors = [
            (1.0, 0.0, 0.0),    # Red
            (0.0, 1.0, 0.0),    # Green
            (0.0, 0.0, 1.0),    # Blue
        ]

        # Load STL files
        if comp1_name:
            try:
                comp1 = analyzer.get_component_by_name(comp1_name)
                if comp1 and comp1.cad_data:
                    self.viewer_top.reset_stl_file(comp1.cad_data)
            except Exception as e:
                if self.logger:
                    self.logger.warn(f"Could not load STL for component '{comp1_name}': {e}")

        if comp2_name:
            try:
                comp2 = analyzer.get_component_by_name(comp2_name)
                if comp2 and comp2.cad_data:
                    self.viewer_bottom.reset_stl_file(comp2.cad_data)
            except Exception as e:
                if self.logger:
                    self.logger.warn(f"Could not load STL for component '{comp2_name}': {e}")

        # Draw planes for all three plane matches
        # Each plane_match_* is a ConstraintPlanes object with plane_name_component_1 and plane_name_component_2
        plane_match_attrs = ["plane_match_1", "plane_match_2", "plane_match_3"]
        for match_idx, plane_match_attr in enumerate(plane_match_attrs):
            if match_idx >= len(plane_colors):
                break
            
            constraint_planes = getattr(inst, plane_match_attr, None)
            if not constraint_planes:
                continue
            
            plane_name_comp1 = getattr(constraint_planes, "plane_name_component_1", "")
            plane_name_comp2 = getattr(constraint_planes, "plane_name_component_2", "")
            
            color_rgb = plane_colors[match_idx]
            
            # Draw plane for component 1
            if plane_name_comp1:
                try:
                    plane_obj = analyzer.get_plane_from_scene(plane_name_comp1)
                    plane_data = self._get_plane_data(plane_obj, analyzer)
                    
                    if plane_data:
                        self.viewer_top.display_plane(
                            f"{plane_name_comp1}",
                            plane_data,
                            color_rgb=color_rgb,
                            alpha=0.3
                        )
                except Exception as e:
                    if self.logger:
                        self.logger.warn(f"Could not draw plane '{plane_name_comp1}': {e}")
            
            # Draw plane for component 2
            if plane_name_comp2:
                try:
                    plane_obj = analyzer.get_plane_from_scene(plane_name_comp2)
                    plane_data = self._get_plane_data(plane_obj, analyzer)
                    
                    if plane_data:
                        self.viewer_bottom.display_plane(
                            f"{plane_name_comp2}",
                            plane_data,
                            color_rgb=color_rgb,
                            alpha=0.3
                        )
                except Exception as e:
                    if self.logger:
                        self.logger.warn(f"Could not draw plane '{plane_name_comp2}': {e}")
        
        # Draw assembly and target frames for both components
        try:
            if frame_top and frame_bottom:
                is_top_assembly = inst.component_1_is_moving_part
                self._draw_assembly_and_target_frames(frame_top, frame_bottom, frame_top_is_assembly=is_top_assembly)
        except Exception as e:
            if self.logger:
                self.logger.warn(f"Could not draw assembly/target frames: {e}")

    def _draw_assembly_and_target_frames(self, frame_top: am_msgs.RefFrame, 
                                         frame_bottom: am_msgs.RefFrame,
                                         frame_top_is_assembly: bool = True):
        """Draw assembly and target frames for the two components.
        
        Args:
            frame_top: Frame to draw in top viewer
            frame_bottom: Frame to draw in bottom viewer
            frame_top_is_assembly: If True, frame_top is assembly (orange) and frame_bottom is target (cyan).
                                   If False, frame_top is target (cyan) and frame_bottom is assembly (orange).
        """
        # Frame type colors
        assembly_frame_color = (1.0, 0.8, 0.0)    # Orange for assembly frames
        target_frame_color = (0.0, 1.0, 1.0)      # Cyan for target frames

        if frame_top:
            top_color = assembly_frame_color if frame_top_is_assembly else target_frame_color
            self.viewer_top.display_frame(
                frame_top.frame_name,
                frame_top.pose.position,
                frame_top.pose.orientation,
                scale=0.02,
                color_rgb=top_color
            )
            
        if frame_bottom:
            bottom_color = target_frame_color if frame_top_is_assembly else assembly_frame_color
            self.viewer_bottom.display_frame(
                frame_bottom.frame_name,
                frame_bottom.pose.position,
                frame_bottom.pose.orientation,
                scale=0.02,
                color_rgb=bottom_color
            )


# ---------------------------------------------------------------------------
# Widget: Properties panel for displaying frame/axis/plane properties
# ---------------------------------------------------------------------------
class PropertiesPanel(QWidget):
    def __init__(self, logger=None):
        super().__init__()
        self.logger = logger
        
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        # Header
        self.header_label = QLabel("Properties")
        self.header_label.setStyleSheet("""
            font-size: 12pt;
            font-weight: bold;
            padding: 4px;
        """)
        layout.addWidget(self.header_label)
        
        # Text display for properties
        self.properties_text = QTextEdit()
        self.properties_text.setReadOnly(True)
        self.properties_text.setStyleSheet("""
            QTextEdit {
                font-family: monospace;
                font-size: 10pt;
            }
        """)
        layout.addWidget(self.properties_text)
    
    def display_frame_properties(self, frame: am_msgs.RefFrame):
        constriant_handler = FrameConstraintsHandler(logger=self.logger)
        constriant_handler.set_from_msg(frame.constraints)
        """Display properties of a frame"""
        text = f"<b>Frame: {frame.frame_name}</b><br>"
        text += f"Parent Frame: {frame.parent_frame}<br>"
        text += f"<br><b>Pose:</b><br>"
        text += f"  Position: ({frame.pose.position.x:.4f}, {frame.pose.position.y:.4f}, {frame.pose.position.z:.4f})<br>"
        text += f"  Orientation: ({frame.pose.orientation.x:.4f}, {frame.pose.orientation.y:.4f}, {frame.pose.orientation.z:.4f}, {frame.pose.orientation.w:.4f})<br>"
        
        text += f"<br><b>Constraints:</b><br>"
        text += constriant_handler.get_info()
        
        text += f"<br><b>Frame Properties:</b><br>"
        
        if frame.properties.vision_frame_properties.is_vision_frame:
            text += f"  <b>Vision Frame:</b><br>"
            text += f"    Has Been Measured: {frame.properties.vision_frame_properties.has_been_measured}<br>"
        
        if frame.properties.glue_pt_frame_properties.is_glue_point:
            text += f"  <b>Glue Point Frame:</b><br>"
            text += f"    Has Been Placed: {frame.properties.glue_pt_frame_properties.has_been_placed}<br>"
            text += f"    Dispense Offset (mm): {frame.properties.glue_pt_frame_properties.dispense_offset_mm}<br>"
            text += f"    Curing Time (ms): {frame.properties.glue_pt_frame_properties.time_ms}<br>"
            text += f"    Has Been Cured: {frame.properties.glue_pt_frame_properties.has_been_cured}<br>"
        
        if frame.properties.laser_frame_properties.is_laser_frame:
            text += f"  <b>Laser Frame:</b><br>"
            text += f"    Has Been Measured: {frame.properties.laser_frame_properties.has_been_measured}<br>"
        
        if frame.properties.gripping_frame_properties.is_gripping_frame:
            text += f"  <b>Gripping Frame:</b><br>"
            text += f"    Compatible Gripper Tips: {frame.properties.gripping_frame_properties.compatible_gripper_tips}<br>"
            text += f"    Compatible Grippers: {frame.properties.gripping_frame_properties.compatible_grippers}<br>"

        
        if frame.properties.assembly_frame_properties.is_assembly_frame:
            text += f"  <b>Assembly Frame:</b><br>"
            text += f"    Associated Frame: {frame.properties.assembly_frame_properties.associated_frame}<br>"

        if frame.properties.assembly_frame_properties.is_target_frame:
            text += f"  <b>Assembly Target Frame:</b><br>"
            text += f"    Associated Frame: {frame.properties.assembly_frame_properties.associated_frame}<br>"        
        
        self.properties_text.setHtml(text)
    
    def display_axis_properties(self, axis: am_msgs.Axis):
        """Display properties of an axis"""
        text = f"<b>Axis: {axis.axis_name}</b><br>"
        self.properties_text.setHtml(text)
    
    def display_plane_properties(self, plane: am_msgs.Plane):
        """Display properties of a plane"""
        text = f"<b>Plane: {plane.ref_plane_name}</b><br>"
        self.properties_text.setHtml(text)
    
    def clear(self):
        """Clear the properties display"""
        self.properties_text.clear()


# ---------------------------------------------------------------------------
# Main scene viewer widget
# ---------------------------------------------------------------------------
class AssemblyScenceViewerWidget(QWidget):
    def __init__(self, ros_node: Node):
        super().__init__()
        self.ros_node = ros_node

        self._current_panel = None
        self._current_item_text = None
        self._current_element_panel = None  # Track the ObjectsElementPanel
        self._current_selected_frame = None  # Track selected frame
        self._current_selected_axis = None  # Track selected axis
        self._current_selected_plane = None  # Track selected plane

        # Subscribe to /assembly_manager/scene
        self._scene_sub = self.ros_node.create_subscription(
            am_msgs.ObjectScene,
            "/assembly_manager/scene",
            self.scene_callback,
            10
        )

        self._assembly_scene = am_msgs.ObjectScene()

        # Initialize analyzer with empty scene
        self.anylzer = AssemblySceneAnalyzerAdv(scene_data=self._assembly_scene,
                                                logger=self.ros_node.get_logger())
        
        # Main grid layout
        self.layout = QGridLayout(self)

        # ----- Panels -----
        self.objects_panel = ObjectsListPanel(analyzer=self.anylzer, 
                                              logger=self.ros_node.get_logger())
        
        self.instructions_panel = InstructionListPanel(logger=self.ros_node.get_logger())

        # Middle column: we'll switch between object details (frames/axis/planes)
        # and instruction properties.
        self.middle_stack = QStackedWidget()
        self._middle_object_container = QWidget()
        self.middle_layout = QVBoxLayout(self._middle_object_container)
        self.middle_layout.setContentsMargins(0, 0, 0, 0)
        self._middle_object_container.setLayout(self.middle_layout)

        self.instruction_properties_panel = InstructionDetailPanel(
            am_msgs.AssemblyInstruction(),
            analyzer=None,
            logger=self.ros_node.get_logger(),
        )
        self.middle_stack.addWidget(self._middle_object_container)        # index 0
        self.middle_stack.addWidget(self.instruction_properties_panel)    # index 1

        # Right column: Create stacked views for object vs instruction mode
        self.right_stack = QStackedWidget()
        
        # Object mode right side: STL viewer (top) + button + properties panel (bottom)
        self.object_right_container = QWidget()
        object_right_layout = QVBoxLayout(self.object_right_container)
        object_right_layout.setContentsMargins(0, 0, 0, 0)
        
        # Add reset view button
        reset_button = QPushButton("Reset View")
        reset_button.setMaximumHeight(30)
        reset_button.clicked.connect(self._on_reset_view_clicked)
        object_right_layout.addWidget(reset_button)
        
        self.stl_viewer_widget = STLViewerWidget()
        
        # Add images button
        self.images_button = QPushButton("View Images")
        self.images_button.setMaximumHeight(35)
        self.images_button.setStyleSheet("""
            QPushButton {
                background-color: #e8f4f8;
                border: 1px solid #4da6b5;
                padding: 4px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #d0e8f0;
            }
            QPushButton:pressed {
                background-color: #b8dce8;
            }
        """)
        self.images_button.clicked.connect(self._on_images_button_clicked)
        
        self.properties_panel = PropertiesPanel(logger=self.ros_node.get_logger())
        object_right_layout.addWidget(self.stl_viewer_widget, 3)      # STL takes 3/4 of space
        object_right_layout.addWidget(self.images_button, 0)           # Button takes minimal space
        object_right_layout.addWidget(self.properties_panel, 1)       # Properties takes 1/4 of space
        
        # Instruction mode right side: two STL viewers (full width)
        self.instruction_stl_panel = InstructionStlPanel(logger=self.ros_node.get_logger())
        
        self.right_stack.addWidget(self.object_right_container)       # index 0: object mode
        self.right_stack.addWidget(self.instruction_stl_panel)        # index 1: instruction mode

        # Layout setup
        self.layout.addWidget(self.objects_panel, 0, 0, 2, 1)              # Left: objects (2 rows)
        self.layout.addWidget(self.instructions_panel, 2, 0, 2, 1)         # Left: instructions (2 rows)
        self.layout.addWidget(self.middle_stack, 0, 1, 4, 1)               # Middle: object/instruction details
        self.layout.addWidget(self.right_stack, 0, 2, 4, 1)                # Right: takes all 4 rows

        # Connect signals
        self.objects_panel.list_widget.itemSelectionChanged.connect(self.on_object_selected)
        self.instructions_panel.list_widget.itemSelectionChanged.connect(self.on_instruction_selected)


    def on_object_selected(self):
        item = self.objects_panel.list_widget.currentItem()
        if not item:
            return


        # Switch UI to object view
        try:
            self.middle_stack.setCurrentIndex(0)
            self.right_stack.setCurrentIndex(0)
        except Exception:
            pass


        self.instructions_panel.list_widget.clearSelection()
        self.clear_middle_panel()
        self.properties_panel.clear()

        obj = next(
            (o for o in self._assembly_scene.objects_in_scene if o.obj_name == item.text()),
            am_msgs.Object()
        )
        
        self.objects_panel.set_current_selected_object(obj.obj_name)

        element_panel = ObjectsElementPanel(obj, 
                                           scene_message=self._assembly_scene,
                                           logger=self.ros_node.get_logger())
        self._current_element_panel = element_panel
        
        # Connect frame selection signal to mark constraining frames
        element_panel.frames_panel.frame_selected.connect(self.on_frame_selected)
        element_panel.axis_panel.axis_selected.connect(self.on_axis_selected)
        element_panel.planes_panel.plane_selected.connect(self.on_plane_selected)
        
        self.middle_layout.addWidget(element_panel)
        
        # Restore previous selection
        self._restore_element_selection(element_panel)

        item_text = item.text()

        comp:am_msgs.Object = self.anylzer.get_component_by_name(item_text)

        stl_path = comp.cad_data
        
        # Clear all visualizations when switching to a new object
        if self.stl_viewer_widget:
            self.stl_viewer_widget.clear_all_frames()
        
        self.stl_viewer_widget.reset_stl_file(stl_path)
        self._current_item_text = item.text()
        self._current_panel = self.objects_panel
        
        # Reset selection tracking
        self._current_selected_frame = None
        self._current_selected_axis = None
        self._current_selected_plane = None

    # ----------------------------------------------------------------------
    # Reset view button handler
    # ----------------------------------------------------------------------
    def _on_reset_view_clicked(self):
        """Reset the STL viewer camera to default view."""
        if self.stl_viewer_widget:
            self.stl_viewer_widget.reset_camera()

    # ----------------------------------------------------------------------
    # Images button handler
    # ----------------------------------------------------------------------
    def _on_images_button_clicked(self):
        """Open the images dialog for the current object."""
        # Get currently selected object
        obj_name = self.objects_panel.get_current_selected_object()

        if not obj_name:
            QMessageBox.warning(self, "No Object Selected", "Please select an object first.")
            return
                
        # For now, pass an empty list of images
        # In the future, this will be populated with actual images from the object
        images_list = []
        frame_graph = self.anylzer.get_frame_graph_for_component(obj_name)
        instructions_graph = self.anylzer.get_instructions_graph_for_component(obj_name)
        plane_graph = self.anylzer.get_plane_graph_for_component(obj_name)
        
        if frame_graph:
            images_list.append(("Frame Constraints Graph", frame_graph))
        if instructions_graph:
            images_list.append(("Instructions Graph", instructions_graph))
        if plane_graph:
            images_list.append(("Plane Constraints Graph", plane_graph))

        # Open the images dialog
        dialog = ImagesDialog(self, images=images_list)
        dialog.exec()

    # ----------------------------------------------------------------------
    # Frame selection handler - marks constraining frames
    # ----------------------------------------------------------------------
    def on_frame_selected(self, full_frame_name: str):
        if not self._current_element_panel:
            return

        obj_name = self._current_element_panel.frames_panel.obj_name

        display_name = full_frame_name
        if display_name.startswith(obj_name + "_"):
            display_name = display_name[len(obj_name)+1:]

        self._current_selected_frame = display_name
        self._current_selected_axis = None
        self._current_selected_plane = None
        
        # Display frame properties
        frame_obj = self.anylzer.get_ref_frame_by_name(full_frame_name)
        self.properties_panel.display_frame_properties(frame_obj)

        # Display frame visualization in STL viewer
        if self.stl_viewer_widget and frame_obj:
            self.stl_viewer_widget.clear_all_frames()
            if frame_obj.pose:
                self.stl_viewer_widget.display_frame(
                    full_frame_name,
                    frame_obj.pose.position,
                    frame_obj.pose.orientation
                )

        constraining_frames = self.find_constraining_frames(full_frame_name)
        self.highlight_constraining_frames(constraining_frames)
        
        # Display constraining frames with orange dots in STL viewer
        if self.stl_viewer_widget and constraining_frames:
            positions = {}
            orientations = {}
            for cf in constraining_frames:
                cf_obj = self.anylzer.get_ref_frame_by_name(cf)
                if cf_obj and cf_obj.pose:
                    positions[cf] = cf_obj.pose.position
                    orientations[cf] = cf_obj.pose.orientation
            if positions:
                self.stl_viewer_widget.display_constraining_frames(constraining_frames, positions, orientations)



    # ----------------------------------------------------------------------
    # Axis selection handler - marks defining frames and axes
    # ----------------------------------------------------------------------
    def on_axis_selected(self, full_axis_name: str):
        if not self._current_element_panel:
            return

        obj_name = self._current_element_panel.axis_panel.obj_name

        display_name = full_axis_name
        if display_name.startswith(obj_name + "_"):
            display_name = display_name[len(obj_name)+1:]

        self._current_selected_axis = display_name
        self._current_selected_frame = None
        self._current_selected_plane = None
        
        # Display axis properties
        axis_obj = self.anylzer.get_axis_from_scene(full_axis_name)
        self.properties_panel.display_axis_properties(axis_obj)

        # Clear frame visualization when switching to axis
        if self.stl_viewer_widget:
            self.stl_viewer_widget.clear_all_frames()
        
        # Display axis visualization in STL viewer
        if self.stl_viewer_widget and axis_obj:
            # Get the two frames that define the axis
            frames = self.anylzer.get_frames_for_axis(full_axis_name)
            if len(frames) == 2 and frames[0].pose and frames[1].pose:
                self.stl_viewer_widget.display_axis(
                    full_axis_name,
                    frames[0].pose.position,
                    frames[1].pose.position,
                    color_rgb=(1, 1, 0)  # Yellow
                )

        self._current_element_panel.clear_frame_highlighting()
        defining_items = self.find_defining_frames_and_axes_for_axis(full_axis_name)
        self.highlight_defining_items(defining_items)



    # ----------------------------------------------------------------------
    # Plane selection handler - marks defining frames and axes
    # ----------------------------------------------------------------------
    def on_plane_selected(self, full_plane_name: str):
        if not self._current_element_panel:
            return

        obj_name = self._current_element_panel.planes_panel.obj_name

        display_name = full_plane_name
        if display_name.startswith(obj_name + "_"):
            display_name = display_name[len(obj_name)+1:]

        self._current_selected_plane = display_name
        self._current_selected_frame = None
        self._current_selected_axis = None
        
        # Display plane properties
        plane_obj = self.anylzer.get_plane_from_scene(full_plane_name)
        self.properties_panel.display_plane_properties(plane_obj)

        # Clear frame visualization when switching to plane
        if self.stl_viewer_widget:
            self.stl_viewer_widget.clear_all_frames()
        
        # Display plane visualization in STL viewer
        if self.stl_viewer_widget and plane_obj:
            plane_data = {}
            all_points = []  # For normal calculation
            
            # Case 1: Plane defined by three points
            if plane_obj.axis_names[0] == '' and plane_obj.point_names[0] != '':
                # Get the three frames
                points = []
                for point_name in plane_obj.point_names:
                    if point_name != '':
                        frame = self.anylzer.get_ref_frame_by_name(point_name)
                        if frame and frame.pose:
                            points.append(frame.pose.position)
                            all_points.append(frame.pose.position)
                
                if len(points) >= 3:
                    plane_data["points"] = points
            
            # Case 2: Plane defined by axis and one point
            elif plane_obj.axis_names[0] != '' and plane_obj.point_names[0] != '':
                # Get the two axis points
                axis_frames = self.anylzer.get_frames_for_axis(plane_obj.axis_names[0])
                if len(axis_frames) >= 2 and axis_frames[0].pose and axis_frames[1].pose:
                    plane_data["axis"] = [axis_frames[0].pose.position, axis_frames[1].pose.position]
                    all_points.append(axis_frames[0].pose.position)
                    all_points.append(axis_frames[1].pose.position)
                
                # Get the support point
                support_frame = self.anylzer.get_ref_frame_by_name(plane_obj.point_names[0])
                if support_frame and support_frame.pose:
                    plane_data["points"] = [support_frame.pose.position]
                    all_points.append(support_frame.pose.position)
            
            # Display the plane if we have the required data
            if plane_data:
                self.stl_viewer_widget.display_plane(
                    full_plane_name,
                    plane_data,
                    color_rgb=(0, 1, 1),  # Cyan
                    alpha=0.3
                )
                
                # Calculate and display plane normal
                if len(all_points) >= 3:
                    import numpy as np
                    # Get three points
                    p1 = np.array([all_points[0].x, all_points[0].y, all_points[0].z])
                    p2 = np.array([all_points[1].x, all_points[1].y, all_points[1].z])
                    p3 = np.array([all_points[2].x, all_points[2].y, all_points[2].z])
                    
                    # Calculate vectors
                    v1 = p2 - p1
                    v2 = p3 - p1
                    
                    # Calculate normal (cross product)
                    normal = np.cross(v1, v2)
                    normal_length = np.linalg.norm(normal)
                    
                    if normal_length > 1e-6:  # Check if not degenerate
                        normal = normal / normal_length  # Normalize
                        
                        # Center of plane (centroid of all points)
                        center = np.mean([p1, p2, p3], axis=0)
                        from geometry_msgs.msg import Point
                        center_point = Point(x=float(center[0]), y=float(center[1]), z=float(center[2]))
                        
                        # Display normal vector
                        self.stl_viewer_widget.display_plane_normal(
                            full_plane_name + "_normal",
                            center_point,
                            normal,
                            scale=0.05,
                            color_rgb=(1, 0, 0)  # Red
                        )

        self._current_element_panel.clear_frame_highlighting()
        defining_items = self.find_defining_frames_and_axes_for_plane(full_plane_name)
        self.highlight_defining_items(defining_items)



    def find_constraining_frames(self, frame_name: str) -> list:
        """
        Find all frames that define the given frame (i.e., frames used in the clicked frame's constraints).
        Returns a list of full frame names that are referenced in the clicked frame's constraints.
        """
        defining_frames = []
        
        _frame = self.anylzer.get_ref_frame_by_name(frame_name)
        frames = self.anylzer.get_constraint_frame_names_for_frame(_frame)
        defining_frames.extend(frames)
        
        return defining_frames

    def highlight_constraining_frames(self, constraining_frames: list):
        """
        Highlight frames in the frames list that are constraining the selected frame.
        """
        if not self._current_element_panel:
            return
        
        frames_panel = self._current_element_panel.frames_panel
        obj_name = frames_panel.obj_name
        
        # Reset all items to normal formatting
        for i in range(frames_panel.count()):
            item = frames_panel.item(i)
            item.setBackground(QColor("white"))
            item.setForeground(QColor("black"))
            font = item.font()
            font.setBold(False)
            item.setFont(font)
        
        # Highlight constraining frames
        for frame_name in constraining_frames:
            # Display name without object prefix
            display_name = frame_name
            if display_name.startswith(obj_name):
                display_name = display_name[len(obj_name)+1:]
            
            # Find and highlight the item
            for i in range(frames_panel.count()):
                item = frames_panel.item(i)
                if item.text() == display_name:
                    item.setBackground(QColor("#90EE90"))  # Light green
                    item.setForeground(QColor("darkgreen"))
                    font = item.font()
                    font.setBold(True)
                    item.setFont(font)
                    break

    def find_defining_frames_and_axes_for_axis(self, axis_name: str) -> dict:
        """
        Find all frames and axes that define the given axis.
        Returns a dict with 'frames' and 'axes' lists of full names.
        """
        defining_items = {'frames': [], 'axes': []}
        
        frames = self.anylzer.get_frames_for_axis(axis_name)

        frame_names = [f.frame_name for f in frames]
        defining_items['frames'].extend(frame_names)
        
        return defining_items

    def find_defining_frames_and_axes_for_plane(self, plane_name: str) -> dict:
        """
        Find all frames and axes that define the given plane.
        Returns a dict with 'frames' and 'axes' lists of full names.
        """
        defining_items = {'frames': [], 'axes': []}
        
        frames = self.anylzer.get_frames_for_plane(plane_name)
        
        try:
            axis = self.anylzer.get_axis_name_for_plane(plane_name)
        except RefAxisNotFoundError:
            axis = None

        if axis is not None:
            defining_items['axes'].append(axis)

        frame_names = [f.frame_name for f in frames]
        defining_items['frames'].extend(frame_names)

        return defining_items

    def highlight_defining_items(self, defining_items: dict):
        """
        Highlight frames and axes in their respective lists.
        defining_items should be a dict with 'frames' and 'axes' lists.
        """
        if not self._current_element_panel:
            return
        
        frames_panel = self._current_element_panel.frames_panel
        axis_panel = self._current_element_panel.axis_panel
        obj_name = frames_panel.obj_name
        
        # Reset all items to normal formatting
        for i in range(frames_panel.count()):
            item = frames_panel.item(i)
            item.setBackground(QColor("white"))
            item.setForeground(QColor("black"))
            font = item.font()
            font.setBold(False)
            item.setFont(font)
        
        for i in range(axis_panel.count()):
            item = axis_panel.item(i)
            item.setBackground(QColor("white"))
            item.setForeground(QColor("black"))
            font = item.font()
            font.setBold(False)
            item.setFont(font)
        
        # Highlight defining frames
        for frame_name in defining_items.get('frames', []):
            display_name = frame_name
            # Strip object prefix if it exists
            if display_name.startswith(obj_name + "_"):
                display_name = display_name[len(obj_name)+1:]
            
            # Try to find and highlight the item
            found = False
            for i in range(frames_panel.count()):
                item = frames_panel.item(i)
                # Try exact match first
                if item.text() == display_name:
                    item.setBackground(QColor("#90EE90"))  # Light green
                    item.setForeground(QColor("darkgreen"))
                    font = item.font()
                    font.setBold(True)
                    item.setFont(font)
                    found = True
                    break
                # Also try matching without any prefix
                elif item.text() == frame_name:
                    item.setBackground(QColor("#90EE90"))  # Light green
                    item.setForeground(QColor("darkgreen"))
                    font = item.font()
                    font.setBold(True)
                    item.setFont(font)
                    found = True
                    break
        
        # Highlight defining axes
        for axis_name in defining_items.get('axes', []):
            display_name = axis_name
            # Strip object prefix if it exists
            if display_name.startswith(obj_name + "_"):
                display_name = display_name[len(obj_name)+1:]
            
            # Try to find and highlight the item
            found = False
            for i in range(axis_panel.count()):
                item = axis_panel.item(i)
                # Try exact match first
                if item.text() == display_name:
                    item.setBackground(QColor("#90EE90"))  # Light green
                    item.setForeground(QColor("darkgreen"))
                    font = item.font()
                    font.setBold(True)
                    item.setFont(font)
                    found = True
                    break
                # Also try matching without any prefix
                elif item.text() == axis_name:
                    item.setBackground(QColor("#90EE90"))  # Light green
                    item.setForeground(QColor("darkgreen"))
                    font = item.font()
                    font.setBold(True)
                    item.setFont(font)
                    found = True
                    break

    # ----------------------------------------------------------------------
    # Instruction selection handler
    # ----------------------------------------------------------------------
    def on_instruction_selected(self):
        item = self.instructions_panel.list_widget.currentItem()
        if item is None:
            return

        self.ros_node.get_logger().warn(f"Selected instruction: {item.text()}")

        # Deselect object list (avoid recursion)
        self.objects_panel.list_widget.blockSignals(True)
        self.objects_panel.list_widget.clearSelection()
        self.objects_panel.list_widget.blockSignals(False)

        # Switch UI to instruction view (middle: instruction properties, right: two STL viewers)
        try:
            self.middle_stack.setCurrentIndex(1)
            self.right_stack.setCurrentIndex(1)
        except Exception:
            pass

        # Clear middle panel content (object details), keep PropertiesPanel for frame/axis/plane
        self.clear_middle_panel()
        self.properties_panel.clear()

        item_text = item.text()
        try:
            inst: am_msgs.AssemblyInstruction = self.anylzer.get_assembly_instruction_by_name(item_text)
        except Exception as e:
            self.ros_node.get_logger().error(f"Failed to get instruction '{item_text}': {e}")
            return

        # Update middle instruction properties panel
        try:
            self.instruction_properties_panel.instruction = inst
            self.instruction_properties_panel.analyzer = self.anylzer
            self.instruction_properties_panel._display_instruction_properties()
        except Exception as e:
            self.ros_node.get_logger().error(f"Error updating instruction properties panel: {e}")

        # Update right-side STL viewers
        try:
            self.instruction_stl_panel.set_instruction(inst, self.anylzer)
        except Exception as e:
            self.ros_node.get_logger().warn(f"Error updating instruction STL panel: {e}")

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
        
        # Update panels but preserve visualizations
        # If we have a current element panel, update its scene data without clearing visualizations
        if self._current_element_panel is not None:
            self._current_element_panel.set_scene(self._assembly_scene)
        
        self.objects_panel.update_scene(self._assembly_scene)
        self.instructions_panel.update_scene(self._assembly_scene)


    # ----------------------------------------------------------------------
    # Update objects and instructions panels
    # ----------------------------------------------------------------------
    def update_panels(self):
        self.objects_panel.update_scene(self._assembly_scene)
        if self._current_element_panel is not None:
            self._restore_element_selection(self._current_element_panel)
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


    # ----------------------------------------------------------------------
    # Restore element selection (frame/axis/plane) after panel recreation
    # ----------------------------------------------------------------------
    def _restore_element_selection(self, element_panel: ObjectsElementPanel):
        """Restore previously selected frame, axis, or plane in the new panel"""
        # Restore selected frame
        if self._current_selected_frame:
            #self.ros_node.get_logger().warn(f"Restoring selected frame: {self._current_selected_frame}")
            for i in range(element_panel.frames_panel.count()):
                item = element_panel.frames_panel.item(i)
                if item.text() == self._current_selected_frame:
                    element_panel.frames_panel.setCurrentItem(item)
                    return
            
        
        # Restore selected axis
        if self._current_selected_axis:
            #self.ros_node.get_logger().warn(f"Restoring selected axis: {self._current_selected_axis}")
            for i in range(element_panel.axis_panel.count()):
                item = element_panel.axis_panel.item(i)
                if item.text() == self._current_selected_axis:
                    element_panel.axis_panel.setCurrentItem(item)
                    return
        
        # Restore selected plane
        if self._current_selected_plane:
            #self.ros_node.get_logger().warn(f"Restoring selected plane: {self._current_selected_plane}")
            for i in range(element_panel.planes_panel.count()):
                item = element_panel.planes_panel.item(i)
                if item.text() == self._current_selected_plane:
                    element_panel.planes_panel.setCurrentItem(item)
                    return

