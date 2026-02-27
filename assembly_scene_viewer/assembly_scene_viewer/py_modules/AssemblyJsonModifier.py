import rclpy
from rclpy.node import Node
from PyQt6.QtWidgets import QApplication
import sys
from rclpy.executors import MultiThreadedExecutor
from threading import Thread 
import json
from pathlib import Path


from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QThread, pyqtSlot, QRunnable, QObject, QThreadPool, QSize, QMimeData
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
from PyQt6.QtGui import QColor, QTextCursor, QFont, QAction, QIcon, QPainter, QPixmap, QDragEnterEvent, QDropEvent, QBrush
from functools import partial
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from geometry_msgs.msg import TransformStamped, Transform
import assembly_manager_interfaces.msg as am_msgs
import rclpy
import yaml
from dataclasses import dataclass, asdict, field
from typing import List, Dict, Any, Optional


@dataclass
class CentroidConstraint:
    """Represents a centroid constraint for a reference frame"""
    refFrameNames: List[str] = field(default_factory=list)
    dim: str = "xyz"
    offsetValues: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'CentroidConstraint':
        return cls(
            refFrameNames=data.get('refFrameNames', []),
            dim=data.get('dim', 'xyz'),
            offsetValues=data.get('offsetValues', [0.0, 0.0, 0.0])
        )
    
    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class OrthogonalConstraint:
    """Represents an orthogonal constraint for a reference frame"""
    frame_1: str = ""
    frame_2: str = ""
    frame_3: str = ""
    distance_from_f1: float = 0.0
    unit_distance_from_f1: str = "%"
    distance_from_f1_f2_connection: float = 0.0
    frame_normal_plane_axis: str = "z"
    frame_orthogonal_connection_axis: str = "x"
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'OrthogonalConstraint':
        return cls(
            frame_1=data.get('frame_1', ''),
            frame_2=data.get('frame_2', ''),
            frame_3=data.get('frame_3', ''),
            distance_from_f1=data.get('distance_from_f1', 0.0),
            unit_distance_from_f1=data.get('unit_distance_from_f1', '%'),
            distance_from_f1_f2_connection=data.get('distance_from_f1_f2_connection', 0.0),
            frame_normal_plane_axis=data.get('frame_normal_plane_axis', 'z'),
            frame_orthogonal_connection_axis=data.get('frame_orthogonal_connection_axis', 'x')
        )
    
    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class InPlaneConstraint:
    """Represents an in-plane constraint for a reference frame"""
    refFrameNames: List[str] = field(default_factory=list)
    planeOffset: float = 0.0
    normalAxis: str = "z"
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'InPlaneConstraint':
        return cls(
            refFrameNames=data.get('refFrameNames', []),
            planeOffset=data.get('planeOffset', 0.0),
            normalAxis=data.get('normalAxis', 'z')
        )
    
    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class TransformConstraint:
    """Represents a transform constraint for a reference frame"""
    frame_name: str = ""
    transform: Dict[str, Any] = field(default_factory=lambda: {
        "translation": {"X": 0.0, "Y": 0.0, "Z": 0.0},
        "rotation": {"X": 0.0, "Y": 0.0, "Z": 0.0, "W": 1.0}
    })
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'TransformConstraint':
        return cls(
            frame_name=data.get('frame_name', ''),
            transform=data.get('transform', {
                "translation": {"X": 0.0, "Y": 0.0, "Z": 0.0},
                "rotation": {"X": 0.0, "Y": 0.0, "Z": 0.0, "W": 1.0}
            })
        )
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            'frame_name': self.frame_name,
            'transform': self.transform
        }


@dataclass
class RefFrameConstraints:
    """Container for all constraints of a reference frame"""
    centroid: CentroidConstraint = field(default_factory=CentroidConstraint)
    orthogonal: OrthogonalConstraint = field(default_factory=OrthogonalConstraint)
    inPlane: InPlaneConstraint = field(default_factory=InPlaneConstraint)
    transformConstraint: Optional[TransformConstraint] = None
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'RefFrameConstraints':
        return cls(
            centroid=CentroidConstraint.from_dict(data.get('centroid', {})),
            orthogonal=OrthogonalConstraint.from_dict(data.get('orthogonal', {})),
            inPlane=InPlaneConstraint.from_dict(data.get('inPlane', {})),
            transformConstraint=TransformConstraint.from_dict(data.get('transformConstraint', {})) 
                if data.get('transformConstraint') else None
        )
    
    def to_dict(self) -> Dict[str, Any]:
        result = {
            'centroid': self.centroid.to_dict(),
            'orthogonal': self.orthogonal.to_dict(),
            'inPlane': self.inPlane.to_dict(),
        }
        if self.transformConstraint:
            result['transformConstraint'] = self.transformConstraint.to_dict()
        return result


class AssemblyModifierWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.setAcceptDrops(True)
        self.current_data = None
        self.current_frame_name = None
        self.init_ui()
    
    def init_ui(self):
        """Initialize the user interface"""
        main_layout = QHBoxLayout()
        
        # Left side layout
        left_layout = QVBoxLayout()
        
        # Drag and drop area at the top
        drop_label = QLabel("Drag and drop JSON file here")
        drop_label.setStyleSheet("""
            QLabel {
                border: 2px dashed #cccccc;
                border-radius: 5px;
                padding: 30px;
                background-color: #f5f5f5;
                text-align: center;
            }
        """)
        drop_label.setMinimumHeight(80)
        drop_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        left_layout.addWidget(drop_label)
        
        # List widget for ref_frames
        list_label = QLabel("Reference Frames:")
        left_layout.addWidget(list_label)
        self.ref_frames_list = QListWidget()
        self.ref_frames_list.itemClicked.connect(self.on_refframe_selected)
        left_layout.addWidget(self.ref_frames_list)
        
        # Right side layout for details
        right_layout = QVBoxLayout()
        right_layout.addWidget(QLabel("Frame Details:"))
        
        # Details display area
        self.details_text = QTextEdit()
        self.details_text.setReadOnly(True)
        right_layout.addWidget(self.details_text)
        
        # Constraint editing section
        constraints_label = QLabel("Edit Constraints:")
        right_layout.addWidget(constraints_label)
        
        constraints_layout = QGridLayout()
        self.edit_centroid_btn = QPushButton("Edit Centroid")
        self.edit_centroid_btn.clicked.connect(self.edit_centroid_constraint)
        constraints_layout.addWidget(self.edit_centroid_btn, 0, 0)
        
        self.edit_orthogonal_btn = QPushButton("Edit Orthogonal")
        self.edit_orthogonal_btn.clicked.connect(self.edit_orthogonal_constraint)
        constraints_layout.addWidget(self.edit_orthogonal_btn, 0, 1)
        
        self.edit_inplane_btn = QPushButton("Edit In-Plane")
        self.edit_inplane_btn.clicked.connect(self.edit_inplane_constraint)
        constraints_layout.addWidget(self.edit_inplane_btn, 1, 0)
        
        self.edit_transform_btn = QPushButton("Edit Transform")
        self.edit_transform_btn.clicked.connect(self.edit_transform_constraint)
        constraints_layout.addWidget(self.edit_transform_btn, 1, 1)
        
        right_layout.addLayout(constraints_layout)
        
        # Save button
        self.save_btn = QPushButton("Save JSON")
        self.save_btn.clicked.connect(self.save_json)
        right_layout.addWidget(self.save_btn)
        
        # Set layout proportions
        left_widget = QWidget()
        left_widget.setLayout(left_layout)
        right_widget = QWidget()
        right_widget.setLayout(right_layout)
        
        main_layout.addWidget(left_widget, 1)
        main_layout.addWidget(right_widget, 1)
        
        self.setLayout(main_layout)
        
        # Disable constraint buttons initially
        self.enable_constraint_buttons(False)
    
    def dragEnterEvent(self, event: QDragEnterEvent):
        """Handle drag enter event"""
        if event.mimeData().hasUrls():
            event.acceptProposedAction()
    
    def dropEvent(self, event: QDropEvent):
        """Handle drop event for loading JSON files"""
        for url in event.mimeData().urls():
            file_path = url.toLocalFile()
            if file_path.endswith('.json'):
                self.load_json(file_path)
                break
    
    def load_json(self, file_path):
        """Load and parse JSON file"""
        try:
            with open(file_path, 'r') as f:
                self.current_data = json.load(f)
            
            self.populate_refframes_list()
            QMessageBox.information(self, "Success", f"Loaded: {Path(file_path).name}")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to load JSON: {str(e)}")
    
    def populate_refframes_list(self):
        """Populate the list widget with reference frames"""
        self.ref_frames_list.clear()
        
        if not self.current_data:
            return
        
        try:
            # Navigate to mounting references -> ref_frames
            ref_frames = self.current_data.get('mountingDescription', {}).get('mountingReferences', {}).get('ref_frames', [])
            
            for frame in ref_frames:
                frame_name = frame.get('name', 'Unknown')
                frame_type = frame.get('type', 'unknown')
                item_text = f"{frame_name} ({frame_type})"
                
                item = QListWidgetItem(item_text)
                item.setData(Qt.ItemDataRole.UserRole, frame_name)  # Store frame name for later retrieval
                
                # Check if frame has constraints and color it green
                if self.has_constraints(frame):
                    item.setData(Qt.ItemDataRole.BackgroundRole, QBrush(QColor(144, 238, 144)))  # Light green
                    item.setData(Qt.ItemDataRole.ForegroundRole, QBrush(QColor(0, 0, 0)))  # Black text for contrast
                
                self.ref_frames_list.addItem(item)
        except Exception as e:
            QMessageBox.warning(self, "Warning", f"Error populating frames: {str(e)}")
    
    def has_constraints(self, frame: Dict[str, Any]) -> bool:
        """Check if a frame has any constraints defined"""
        constraints = frame.get('constraints', {})
        
        # Check centroid constraint
        centroid = constraints.get('centroid', {})
        if centroid.get('refFrameNames'):
            return True
        
        # Check orthogonal constraint
        orthogonal = constraints.get('orthogonal', {})
        if any([orthogonal.get('frame_1'), orthogonal.get('frame_2'), orthogonal.get('frame_3')]):
            return True
        
        # Check in-plane constraint
        inplane = constraints.get('inPlane', {})
        if inplane.get('refFrameNames'):
            return True
        
        # Check transform constraint
        transform = constraints.get('transformConstraint', {})
        if transform and transform.get('frame_name'):
            return True
        
        return False
    
    def update_frame_item_color(self):
        """Update the color of the currently selected frame item based on constraint status"""
        if not self.current_frame_name:
            return
        
        frame = self.get_current_frame()
        if not frame:
            return
        
        # Find and update the list item
        for i in range(self.ref_frames_list.count()):
            item = self.ref_frames_list.item(i)
            if item.data(Qt.ItemDataRole.UserRole) == self.current_frame_name:
                if self.has_constraints(frame):
                    item.setData(Qt.ItemDataRole.BackgroundRole, QBrush(QColor(144, 238, 144)))  # Light green
                    item.setData(Qt.ItemDataRole.ForegroundRole, QBrush(QColor(0, 0, 0)))  # Black text for contrast
                else:
                    item.setData(Qt.ItemDataRole.BackgroundRole, QBrush())  # Default color
                    item.setData(Qt.ItemDataRole.ForegroundRole, QBrush())  # Default text color
                break
    
    def on_refframe_selected(self, item):
        """Handle selection of a reference frame"""
        frame_name = item.data(Qt.ItemDataRole.UserRole)
        self.current_frame_name = frame_name
        
        try:
            ref_frames = self.current_data.get('mountingDescription', {}).get('mountingReferences', {}).get('ref_frames', [])
            
            # Find the selected frame
            selected_frame = None
            for frame in ref_frames:
                if frame.get('name') == frame_name:
                    selected_frame = frame
                    break
            
            if selected_frame:
                self.display_frame_details(selected_frame)
                self.enable_constraint_buttons(True)
        except Exception as e:
            QMessageBox.warning(self, "Error", f"Error displaying frame: {str(e)}")
    
    def enable_constraint_buttons(self, enabled: bool):
        """Enable or disable constraint editing buttons"""
        self.edit_centroid_btn.setEnabled(enabled)
        self.edit_orthogonal_btn.setEnabled(enabled)
        self.edit_inplane_btn.setEnabled(enabled)
        self.edit_transform_btn.setEnabled(enabled)
    
    def get_current_frame(self):
        """Get the currently selected frame dictionary"""
        if not self.current_frame_name or not self.current_data:
            return None
        
        ref_frames = self.current_data.get('mountingDescription', {}).get('mountingReferences', {}).get('ref_frames', [])
        for frame in ref_frames:
            if frame.get('name') == self.current_frame_name:
                return frame
        return None
    
    def edit_centroid_constraint(self):
        """Open editor for centroid constraint"""
        frame = self.get_current_frame()
        if not frame:
            QMessageBox.warning(self, "Warning", "No frame selected")
            return
        
        constraints = frame.get('constraints', {})
        centroid_data = constraints.get('centroid', {})
        
        # Get all available frame names for reference
        all_frames = self.current_data.get('mountingDescription', {}).get('mountingReferences', {}).get('ref_frames', [])
        frame_names = [f.get('name', '') for f in all_frames]
        
        dialog = CentroidConstraintDialog(centroid_data, frame_names, self)
        if dialog.exec():
            constraints['centroid'] = dialog.get_data()
            frame['constraints'] = constraints
            self.display_frame_details(frame)
            self.update_frame_item_color()
    
    def edit_orthogonal_constraint(self):
        """Open editor for orthogonal constraint"""
        frame = self.get_current_frame()
        if not frame:
            QMessageBox.warning(self, "Warning", "No frame selected")
            return
        
        constraints = frame.get('constraints', {})
        orthogonal_data = constraints.get('orthogonal', {})
        
        # Get all available frame names for reference
        all_frames = self.current_data.get('mountingDescription', {}).get('mountingReferences', {}).get('ref_frames', [])
        frame_names = [f.get('name', '') for f in all_frames]
        
        dialog = OrthogonalConstraintDialog(orthogonal_data, frame_names, self)
        if dialog.exec():
            constraints['orthogonal'] = dialog.get_data()
            frame['constraints'] = constraints
            self.display_frame_details(frame)
            self.update_frame_item_color()
    
    def edit_inplane_constraint(self):
        """Open editor for in-plane constraint"""
        frame = self.get_current_frame()
        if not frame:
            QMessageBox.warning(self, "Warning", "No frame selected")
            return
        
        constraints = frame.get('constraints', {})
        inplane_data = constraints.get('inPlane', {})
        
        # Get all available frame names for reference
        all_frames = self.current_data.get('mountingDescription', {}).get('mountingReferences', {}).get('ref_frames', [])
        frame_names = [f.get('name', '') for f in all_frames]
        
        dialog = InPlaneConstraintDialog(inplane_data, frame_names, self)
        if dialog.exec():
            constraints['inPlane'] = dialog.get_data()
            frame['constraints'] = constraints
            self.display_frame_details(frame)
            self.update_frame_item_color()
    
    def edit_transform_constraint(self):
        """Open editor for transform constraint"""
        frame = self.get_current_frame()
        if not frame:
            QMessageBox.warning(self, "Warning", "No frame selected")
            return
        
        constraints = frame.get('constraints', {})
        transform_data = constraints.get('transformConstraint', {})
        
        # Get all available frame names for reference
        all_frames = self.current_data.get('mountingDescription', {}).get('mountingReferences', {}).get('ref_frames', [])
        frame_names = [f.get('name', '') for f in all_frames]
        
        dialog = TransformConstraintDialog(transform_data, frame_names, self)
        if dialog.exec():
            constraints['transformConstraint'] = dialog.get_data()
            frame['constraints'] = constraints
            self.display_frame_details(frame)
            self.update_frame_item_color()
    
    def save_json(self):
        """Save the current data to a JSON file with confirmation dialog"""
        if not self.current_data:
            QMessageBox.warning(self, "Warning", "No data loaded")
            return
        
        # Show confirmation dialog
        reply = QMessageBox.question(
            self,
            "Save Changes",
            "Do you want to save all changes to the JSON file?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.Yes
        )
        
        if reply == QMessageBox.StandardButton.No:
            return
        
        file_path, _ = QFileDialog.getSaveFileName(
            self,
            "Save JSON File",
            "",
            "JSON Files (*.json);;All Files (*)"
        )
        
        if file_path:
            try:
                with open(file_path, 'w') as f:
                    json.dump(self.current_data, f, indent=2)
                QMessageBox.information(self, "Success", "File saved successfully")
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to save: {str(e)}")
    
    def display_frame_details(self, frame):
        """Display detailed information about a reference frame"""
        details = []
        details.append(f"<b>Name:</b> {frame.get('name', 'N/A')}")
        details.append(f"<b>Type:</b> {frame.get('type', 'N/A')}")
        
        # Translation
        translation = frame.get('transformation', {}).get('translation', {})
        details.append(f"<b>Translation:</b>")
        details.append(f"  X: {translation.get('X', 'N/A')}")
        details.append(f"  Y: {translation.get('Y', 'N/A')}")
        details.append(f"  Z: {translation.get('Z', 'N/A')}")
        
        # Rotation
        rotation = frame.get('transformation', {}).get('rotation', {})
        details.append(f"<b>Rotation (Quaternion):</b>")
        details.append(f"  X: {rotation.get('X', 'N/A')}")
        details.append(f"  Y: {rotation.get('Y', 'N/A')}")
        details.append(f"  Z: {rotation.get('Z', 'N/A')}")
        details.append(f"  W: {rotation.get('W', 'N/A')}")
        details.append(f"  IsIdentity: {rotation.get('IsIdentity', 'N/A')}")
        
        # Constraints
        constraints = frame.get('constraints', {})
        details.append(f"<b>Constraints:</b>")
        
        # Centroid constraint
        centroid = constraints.get('centroid', {})
        details.append(f"  <b>Centroid:</b>")
        details.append(f"    Ref Frames: {', '.join(centroid.get('refFrameNames', []))}")
        details.append(f"    Dimension: {centroid.get('dim', 'N/A')}")
        details.append(f"    Offset: {centroid.get('offsetValues', [])}")
        
        # Orthogonal constraint
        orthogonal = constraints.get('orthogonal', {})
        if any(orthogonal.values()):
            details.append(f"  <b>Orthogonal:</b>")
            details.append(f"    Frame 1: {orthogonal.get('frame_1', 'N/A')}")
            details.append(f"    Frame 2: {orthogonal.get('frame_2', 'N/A')}")
            details.append(f"    Frame 3: {orthogonal.get('frame_3', 'N/A')}")
            details.append(f"    Distance from F1: {orthogonal.get('distance_from_f1', 'N/A')} {orthogonal.get('unit_distance_from_f1', '')}")
        
        # In-plane constraint
        in_plane = constraints.get('inPlane', {})
        if any(in_plane.values()):
            details.append(f"  <b>In Plane:</b>")
            details.append(f"    Ref Frames: {', '.join(in_plane.get('refFrameNames', []))}")
            details.append(f"    Plane Offset: {in_plane.get('planeOffset', 'N/A')}")
            details.append(f"    Normal Axis: {in_plane.get('normalAxis', 'N/A')}")
        
        # Transform constraint
        transform_constraint = constraints.get('transformConstraint', {})
        if transform_constraint:
            details.append(f"  <b>Transform Constraint:</b>")
            details.append(f"    Frame Name: {transform_constraint.get('frame_name', 'N/A')}")
            transform_data = transform_constraint.get('transform', {})
            translation = transform_data.get('translation', {})
            rotation = transform_data.get('rotation', {})
            details.append(f"    Translation: X={translation.get('X', 'N/A')}, Y={translation.get('Y', 'N/A')}, Z={translation.get('Z', 'N/A')}")
            details.append(f"    Rotation: X={rotation.get('X', 'N/A')}, Y={rotation.get('Y', 'N/A')}, Z={rotation.get('Z', 'N/A')}, W={rotation.get('W', 'N/A')}")
        
        html_content = "<br>".join(details)
        self.details_text.setHtml(html_content)


class CentroidConstraintDialog(QDialog):
    """Dialog for editing centroid constraints"""
    def __init__(self, data: Dict[str, Any], frame_names: List[str], parent=None):
        super().__init__(parent)
        self.setWindowTitle("Edit Centroid Constraint")
        self.setModal(True)
        self.setGeometry(100, 100, 700, 500)
        self.data = data
        self.frame_names = frame_names
        self.selected_frames = data.get('refFrameNames', [])
        self.init_ui()
    
    def init_ui(self):
        layout = QVBoxLayout()
        
        # Reference frames selector
        layout.addWidget(QLabel("<b>Select Reference Frames:</b>"))
        
        # Dual list layout
        frames_layout = QHBoxLayout()
        
        # Available frames
        frames_layout.addWidget(QLabel("Available Frames:"))
        self.available_list = QListWidget()
        self.available_list.setMinimumSize(200, 150)
        available_frames = [f for f in self.frame_names if f not in self.selected_frames]
        self.available_list.addItems(available_frames)
        frames_layout.addWidget(self.available_list)
        
        # Add/Remove buttons
        button_column = QVBoxLayout()
        button_column.addStretch()
        
        add_btn = QPushButton("Add >>>")
        add_btn.clicked.connect(self.add_frame)
        button_column.addWidget(add_btn)
        
        remove_btn = QPushButton("<<< Remove")
        remove_btn.clicked.connect(self.remove_frame)
        button_column.addWidget(remove_btn)
        
        button_column.addStretch()
        frames_layout.addLayout(button_column)
        
        # Selected frames
        frames_layout.addWidget(QLabel("Selected Frames:"))
        self.selected_list = QListWidget()
        self.selected_list.setMinimumSize(200, 150)
        self.selected_list.addItems(self.selected_frames)
        frames_layout.addWidget(self.selected_list)
        
        layout.addLayout(frames_layout)
        
        # Dimension input
        layout.addWidget(QLabel("Dimension:"))
        self.dim_combo = QComboBox()
        self.dim_combo.addItems(["xyz", "xy", "xz", "yz", "x", "y", "z"])
        self.dim_combo.setCurrentText(self.data.get('dim', 'xyz'))
        layout.addWidget(self.dim_combo)
        
        # Offset values
        layout.addWidget(QLabel("Offset Values (X, Y, Z):"))
        offsets = self.data.get('offsetValues', [0.0, 0.0, 0.0])
        
        self.offset_x = QDoubleSpinBox()
        self.offset_x.setRange(-1000, 1000)
        self.offset_x.setValue(offsets[0] if len(offsets) > 0 else 0.0)
        layout.addWidget(QLabel("X:"))
        layout.addWidget(self.offset_x)
        
        self.offset_y = QDoubleSpinBox()
        self.offset_y.setRange(-1000, 1000)
        self.offset_y.setValue(offsets[1] if len(offsets) > 1 else 0.0)
        layout.addWidget(QLabel("Y:"))
        layout.addWidget(self.offset_y)
        
        self.offset_z = QDoubleSpinBox()
        self.offset_z.setRange(-1000, 1000)
        self.offset_z.setValue(offsets[2] if len(offsets) > 2 else 0.0)
        layout.addWidget(QLabel("Z:"))
        layout.addWidget(self.offset_z)
        
        # Buttons
        button_layout = QHBoxLayout()
        clear_btn = QPushButton("Clear")
        clear_btn.clicked.connect(self.clear_constraint)
        ok_btn = QPushButton("OK")
        ok_btn.clicked.connect(self.accept)
        cancel_btn = QPushButton("Cancel")
        cancel_btn.clicked.connect(self.reject)
        button_layout.addWidget(clear_btn)
        button_layout.addWidget(ok_btn)
        button_layout.addWidget(cancel_btn)
        layout.addLayout(button_layout)
        
        self.setLayout(layout)
    
    def add_frame(self):
        """Move selected item from available to selected list"""
        current_row = self.available_list.currentRow()
        if current_row >= 0:
            item = self.available_list.takeItem(current_row)
            self.selected_list.addItem(item)
    
    def remove_frame(self):
        """Move selected item from selected to available list"""
        current_row = self.selected_list.currentRow()
        if current_row >= 0:
            item = self.selected_list.takeItem(current_row)
            self.available_list.addItem(item)
    
    def clear_constraint(self):
        """Reset constraint to default values"""
        while self.selected_list.count() > 0:
            item = self.selected_list.takeItem(0)
            self.available_list.addItem(item)
        self.dim_combo.setCurrentText("xyz")
        self.offset_x.setValue(0.0)
        self.offset_y.setValue(0.0)
        self.offset_z.setValue(0.0)
    
    def get_data(self) -> Dict[str, Any]:
        ref_frames = [self.selected_list.item(i).text() for i in range(self.selected_list.count())]
        return {
            'refFrameNames': ref_frames,
            'dim': self.dim_combo.currentText(),
            'offsetValues': [self.offset_x.value(), self.offset_y.value(), self.offset_z.value()]
        }


class OrthogonalConstraintDialog(QDialog):
    """Dialog for editing orthogonal constraints"""
    def __init__(self, data: Dict[str, Any], frame_names: List[str], parent=None):
        super().__init__(parent)
        self.setWindowTitle("Edit Orthogonal Constraint")
        self.setModal(True)
        self.setGeometry(100, 100, 500, 500)
        self.data = data
        self.frame_names = frame_names
        self.init_ui()
    
    def init_ui(self):
        layout = QVBoxLayout()
        
        # Frame 1
        layout.addWidget(QLabel("Frame 1:"))
        self.frame_1_combo = QComboBox()
        self.frame_1_combo.addItems([""] + self.frame_names)
        self.frame_1_combo.setCurrentText(self.data.get('frame_1', ''))
        layout.addWidget(self.frame_1_combo)
        
        # Frame 2
        layout.addWidget(QLabel("Frame 2:"))
        self.frame_2_combo = QComboBox()
        self.frame_2_combo.addItems([""] + self.frame_names)
        self.frame_2_combo.setCurrentText(self.data.get('frame_2', ''))
        layout.addWidget(self.frame_2_combo)
        
        # Frame 3
        layout.addWidget(QLabel("Frame 3:"))
        self.frame_3_combo = QComboBox()
        self.frame_3_combo.addItems([""] + self.frame_names)
        self.frame_3_combo.setCurrentText(self.data.get('frame_3', ''))
        layout.addWidget(self.frame_3_combo)
        
        # Distance from F1
        layout.addWidget(QLabel("Distance from Frame 1:"))
        self.distance_spin = QDoubleSpinBox()
        self.distance_spin.setRange(-10000, 10000)
        self.distance_spin.setValue(self.data.get('distance_from_f1', 0.0))
        layout.addWidget(self.distance_spin)
        
        # Unit
        layout.addWidget(QLabel("Unit:"))
        self.unit_combo = QComboBox()
        self.unit_combo.addItems(["%", "mm", "m"])
        self.unit_combo.setCurrentText(self.data.get('unit_distance_from_f1', '%'))
        layout.addWidget(self.unit_combo)
        
        # Distance F1-F2 connection
        layout.addWidget(QLabel("Distance F1-F2 Connection:"))
        self.distance_f1_f2_spin = QDoubleSpinBox()
        self.distance_f1_f2_spin.setRange(-1000, 1000)
        self.distance_f1_f2_spin.setValue(self.data.get('distance_from_f1_f2_connection', 0.0))
        layout.addWidget(self.distance_f1_f2_spin)
        
        # Frame normal plane axis
        layout.addWidget(QLabel("Frame Normal Plane Axis:"))
        self.normal_axis_combo = QComboBox()
        self.normal_axis_combo.addItems(["x", "y", "z"])
        self.normal_axis_combo.setCurrentText(self.data.get('frame_normal_plane_axis', 'z'))
        layout.addWidget(self.normal_axis_combo)
        
        # Frame orthogonal connection axis
        layout.addWidget(QLabel("Frame Orthogonal Connection Axis:"))
        self.connection_axis_combo = QComboBox()
        self.connection_axis_combo.addItems(["x", "y", "z"])
        self.connection_axis_combo.setCurrentText(self.data.get('frame_orthogonal_connection_axis', 'x'))
        layout.addWidget(self.connection_axis_combo)
        
        # Buttons
        button_layout = QHBoxLayout()
        clear_btn = QPushButton("Clear")
        clear_btn.clicked.connect(self.clear_constraint)
        ok_btn = QPushButton("OK")
        ok_btn.clicked.connect(self.accept)
        cancel_btn = QPushButton("Cancel")
        cancel_btn.clicked.connect(self.reject)
        button_layout.addWidget(clear_btn)
        button_layout.addWidget(ok_btn)
        button_layout.addWidget(cancel_btn)
        layout.addLayout(button_layout)
        
        self.setLayout(layout)
    
    def clear_constraint(self):
        """Reset constraint to default values"""
        self.frame_1_combo.setCurrentText("")
        self.frame_2_combo.setCurrentText("")
        self.frame_3_combo.setCurrentText("")
        self.distance_spin.setValue(0.0)
        self.unit_combo.setCurrentText("%")
        self.distance_f1_f2_spin.setValue(0.0)
        self.normal_axis_combo.setCurrentText("z")
        self.connection_axis_combo.setCurrentText("x")
    
    def get_data(self) -> Dict[str, Any]:
        return {
            'frame_1': self.frame_1_combo.currentText(),
            'frame_2': self.frame_2_combo.currentText(),
            'frame_3': self.frame_3_combo.currentText(),
            'distance_from_f1': self.distance_spin.value(),
            'unit_distance_from_f1': self.unit_combo.currentText(),
            'distance_from_f1_f2_connection': self.distance_f1_f2_spin.value(),
            'frame_normal_plane_axis': self.normal_axis_combo.currentText(),
            'frame_orthogonal_connection_axis': self.connection_axis_combo.currentText()
        }


class InPlaneConstraintDialog(QDialog):
    """Dialog for editing in-plane constraints"""
    def __init__(self, data: Dict[str, Any], frame_names: List[str], parent=None):
        super().__init__(parent)
        self.setWindowTitle("Edit In-Plane Constraint")
        self.setModal(True)
        self.setGeometry(100, 100, 700, 450)
        self.data = data
        self.frame_names = frame_names
        self.selected_frames = data.get('refFrameNames', [])
        self.init_ui()
    
    def init_ui(self):
        layout = QVBoxLayout()
        
        # Reference frames selector
        layout.addWidget(QLabel("<b>Select Reference Frames:</b>"))
        
        # Dual list layout
        frames_layout = QHBoxLayout()
        
        # Available frames
        frames_layout.addWidget(QLabel("Available Frames:"))
        self.available_list = QListWidget()
        self.available_list.setMinimumSize(200, 150)
        available_frames = [f for f in self.frame_names if f not in self.selected_frames]
        self.available_list.addItems(available_frames)
        frames_layout.addWidget(self.available_list)
        
        # Add/Remove buttons
        button_column = QVBoxLayout()
        button_column.addStretch()
        
        add_btn = QPushButton("Add >>>")
        add_btn.clicked.connect(self.add_frame)
        button_column.addWidget(add_btn)
        
        remove_btn = QPushButton("<<< Remove")
        remove_btn.clicked.connect(self.remove_frame)
        button_column.addWidget(remove_btn)
        
        button_column.addStretch()
        frames_layout.addLayout(button_column)
        
        # Selected frames
        frames_layout.addWidget(QLabel("Selected Frames:"))
        self.selected_list = QListWidget()
        self.selected_list.setMinimumSize(200, 150)
        self.selected_list.addItems(self.selected_frames)
        frames_layout.addWidget(self.selected_list)
        
        layout.addLayout(frames_layout)
        
        # Plane offset
        layout.addWidget(QLabel("Plane Offset:"))
        self.offset_spin = QDoubleSpinBox()
        self.offset_spin.setRange(-1000, 1000)
        self.offset_spin.setValue(self.data.get('planeOffset', 0.0))
        layout.addWidget(self.offset_spin)
        
        # Normal axis
        layout.addWidget(QLabel("Normal Axis:"))
        self.normal_axis_combo = QComboBox()
        self.normal_axis_combo.addItems(["x", "y", "z"])
        self.normal_axis_combo.setCurrentText(self.data.get('normalAxis', 'z'))
        layout.addWidget(self.normal_axis_combo)
        
        # Buttons
        button_layout = QHBoxLayout()
        clear_btn = QPushButton("Clear")
        clear_btn.clicked.connect(self.clear_constraint)
        ok_btn = QPushButton("OK")
        ok_btn.clicked.connect(self.accept)
        cancel_btn = QPushButton("Cancel")
        cancel_btn.clicked.connect(self.reject)
        button_layout.addWidget(clear_btn)
        button_layout.addWidget(ok_btn)
        button_layout.addWidget(cancel_btn)
        layout.addLayout(button_layout)
        
        self.setLayout(layout)
    
    def add_frame(self):
        """Move selected item from available to selected list"""
        current_row = self.available_list.currentRow()
        if current_row >= 0:
            item = self.available_list.takeItem(current_row)
            self.selected_list.addItem(item)
    
    def remove_frame(self):
        """Move selected item from selected to available list"""
        current_row = self.selected_list.currentRow()
        if current_row >= 0:
            item = self.selected_list.takeItem(current_row)
            self.available_list.addItem(item)
    
    def clear_constraint(self):
        """Reset constraint to default values"""
        while self.selected_list.count() > 0:
            item = self.selected_list.takeItem(0)
            self.available_list.addItem(item)
        self.offset_spin.setValue(0.0)
        self.normal_axis_combo.setCurrentText("z")
    
    def get_data(self) -> Dict[str, Any]:
        ref_frames = [self.selected_list.item(i).text() for i in range(self.selected_list.count())]
        return {
            'refFrameNames': ref_frames,
            'planeOffset': self.offset_spin.value(),
            'normalAxis': self.normal_axis_combo.currentText()
        }


class TransformConstraintDialog(QDialog):
    """Dialog for editing transform constraints"""
    def __init__(self, data: Dict[str, Any], frame_names: List[str], parent=None):
        super().__init__(parent)
        self.setWindowTitle("Edit Transform Constraint")
        self.setModal(True)
        self.setGeometry(100, 100, 500, 600)
        self.data = data
        self.frame_names = frame_names
        self.init_ui()
    
    def init_ui(self):
        layout = QVBoxLayout()
        
        # Frame name
        layout.addWidget(QLabel("Frame Name:"))
        self.frame_combo = QComboBox()
        self.frame_combo.addItems([""] + self.frame_names)
        self.frame_combo.setCurrentText(self.data.get('frame_name', ''))
        layout.addWidget(self.frame_combo)
        
        # Transform data
        transform = self.data.get('transform', {
            'translation': {'X': 0.0, 'Y': 0.0, 'Z': 0.0},
            'rotation': {'X': 0.0, 'Y': 0.0, 'Z': 0.0, 'W': 1.0}
        })
        
        translation = transform.get('translation', {'X': 0.0, 'Y': 0.0, 'Z': 0.0})
        
        # Translation
        layout.addWidget(QLabel("<b>Translation:</b>"))
        layout.addWidget(QLabel("X:"))
        self.trans_x = QDoubleSpinBox()
        self.trans_x.setRange(-10000, 10000)
        self.trans_x.setValue(translation.get('X', 0.0))
        layout.addWidget(self.trans_x)
        
        layout.addWidget(QLabel("Y:"))
        self.trans_y = QDoubleSpinBox()
        self.trans_y.setRange(-10000, 10000)
        self.trans_y.setValue(translation.get('Y', 0.0))
        layout.addWidget(self.trans_y)
        
        layout.addWidget(QLabel("Z:"))
        self.trans_z = QDoubleSpinBox()
        self.trans_z.setRange(-10000, 10000)
        self.trans_z.setValue(translation.get('Z', 0.0))
        layout.addWidget(self.trans_z)
        
        # Rotation (Quaternion)
        rotation = transform.get('rotation', {'X': 0.0, 'Y': 0.0, 'Z': 0.0, 'W': 1.0})
        
        layout.addWidget(QLabel("<b>Rotation (Quaternion):</b>"))
        layout.addWidget(QLabel("X:"))
        self.rot_x = QDoubleSpinBox()
        self.rot_x.setRange(-1, 1)
        self.rot_x.setDecimals(5)
        self.rot_x.setValue(rotation.get('X', 0.0))
        layout.addWidget(self.rot_x)
        
        layout.addWidget(QLabel("Y:"))
        self.rot_y = QDoubleSpinBox()
        self.rot_y.setRange(-1, 1)
        self.rot_y.setDecimals(5)
        self.rot_y.setValue(rotation.get('Y', 0.0))
        layout.addWidget(self.rot_y)
        
        layout.addWidget(QLabel("Z:"))
        self.rot_z = QDoubleSpinBox()
        self.rot_z.setRange(-1, 1)
        self.rot_z.setDecimals(5)
        self.rot_z.setValue(rotation.get('Z', 0.0))
        layout.addWidget(self.rot_z)
        
        layout.addWidget(QLabel("W:"))
        self.rot_w = QDoubleSpinBox()
        self.rot_w.setRange(-1, 1)
        self.rot_w.setDecimals(5)
        self.rot_w.setValue(rotation.get('W', 1.0))
        layout.addWidget(self.rot_w)
        
        # Buttons
        button_layout = QHBoxLayout()
        clear_btn = QPushButton("Clear")
        clear_btn.clicked.connect(self.clear_constraint)
        ok_btn = QPushButton("OK")
        ok_btn.clicked.connect(self.accept)
        cancel_btn = QPushButton("Cancel")
        cancel_btn.clicked.connect(self.reject)
        button_layout.addWidget(clear_btn)
        button_layout.addWidget(ok_btn)
        button_layout.addWidget(cancel_btn)
        layout.addLayout(button_layout)
        
        self.setLayout(layout)
    
    def clear_constraint(self):
        """Reset constraint to default values"""
        self.frame_combo.setCurrentText("")
        self.trans_x.setValue(0.0)
        self.trans_y.setValue(0.0)
        self.trans_z.setValue(0.0)
        self.rot_x.setValue(0.0)
        self.rot_y.setValue(0.0)
        self.rot_z.setValue(0.0)
        self.rot_w.setValue(1.0)
    
    def get_data(self) -> Dict[str, Any]:
        return {
            'frame_name': self.frame_combo.currentText(),
            'transform': {
                'translation': {
                    'X': self.trans_x.value(),
                    'Y': self.trans_y.value(),
                    'Z': self.trans_z.value()
                },
                'rotation': {
                    'X': self.rot_x.value(),
                    'Y': self.rot_y.value(),
                    'Z': self.rot_z.value(),
                    'W': self.rot_w.value()
                }
            }
        }


class AssemblyModifierMainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Assembly Scene Modifier")
        self.setGeometry(100, 100, 1200, 800)
        self._main_widget = AssemblyModifierWidget()
        self.setCentralWidget(self._main_widget)

class AssemblyJsonModifierNode(Node):

    def __init__(self):
        super().__init__('assembly_json_modifier_node')
        self.get_logger().info('Assembly Json Modifier Node has been started.')

        self.qt_window = AssemblyModifierMainWindow()
        
def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor(num_threads=6) 

    app = QApplication(sys.argv)

    assembly_json_modifier_node = AssemblyJsonModifierNode()
    executor.add_node(assembly_json_modifier_node)

    thread = Thread(target=executor.spin)
    thread.start()
    
    try:
        assembly_json_modifier_node.qt_window.show()
        sys.exit(app.exec())

    finally:
        assembly_json_modifier_node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()
    