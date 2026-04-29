import json
from pathlib import Path
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
    refFrame: str = ""
    transform: Dict[str, Any] = field(default_factory=lambda: {
        "translation": {"X": 0.0, "Y": 0.0, "Z": 0.0},
        "rotation": {"X": 0.0, "Y": 0.0, "Z": 0.0, "W": 1.0}
    })
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'TransformConstraint':
        return cls(
            refFrame=data.get('refFrame', ''),
            transform=data.get('transform', {
                "translation": {"X": 0.0, "Y": 0.0, "Z": 0.0},
                "rotation": {"X": 0.0, "Y": 0.0, "Z": 0.0, "W": 1.0}
            })
        )
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            'refFrame': self.refFrame,
            'transform': self.transform
        }


@dataclass
class RefFrameConstraints:
    """Container for all constraints of a reference frame"""
    centroid: CentroidConstraint = field(default_factory=CentroidConstraint)
    orthogonal: OrthogonalConstraint = field(default_factory=OrthogonalConstraint)
    inPlane: InPlaneConstraint = field(default_factory=InPlaneConstraint)
    transform: Optional[TransformConstraint] = None
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'RefFrameConstraints':
        return cls(
            centroid=CentroidConstraint.from_dict(data.get('centroid', {})),
            orthogonal=OrthogonalConstraint.from_dict(data.get('orthogonal', {})),
            inPlane=InPlaneConstraint.from_dict(data.get('inPlane', {})),
            transform=TransformConstraint.from_dict(data.get('transform', {})) 
                if data.get('transform') else None
        )
    
    def to_dict(self) -> Dict[str, Any]:
        result = {
            'centroid': self.centroid.to_dict(),
            'orthogonal': self.orthogonal.to_dict(),
            'inPlane': self.inPlane.to_dict(),
        }
        if self.transform:
            result['transform'] = self.transform.to_dict()
        return result


@dataclass
class RefFrame:
    """Represents a reference frame with its properties and constraints"""
    name: str = ""
    type: str = ""  # "point" (position only) or "frame" (position + orientation)
    transformation: Dict[str, Any] = field(default_factory=lambda: {
        "translation": {"X": 0.0, "Y": 0.0, "Z": 0.0},
        "rotation": {"X": 0.0, "Y": 0.0, "Z": 0.0, "W": 1.0, "IsIdentity": True}
    })
    constraints: RefFrameConstraints = field(default_factory=RefFrameConstraints)
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'RefFrame':
        constraints_data = data.get('constraints', {})
        # Determine if this is a point or frame based on type field
        frame_type = data.get('type', 'frame')
        
        # All frames have the same transformation structure with translation and rotation
        default_transform = {
            "translation": {"X": 0.0, "Y": 0.0, "Z": 0.0},
            "rotation": {"X": 0.0, "Y": 0.0, "Z": 0.0, "W": 1.0, "IsIdentity": True}
        }
        
        return cls(
            name=data.get('name', ''),
            type=frame_type,
            transformation=data.get('transformation', default_transform),
            constraints=RefFrameConstraints.from_dict(constraints_data)
        )
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            'name': self.name,
            'type': self.type,
            'transformation': self.transformation,
            'constraints': self.constraints.to_dict()
        }
    
    def is_point(self) -> bool:
        """Check if this is a point (position only)"""
        return self.type == "point"
    
    def is_frame(self) -> bool:
        """Check if this is a frame (with orientation)"""
        return self.type == "frame"
    
    def has_constraints(self) -> bool:
        """Check if frame has any constraints defined"""
        # Check centroid constraint
        if self.constraints.centroid.refFrameNames:
            return True
        
        # Check orthogonal constraint
        if any([self.constraints.orthogonal.frame_1, 
                self.constraints.orthogonal.frame_2, 
                self.constraints.orthogonal.frame_3]):
            return True
        
        # Check in-plane constraint
        if self.constraints.inPlane.refFrameNames:
            return True
        
        # Check transform constraint
        if self.constraints.transform and self.constraints.transform.refFrame:
            return True
        
        return False


@dataclass
class ComponentDescriptionModifier:
    """Interface to the component description JSON data"""
    
    # Raw JSON data structure
    _data: Dict[str, Any] = field(default_factory=dict)
    _file_path: Optional[Path] = None
    _ref_frames: List[RefFrame] = field(default_factory=list)
    _spawning_transformation: Dict[str, Any] = field(default_factory=lambda: {
        'translation': {'X': 0.0, 'Y': 0.0, 'Z': 0.0},
        'rotation': {'X': 0.0, 'Y': 0.0, 'Z': 0.0, 'W': 1.0, 'IsIdentity': True}
    })
    
    @classmethod
    def from_file(cls, file_path: str) -> 'ComponentDescriptionModifier':
        """Load component description from JSON file"""
        instance = cls()
        instance._file_path = Path(file_path)
        
        with open(instance._file_path, 'r') as f:
            instance._data = json.load(f)
        
        instance._load_from_data()
        return instance
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'ComponentDescriptionModifier':
        """Create instance from dictionary"""
        instance = cls(_data=data)
        instance._load_from_data()
        return instance
    
    @classmethod
    def create_new(cls, name: str = "New Component") -> 'ComponentDescriptionModifier':
        """Create a new empty component description
        
        Args:
            name: Name of the new component
            
        Returns:
            New ComponentDescriptionModifier instance with empty structure
        """
        import uuid
        from datetime import datetime
        
        data = {
            "mountingDescription": {
                "components": [],
                "assemblyConstraints": [],
                "mountingReferences": {
                    "spawningOrigin": "",
                    "spawningTransformation": {
                        "translation": {"X": 0.0, "Y": 0.0, "Z": 0.0},
                        "rotation": {"X": 0.0, "Y": 0.0, "Z": 0.0, "W": 1.0, "IsIdentity": True}
                    },
                    "ref_planes": [],
                    "ref_axes": [],
                    "ref_frames": []
                }
            },
            "documentUnits": "mm",
            "cadPath": "",
            "cadPathCollision": "",
            "name": name,
            "description": "",
            "guid": str(uuid.uuid4()),
            "type": "Component",
            "saveDate": datetime.now().isoformat()
        }
        
        instance = cls(_data=data)
        instance._load_from_data()
        return instance
    
    def _load_from_data(self):
        """Load spawning transformation and reference frames from raw data"""
        # Load spawning transformation
        mounting_desc = self._data.get('mountingDescription', {})
        mounting_refs = mounting_desc.get('mountingReferences', {})
        self._spawning_transformation = mounting_refs.get('spawningTransformation', {
            'translation': {'X': 0.0, 'Y': 0.0, 'Z': 0.0},
            'rotation': {'X': 0.0, 'Y': 0.0, 'Z': 0.0, 'W': 1.0, 'IsIdentity': True}
        })
        
        # Load reference frames
        raw_ref_frames = mounting_refs.get('ref_frames', [])
        self._ref_frames = [RefFrame.from_dict(frame) for frame in raw_ref_frames]
    
    def get_spawning_transformation(self) -> Dict[str, Any]:
        """Get the spawning transformation"""
        return self._spawning_transformation
    
    def set_spawning_transformation(self, transformation: Dict[str, Any]):
        """Set the spawning transformation"""
        self._spawning_transformation = transformation
        self._sync_to_data()
    
    # Component metadata methods
    def get_name(self) -> str:
        """Get component name"""
        return self._data.get('name', '')
    
    def set_name(self, name: str):
        """Set component name"""
        self._data['name'] = name
    
    def get_description(self) -> str:
        """Get component description"""
        return self._data.get('description', '')
    
    def set_description(self, description: str):
        """Set component description"""
        self._data['description'] = description
    
    def get_cad_path(self) -> str:
        """Get CAD model path"""
        return self._data.get('cadPath', '')
    
    def set_cad_path(self, path: str):
        """Set CAD model path"""
        self._data['cadPath'] = path
    
    def get_cad_path_collision(self) -> str:
        """Get CAD collision model path"""
        return self._data.get('cadPathCollision', '')
    
    def set_cad_path_collision(self, path: str):
        """Set CAD collision model path"""
        self._data['cadPathCollision'] = path
    
    def get_document_units(self) -> str:
        """Get document units"""
        return self._data.get('documentUnits', 'mm')
    
    def set_document_units(self, units: str):
        """Set document units"""
        self._data['documentUnits'] = units
    
    def get_spawning_origin(self) -> str:
        """Get spawning origin reference frame name"""
        mounting_desc = self._data.get('mountingDescription', {})
        mounting_refs = mounting_desc.get('mountingReferences', {})
        return mounting_refs.get('spawningOrigin', '')
    
    def set_spawning_origin(self, origin: str):
        """Set spawning origin reference frame name"""
        if 'mountingDescription' not in self._data:
            self._data['mountingDescription'] = {}
        if 'mountingReferences' not in self._data['mountingDescription']:
            self._data['mountingDescription']['mountingReferences'] = {}
        self._data['mountingDescription']['mountingReferences']['spawningOrigin'] = origin
    
    def get_ref_frames(self) -> List[RefFrame]:
        """Get all reference frames"""
        return self._ref_frames
    
    def get_ref_frame_names(self) -> List[str]:
        """Get names of all reference frames"""
        return [frame.name for frame in self._ref_frames]
    
    def get_ref_frame(self, name: str) -> Optional[RefFrame]:
        """Get a specific reference frame by name"""
        for frame in self._ref_frames:
            if frame.name == name:
                return frame
        return None
    
    def update_ref_frame(self, name: str, frame: RefFrame):
        """Update a reference frame"""
        for i, f in enumerate(self._ref_frames):
            if f.name == name:
                self._ref_frames[i] = frame
                self._sync_to_data()
                break
    
    def add_ref_frame(self, name: str, frame_type: str = "frame") -> RefFrame:
        """Add a new reference frame
        
        Args:
            name: Name of the new frame
            frame_type: Type of the frame - "point" (position only) or "frame" (position + orientation)
                        Note: All frames store full transformation with rotation in JSON, type is metadata
            
        Returns:
            The newly created RefFrame
        """
        # Check if frame with this name already exists
        if self.get_ref_frame(name):
            raise ValueError(f"Reference frame '{name}' already exists")
        
        # Validate frame_type
        if frame_type not in ("point", "frame"):
            raise ValueError(f"frame_type must be 'point' or 'frame', got '{frame_type}'")
        
        # All frames have complete transformation with translation and rotation
        transformation = {
            "translation": {"X": 0.0, "Y": 0.0, "Z": 0.0},
            "rotation": {"X": 0.0, "Y": 0.0, "Z": 0.0, "W": 1.0, "IsIdentity": True}
        }
        
        # Create new frame with default values
        new_frame = RefFrame(
            name=name,
            type=frame_type,
            transformation=transformation,
            constraints=RefFrameConstraints()
        )
        
        self._ref_frames.append(new_frame)
        self._sync_to_data()
        return new_frame
    
    def remove_ref_frame(self, name: str) -> bool:
        """Remove a reference frame by name
        
        Args:
            name: Name of the frame to remove
            
        Returns:
            True if frame was removed, False if not found
        """
        for i, f in enumerate(self._ref_frames):
            if f.name == name:
                self._ref_frames.pop(i)
                self._sync_to_data()
                return True
        return False
    
    def get_frame_constraints(self, frame_name: str) -> Optional[RefFrameConstraints]:
        """Get constraints for a specific frame"""
        frame = self.get_ref_frame(frame_name)
        return frame.constraints if frame else None
    
    def update_frame_constraint(self, frame_name: str, constraint_type: str, 
                               constraint_data: Dict[str, Any]):
        """Update a specific constraint for a frame
        
        Args:
            frame_name: Name of the reference frame
            constraint_type: Type of constraint ('centroid', 'orthogonal', 'inPlane', 'transform')
            constraint_data: The constraint data
        """
        frame = self.get_ref_frame(frame_name)
        if not frame:
            return
        
        if constraint_type == 'centroid':
            frame.constraints.centroid = CentroidConstraint.from_dict(constraint_data)
        elif constraint_type == 'orthogonal':
            frame.constraints.orthogonal = OrthogonalConstraint.from_dict(constraint_data)
        elif constraint_type == 'inPlane':
            frame.constraints.inPlane = InPlaneConstraint.from_dict(constraint_data)
        elif constraint_type == 'transform':
            frame.constraints.transform = TransformConstraint.from_dict(constraint_data)
        
        self.update_ref_frame(frame_name, frame)
    
    def _sync_to_data(self):
        """Synchronize internal dataclass state back to raw JSON data"""
        if 'mountingDescription' not in self._data:
            self._data['mountingDescription'] = {}
        if 'mountingReferences' not in self._data['mountingDescription']:
            self._data['mountingDescription']['mountingReferences'] = {}
        
        self._data['mountingDescription']['mountingReferences']['spawningTransformation'] = self._spawning_transformation
        self._data['mountingDescription']['mountingReferences']['ref_frames'] = [
            frame.to_dict() for frame in self._ref_frames
        ]
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary representation"""
        self._sync_to_data()
        return self._data
    
    def save_to_file(self, file_path: Optional[str] = None) -> Path:
        """Save to JSON file"""
        if file_path:
            self._file_path = Path(file_path)
        
        if not self._file_path:
            raise ValueError("No file path specified")
        
        self._sync_to_data()
        with open(self._file_path, 'w') as f:
            json.dump(self._data, f, indent=2)
        
        return self._file_path
    
    def reload_from_file(self):
        """Reload data from current file path"""
        if not self._file_path:
            raise ValueError("No file path set")
        
        with open(self._file_path, 'r') as f:
            self._data = json.load(f)
        
        self._load_from_data()
    
    def get_file_path(self) -> Optional[Path]:
        """Get current file path"""
        return self._file_path
    
    def set_file_path(self, file_path: str):
        """Set current file path"""
        self._file_path = Path(file_path)
