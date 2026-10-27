"""Data model used by the assembly-description JSON editor.

The editor deliberately preserves fields it does not know.  Assembly exports are
also consumed by other tools and dropping vendor metadata while editing one
instruction would be surprising.
"""

import json
import uuid
from copy import deepcopy
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional


IDENTITY_TRANSFORM = {
    'translation': {'X': 0.0, 'Y': 0.0, 'Z': 0.0},
    'rotation': {
        'X': 0.0, 'Y': 0.0, 'Z': 0.0, 'W': 1.0, 'IsIdentity': True,
    },
}


class AssemblyDescriptionModifier:
    """Load, validate and modify an assembly description without losing data."""

    def __init__(self, data: Dict[str, Any], file_path: Optional[Path] = None):
        self._data = deepcopy(data)
        self._file_path = Path(file_path) if file_path else None
        self.validate_document(self._data)

    @classmethod
    def from_file(cls, file_path: str):
        path = Path(file_path)
        with path.open(encoding='utf-8') as stream:
            return cls(json.load(stream), path)

    @classmethod
    def create_new(cls, name: str = 'New Assembly', document_type: str = 'Assembly'):
        return cls({
            'mountingDescription': {
                'components': [],
                'assemblyConstraints': [],
                'mountingReferences': {
                    'spawningOrigin': '',
                    'spawningTransformation': deepcopy(IDENTITY_TRANSFORM),
                    'ref_planes': [],
                    'ref_axes': [],
                    'ref_frames': [],
                },
            },
            'documentUnits': 'mm',
            'cadPath': '',
            'name': name,
            'description': '',
            'guid': str(uuid.uuid4()),
            'type': document_type,
            'saveDate': datetime.now().astimezone().isoformat(),
        })

    @staticmethod
    def validate_document(data: Dict[str, Any]):
        if not isinstance(data, dict):
            raise ValueError('The JSON root must be an object.')
        if data.get('type') not in ('Assembly', 'Component'):
            raise ValueError("Expected a description with type 'Assembly' or 'Component'.")
        mounting = data.get('mountingDescription')
        if not isinstance(mounting, dict):
            raise ValueError("Missing object 'mountingDescription'.")
        components = mounting.get('components', [])
        instructions = mounting.get('assemblyConstraints', [])
        if not isinstance(components, list) or not isinstance(instructions, list):
            raise ValueError('Components and assemblyConstraints must be arrays.')

        component_names = []
        for component in components:
            name = component.get('name') if isinstance(component, dict) else None
            if not name:
                raise ValueError('Every component requires a name.')
            component_names.append(name)
        if len(component_names) != len(set(component_names)):
            raise ValueError('Component names must be unique.')

        instruction_names = []
        for instruction in instructions:
            AssemblyDescriptionModifier.validate_instruction(instruction)
            instruction_names.append(instruction['name'])
        if len(instruction_names) != len(set(instruction_names)):
            raise ValueError('Assembly instruction names must be unique.')
        references = mounting.get('mountingReferences', {})
        if not isinstance(references, dict):
            raise ValueError('Mounting references must be an object.')
        for key in ('ref_frames', 'ref_axes', 'ref_planes'):
            items = references.get(key, [])
            if not isinstance(items, list):
                raise ValueError(f'{key} must be an array.')
            names = []
            for item in items:
                if not isinstance(item, dict) or not item.get('name'):
                    raise ValueError(f'Every {key} entry requires a name.')
                names.append(item['name'])
            if len(names) != len(set(names)):
                raise ValueError(f'Names in {key} must be unique.')

    @staticmethod
    def validate_instruction(instruction: Dict[str, Any]):
        if not isinstance(instruction, dict) or not instruction.get('name'):
            raise ValueError('Every assembly instruction requires a name.')
        match_type = instruction.get('type', 'PlaneMatch')
        if match_type == 'FrameMatch':
            for key in ('assemblyFrame', 'targetFrame'):
                reference = instruction.get(key)
                if not isinstance(reference, dict):
                    raise ValueError(f"FrameMatch requires '{key}'.")
                if not reference.get('component') or not reference.get('frame'):
                    raise ValueError(f"'{key}' requires component and frame.")
            if (instruction['assemblyFrame']['component'] ==
                    instruction['targetFrame']['component']):
                raise ValueError('FrameMatch endpoints must use different components.')
        elif match_type == 'PlaneMatch':
            if not instruction.get('component_1') or not instruction.get('component_2'):
                raise ValueError('PlaneMatch requires component_1 and component_2.')
            description = instruction.get('description')
            if not isinstance(description, dict):
                raise ValueError("PlaneMatch requires object 'description'.")
            for index in range(1, 4):
                match = description.get(f'planeMatch_{index}')
                if not isinstance(match, dict):
                    raise ValueError(f'PlaneMatch requires planeMatch_{index}.')
                for key in ('planeNameComponent_1', 'planeNameComponent_2'):
                    if not match.get(key):
                        raise ValueError(f'planeMatch_{index} requires {key}.')
        else:
            raise ValueError(f"Unsupported assembly instruction type '{match_type}'.")

        prerequisites = instruction.get('prerequisites', {})
        if not isinstance(prerequisites, dict):
            raise ValueError('Prerequisites must be an object.')
        for key in ('gluePrerequisites', 'glueLinePrerequisite'):
            for reference in prerequisites.get(key, []):
                if not isinstance(reference, dict) or not reference.get('component') \
                        or not reference.get('frame'):
                    raise ValueError(f'{key} entries require component and frame.')
        for component in prerequisites.get('componentPrerequisites', []):
            name = component.get('component') if isinstance(component, dict) else component
            if not name:
                raise ValueError('Component prerequisites require a component name.')

    @property
    def file_path(self) -> Optional[Path]:
        return self._file_path

    @property
    def components(self) -> List[Dict[str, Any]]:
        return self._data['mountingDescription']['components']

    @property
    def instructions(self) -> List[Dict[str, Any]]:
        return self._data['mountingDescription']['assemblyConstraints']

    @property
    def metadata(self) -> Dict[str, Any]:
        return {key: value for key, value in self._data.items()
                if key != 'mountingDescription'}

    @property
    def mounting_references(self) -> Dict[str, Any]:
        mounting = self._data['mountingDescription']
        return mounting.setdefault('mountingReferences', {})

    def replace_metadata(self, metadata: Dict[str, Any]):
        if not isinstance(metadata, dict):
            raise ValueError('Document metadata must be a JSON object.')
        candidate = deepcopy(metadata)
        candidate['mountingDescription'] = deepcopy(self._data['mountingDescription'])
        candidate['type'] = self._data['type']
        self.replace_document(candidate)

    def replace_document(self, data: Dict[str, Any]):
        """Validate an edited document before replacing the current data."""
        self.validate_document(data)
        self._data = deepcopy(data)

    def replace_mounting_references(self, references: Dict[str, Any]):
        if not isinstance(references, dict):
            raise ValueError('Mounting references must be a JSON object.')
        self._data['mountingDescription']['mountingReferences'] = deepcopy(references)

    def add_component(self, component: Optional[Dict[str, Any]] = None) -> int:
        number = len(self.components) + 1
        value = deepcopy(component) if component else {
            'name': f'Component-{number}',
            'type': 'component',
            'transformation': deepcopy(IDENTITY_TRANSFORM),
            'guid': str(uuid.uuid4()),
            'frameProperties': {},
        }
        self.components.append(value)
        return len(self.components) - 1

    def add_instruction(self, match_type: str) -> int:
        number = len(self.instructions) + 1
        prerequisites = {
            'gluePrerequisites': [],
            'glueLinePrerequisite': [],
            'componentPrerequisites': [],
        }
        if match_type == 'FrameMatch':
            value = {
                'name': f'frame_match_{number}',
                'type': 'FrameMatch',
                'assemblyFrame': {'component': '', 'frame': ''},
                'targetFrame': {'component': '', 'frame': ''},
                'prerequisites': prerequisites,
            }
        elif match_type == 'PlaneMatch':
            match = {
                'planeNameComponent_1': '',
                'planeNameComponent_2': '',
                'planeOffset': 0.0,
                'invertNormalVector': False,
            }
            value = {
                'name': f'plane_match_{number}',
                'type': 'PlaneMatch',
                'component_1': '',
                'component_2': '',
                'move_component_1': False,
                'description': {
                    f'planeMatch_{index}': deepcopy(match)
                    for index in range(1, 4)
                },
                'prerequisites': prerequisites,
            }
        else:
            raise ValueError(f'Unsupported match type: {match_type}')
        self.instructions.append(value)
        return len(self.instructions) - 1

    def replace_item(self, collection: str, index: int, value: Dict[str, Any]):
        if not isinstance(value, dict):
            raise ValueError('The edited value must be a JSON object.')
        target = self.components if collection == 'components' else self.instructions
        candidate = deepcopy(self._data)
        candidate_target = candidate['mountingDescription'][
            'components' if collection == 'components' else 'assemblyConstraints']
        candidate_target[index] = deepcopy(value)
        self.validate_document(candidate)
        target[index] = deepcopy(value)

    def remove_item(self, collection: str, index: int):
        target = self.components if collection == 'components' else self.instructions
        del target[index]

    def duplicate_item(self, collection: str, index: int) -> int:
        target = self.components if collection == 'components' else self.instructions
        value = deepcopy(target[index])
        existing = {item.get('name') for item in target}
        base = f"{value.get('name', 'copy')}_copy"
        value['name'] = base
        suffix = 2
        while value['name'] in existing:
            value['name'] = f'{base}_{suffix}'
            suffix += 1
        if collection == 'components':
            value['guid'] = str(uuid.uuid4())
        target.insert(index + 1, value)
        return index + 1

    def to_dict(self) -> Dict[str, Any]:
        self.validate_document(self._data)
        return deepcopy(self._data)

    def save(self, file_path: Optional[str] = None) -> Path:
        if file_path:
            self._file_path = Path(file_path)
        if self._file_path is None:
            raise ValueError('No output file selected.')
        self._data['saveDate'] = datetime.now().astimezone().isoformat()
        data = self.to_dict()
        with self._file_path.open('w', encoding='utf-8') as stream:
            json.dump(data, stream, indent=2)
            stream.write('\n')
        return self._file_path
