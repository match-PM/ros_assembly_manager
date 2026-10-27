"""Load component and assembly descriptions into an assembly scene."""

import json
import os
import re
import time
from copy import deepcopy

import assembly_manager_interfaces.msg as ami_msg
import assembly_manager_interfaces.srv as ami_srv
from std_msgs.msg import ColorRGBA

from assembly_scene_publisher.py_modules.AssemblySceneAnalyzerAdv import (
    AssemblySceneAnalyzerAdv,
)
from assembly_scene_publisher.py_modules.frame_constraints import (
    FrameConstraintsHandler,
)
from assembly_scene_publisher.py_modules.frame_properties import (
    import_frame_properties,
    scene_frame_name,
)


class AssemblyDescriptionHandler:
    """Translate exported JSON descriptions into scene service operations."""

    UNIT_MULTIPLIERS = {
        'mm': 0.001,
        'cm': 0.01,
        'um': 0.000001,
    }

    def __init__(
            self,
            node,
            object_scene,
            spawn_object_callback,
            create_ref_frame_callback,
            create_axis_callback,
            create_ref_plane_callback,
            create_assembly_instructions_callback,
            create_frame_assembly_instructions_callback):
        self._logger = node.get_logger()
        self._object_scene = object_scene
        self._spawn_object = spawn_object_callback
        self._create_ref_frame = create_ref_frame_callback
        self._create_axis = create_axis_callback
        self._create_ref_plane = create_ref_plane_callback
        self._create_assembly_instructions = \
            create_assembly_instructions_callback
        self._create_frame_assembly_instructions = \
            create_frame_assembly_instructions_callback
        self._scene_analyzer = AssemblySceneAnalyzerAdv(
            object_scene.scene, self._logger)

    def spawn_component_callback(self, request, response):
        """Handle SpawnComponentFromDescription requests."""
        response.success, response.message = self.spawn_component(request)
        return response

    def create_assembly_instructions_callback(self, request, response):
        """Handle CreateAssemblyInstructionFromDescription requests."""
        self._logger.info('Creating assembly instructions.')
        file_data, error = self._load_json_file(request.file_path)
        if error:
            return self._failed_response(response, error)

        if file_data.get('type') != 'Assembly':
            return self._failed_response(
                response, 'Error: File is not an assembly file!')

        try:
            mounting_description = file_data['mountingDescription']
            multiplier = self._unit_multiplier(
                file_data.get('documentUnits'))

            if request.spawn_components:
                success, message = self._spawn_assembly_components(
                    mounting_description.get('components', []),
                    request.file_path,
                )
                if not success:
                    return self._failed_response(response, message)

            # Incremental imports may reference components not spawned yet.
            self._apply_component_frame_properties(
                mounting_description.get('components', []),
                skip_missing_components=not request.spawn_components)
            self._scene_analyzer.set_scene(self._object_scene.scene)

            created_count = 0
            response.instruction_id = ''
            for constraint in mounting_description.get(
                    'assemblyConstraints', []):
                instruction_request = self._build_instruction_request(
                    constraint, multiplier)
                instruction_request.assembly_instruction.parent_file_path = \
                    request.file_path
                instruction = instruction_request.assembly_instruction
                if isinstance(instruction_request,
                              ami_srv.CreateFrameAssemblyInstructions.Request):
                    instruction_response = self._create_frame_assembly_instructions(
                        instruction_request,
                        ami_srv.CreateFrameAssemblyInstructions.Response(),
                    )
                else:
                    instruction_response = self._create_assembly_instructions(
                        instruction_request,
                        ami_srv.CreateAssemblyInstructions.Response(),
                    )
                if not instruction_response.success:
                    return self._failed_response(
                        response,
                        'Error while creating assembly instruction '
                        f"{instruction.id}!",
                    )
                created_count += 1
                response.instruction_id = instruction.id

            if created_count == 0:
                return self._failed_response(
                    response,
                    'No assembly instructions found in the description.',
                )

            time.sleep(1.5)
            response.success = True
            response.message = (
                f'Created {created_count} assembly instruction(s).')
        except Exception as error:
            return self._failed_response(
                response,
                'Error while creating assembly instructions from '
                f'description: {error}',
            )
        return response

    def spawn_component(self, request, component_name_override=None):
        """Spawn a component and its reference geometry from a JSON file."""
        self._logger.info('Spawning component from description.')
        file_data, error = self._load_json_file(request.file_path)
        if error:
            return False, error

        component_name = (
            component_name_override
            or request.component_name_override
            or file_data.get('name')
        )

        try:
            references = file_data['mountingDescription'][
                'mountingReferences']
            units = file_data.get('documentUnits')
            multiplier = self._unit_multiplier(units)
            spawn_request = self._build_spawn_request(
                request,
                file_data,
                references,
                component_name,
                multiplier,
            )

            spawn_response = self._spawn_object(
                spawn_request, ami_srv.SpawnObject.Response())
            if not spawn_response.success:
                return False, spawn_response.message

            # Constraint calculations can require the component TF.
            time.sleep(0.5)

            success, message = self._spawn_reference_frames(
                component_name,
                references.get('ref_frames', []),
                units,
                multiplier,
            )
            if not success:
                return False, message

            success, message = self._spawn_reference_axes(
                component_name, references.get('ref_axes', []))
            if not success:
                return False, message

            success, message = self._spawn_reference_planes(
                component_name, references.get('ref_planes', []))
            if not success:
                return False, message

            self._object_scene.initialize_pending_instructions()
            self._object_scene.publish_information()

            return (
                True,
                f"Component '{component_name}' spawned successfully from "
                'description.',
            )
        except Exception as error:
            message = (
                f"Error while spawning component '{component_name}' from "
                f'description: {error}')
            self._logger.error(message)
            return False, message

    def _spawn_assembly_components(
            self, components, assembly_file_path):
        self._scene_analyzer.set_scene(self._object_scene.scene)
        component_directory = os.path.dirname(assembly_file_path).replace(
            'assemblies', 'components')

        for component in components:
            component_name = component.get('name')
            valid, message = self._validate_component_name(component_name)
            if not valid:
                return False, message

            if self._scene_analyzer.check_component_exists(component_name):
                self._logger.info(
                    f"Component '{component_name}' already exists; "
                    'skipping it.')
                continue

            component_request = \
                ami_srv.SpawnComponentFromDescription.Request()
            component_request.file_path = os.path.join(
                component_directory,
                f"{component_name.rsplit('-', 1)[0]}.json")
            success, message = self.spawn_component(
                component_request,
                component_name_override=component_name,
            )
            if not success:
                return False, message

        return True, ''

    def _apply_component_frame_properties(
            self, components, skip_missing_components=False):
        self._scene_analyzer.set_scene(self._object_scene.scene)
        updates = []
        for component in components:
            component_name = component['name']
            if (skip_missing_components and
                    not self._scene_analyzer.check_component_exists(component_name)):
                if component.get('frameProperties'):
                    self._logger.warn(
                        f"Skipping frame properties for component '{component_name}': "
                        'component is not in the scene.')
                continue
            for frame_name, properties in component.get(
                    'frameProperties', {}).items():
                frame = self._scene_analyzer.get_ref_frame_by_name(
                    scene_frame_name(component_name, frame_name))
                updates.append((frame, import_frame_properties(
                    properties, component_name, deepcopy(frame.properties))))
        for frame, properties in updates:
            frame.properties = properties

    def _spawn_reference_frames(
            self, component_name, frames, units, multiplier):
        for index, frame_data in enumerate(frames):
            request = self._build_ref_frame_request(
                frame_data,
                component_name,
                units,
                multiplier,
                recalculate_constraints=index == len(frames) - 1,
            )
            response = self._create_ref_frame(
                request, ami_srv.CreateRefFrame.Response())
            if not response.success:
                message = (
                    'Error while spawning ref frame '
                    f'{request.ref_frame.frame_name}!')
                self._logger.error(message)
                return False, message
        return True, ''

    def _spawn_reference_axes(self, component_name, axes):
        for axis_data in axes:
            request = ami_srv.CreateAxis.Request()
            request.axis.axis_name = \
                f"{component_name}_{axis_data.get('name')}"
            self._prefix_nonempty_names(
                axis_data.get('refPointNames', []),
                request.axis.point_names,
                component_name,
            )
            response = self._create_axis(
                request, ami_srv.CreateAxis.Response())
            if not response.success:
                message = (
                    f'Error while spawning axis {request.axis.axis_name}!')
                self._logger.error(message)
                return False, message
        return True, ''

    def _spawn_reference_planes(self, component_name, planes):
        for plane_data in planes:
            request = ami_srv.CreateRefPlane.Request()
            plane = request.ref_plane
            plane.ref_plane_name = \
                f"{component_name}_{plane_data.get('name')}"
            self._prefix_nonempty_names(
                plane_data.get('refPointNames', []),
                plane.point_names,
                component_name,
            )
            self._prefix_nonempty_names(
                plane_data.get('refAxisNames', []),
                plane.axis_names,
                component_name,
            )
            normal = plane_data['normalVector']
            plane.ideal_norm_vector.x = normal['X']
            plane.ideal_norm_vector.y = normal['Y']
            plane.ideal_norm_vector.z = normal['Z']

            response = self._create_ref_plane(
                request, ami_srv.CreateRefPlane.Response())
            if not response.success:
                message = (
                    f'Error while spawning plane {plane.ref_plane_name}!')
                self._logger.error(message)
                return False, message
        return True, ''

    def _build_spawn_request(
            self,
            source_request,
            file_data,
            references,
            component_name,
            multiplier):
        request = ami_srv.SpawnObject.Request()
        request.obj_name = component_name
        if source_request.uuid_override:
            request.uuid = source_request.uuid_override
        request.apperance_color = self._build_color(file_data.get('color'))
        request.component_type_uuid = file_data.get('guid')
        request.parent_frame = references.get('spawningOrigin')

        description_dir = os.path.dirname(source_request.file_path)
        cad_path = file_data.get('cadPath')
        collision_path = file_data.get('cadPathCollision') or cad_path
        request.cad_data = os.path.join(description_dir, cad_path)
        request.cad_data_collision = os.path.join(
            description_dir, collision_path)

        transform = references['spawningTransformation']
        self._set_pose_fields(
            request.translation,
            request.rotation,
            transform,
            multiplier,
        )
        return request

    def _build_ref_frame_request(
            self,
            frame_data,
            component_name,
            units,
            multiplier,
            recalculate_constraints):
        request = ami_srv.CreateRefFrame.Request()
        request.recalculate_constraints = recalculate_constraints
        request.ref_frame.frame_name = \
            f"{component_name}_{frame_data.get('name')}"
        request.ref_frame.parent_frame = component_name
        self._set_pose_fields(
            request.ref_frame.pose.position,
            request.ref_frame.pose.orientation,
            frame_data['transformation'],
            multiplier,
        )

        constraint_handler = FrameConstraintsHandler.return_handler_from_dict(
            dictionary=frame_data.get('constraints', {}),
            component_name=component_name,
            logger=self._logger,
        )
        constraint_handler.unit = units
        request.ref_frame.constraints = constraint_handler.return_as_msg()
        request.ref_frame.properties = import_frame_properties(
            frame_data.get('properties', {}), component_name)
        return request

    @staticmethod
    def _build_instruction_request(constraint, multiplier):
        match_type = constraint.get('type', 'PlaneMatch')
        if match_type == 'FrameMatch':
            request = ami_srv.CreateFrameAssemblyInstructions.Request()
        elif match_type == 'PlaneMatch':
            request = ami_srv.CreateAssemblyInstructions.Request()
        else:
            raise ValueError(f'Unknown assembly match type: {match_type}')
        instruction = request.assembly_instruction
        component_1 = constraint.get('component_1')
        component_2 = constraint.get('component_2')
        instruction.id = constraint.get('name')
        instruction.prerequisites = \
            AssemblyDescriptionHandler._build_prerequisites(
                constraint.get('prerequisites', {}))

        if match_type == 'FrameMatch':
            match = instruction.frame_match
            match.assembly_frame = AssemblyDescriptionHandler._frame_reference(
                constraint['assemblyFrame'])
            match.target_frame = AssemblyDescriptionHandler._frame_reference(
                constraint['targetFrame'])
            if match.assembly_frame.component == match.target_frame.component:
                raise ValueError('FrameMatch requires two distinct components.')
            if (component_1 is not None or component_2 is not None) and \
                    {match.assembly_frame.component, match.target_frame.component} \
                    != {component_1, component_2}:
                raise ValueError(
                    'FrameMatch endpoints must reference component_1 and '
                    'component_2, on different components.')
            return request

        instruction.component_1 = component_1
        instruction.component_2 = component_2
        instruction.component_1_is_moving_part = constraint.get(
            'move_component_1', constraint.get('moveComponent_1', False))

        description = constraint.get('description')
        for index in range(1, 4):
            match_data = description.get(f'planeMatch_{index}')
            plane_match = getattr(instruction, f'plane_match_{index}')
            plane_match.plane_name_component_1 = (
                f"{component_1}_"
                f"{match_data.get('planeNameComponent_1')}")
            plane_match.plane_name_component_2 = (
                f"{component_2}_"
                f"{match_data.get('planeNameComponent_2')}")
            plane_match.plane_offset = \
                match_data.get('planeOffset') * multiplier
            plane_match.inv_normal_vector = \
                match_data.get('invertNormalVector')
        return request

    @staticmethod
    def _frame_reference(data):
        reference = ami_msg.FrameReference()
        reference.component = data['component']
        reference.frame = data['frame']
        if not reference.component or not reference.frame:
            raise ValueError('Frame references require component and frame.')
        return reference

    @staticmethod
    def _build_prerequisites(data):
        prerequisites = ami_msg.AssemblyPrerequisites()
        prerequisites.glue_prerequisites = [
            AssemblyDescriptionHandler._frame_reference(item)
            for item in data.get('gluePrerequisites', [])
        ]
        prerequisites.glue_line_prerequisites = [
            AssemblyDescriptionHandler._frame_reference(item)
            for item in data.get('glueLinePrerequisite',
                                 data.get('glueLinePrerequisites', []))
        ]
        prerequisites.component_prerequisites = [
            item['component'] if isinstance(item, dict) else item
            for item in data.get('componentPrerequisites', [])
        ]
        if any(not name for name in prerequisites.component_prerequisites):
            raise ValueError('Component prerequisites require a name.')
        return prerequisites

    @staticmethod
    def _set_pose_fields(translation, rotation, transform, multiplier):
        source_translation = transform['translation']
        source_rotation = transform['rotation']
        translation.x = source_translation['X'] * multiplier
        translation.y = source_translation['Y'] * multiplier
        translation.z = source_translation['Z'] * multiplier
        rotation.w = source_rotation['W']
        rotation.x = source_rotation['X']
        rotation.y = source_rotation['Y']
        rotation.z = source_rotation['Z']

    @staticmethod
    def _build_color(color_data):
        color_data = color_data or {}
        color = ColorRGBA()
        color.r = float(color_data.get('R', 0.0))
        color.g = float(color_data.get('G', 0.0))
        color.b = float(color_data.get('B', 0.0))
        color.a = 1.0
        return color

    @staticmethod
    def _prefix_nonempty_names(source_names, target_names, component_name):
        for index, name in enumerate(source_names):
            if name:
                target_names[index] = f'{component_name}_{name}'

    @classmethod
    def _unit_multiplier(cls, units):
        return cls.UNIT_MULTIPLIERS.get(units, 1)

    @staticmethod
    def _validate_component_name(component_name):
        if re.match(r'^.*-\d+$', component_name):
            return True, f"Component name '{component_name}' is valid."
        return (
            False,
            f"Component name '{component_name}' does not match the required "
            "pattern. It must end with '-X' where X is a positive integer "
            "(e.g., 'component-1').",
        )

    def _load_json_file(self, file_path):
        try:
            with open(file_path, 'r') as file:
                return json.load(file), None
        except FileNotFoundError:
            message = f"Error: File not found at path '{file_path}'."
        except json.JSONDecodeError as error:
            message = f'Error decoding JSON: {error}'
        except Exception as error:
            message = f'An unexpected error occurred: {error}'
        self._logger.error(message)
        return None, message

    def _failed_response(self, response, message):
        response.message = message
        response.success = False
        self._logger.error(message)
        return response
