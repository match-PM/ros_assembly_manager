"""Import explicitly declared frame properties from description JSON."""

import assembly_manager_interfaces.msg as ami_msg


PROPERTY_GROUPS = {
    'visionProperties': ('vision_frame_properties', {
        'isVisionFrame': 'is_vision_frame',
        'hasBeenMeasured': 'has_been_measured',
    }),
    'laserProperties': ('laser_frame_properties', {
        'isLaserFrame': 'is_laser_frame',
        'hasBeenMeasured': 'has_been_measured',
    }),
    'grippingProperties': ('gripping_frame_properties', {
        'isGrippingFrame': 'is_gripping_frame',
        'compatibleGrippers': 'compatible_grippers',
        'compatibleGripperTips': 'compatible_gripper_tips',
    }),
    'gluePtProperties': ('glue_pt_frame_properties', {
        'isGlueFrame': 'is_glue_point',
        'isGluePoint': 'is_glue_point',
        'hasBeenPlaced': 'has_been_placed',
        'hasBeenCured': 'has_been_cured',
        'timeMs': 'time_ms',
        'dispenseOffsetMm': 'dispense_offset_mm',
    }),
    'glueLineProperties': ('glue_line_frame_properties', {
        'isGlueLineFrame': 'is_glue_line_frame',
        'gCodeFileName': 'g_code_file_name',
        'use_1K_dispenser': 'use_1k_dispenser',
        'hasBeenPlaced': 'has_been_placed',
    }),
    'assemblyProperties': ('assembly_frame_properties', {
        'isAssemblyFrame': 'is_assembly_frame',
        'isTargetFrame': 'is_target_frame',
        'associatedFrame': 'associated_frame',
        'associatedComponent': 'associated_component',
    }),
}


def scene_frame_name(component, frame):
    """Resolve a local reference without duplicating an existing prefix."""
    if not frame or not component or frame.startswith(f'{component}_'):
        return frame
    return f'{component}_{frame}'


def import_frame_properties(data, component_name='', properties=None):
    """Apply only JSON fields supplied, preserving other groups on overrides."""
    if properties is None:
        properties = ami_msg.RefFrameProperties()
    if not isinstance(data, dict):
        raise ValueError('Frame properties must be a JSON object.')
    for group_name, values in data.items():
        if group_name not in PROPERTY_GROUPS:
            raise ValueError(f'Unknown frame property group: {group_name}')
        attribute, aliases = PROPERTY_GROUPS[group_name]
        group = getattr(properties, attribute)
        fields = group.get_fields_and_field_types()
        for name, value in values.items():
            field = aliases.get(name, name)
            if field not in fields:
                raise ValueError(f'Unknown property: {group_name}.{name}')
            # JSON exporters sometimes write integral numeric values as ints.
            if fields[field] in ('float', 'double'):
                value = float(value)
            setattr(group, field, value)

    assembly = properties.assembly_frame_properties
    if assembly.is_assembly_frame and assembly.is_target_frame:
        raise ValueError('A frame cannot be both assembly and target frame.')
    if assembly.associated_frame:
        assembly.associated_component = (
            assembly.associated_component or component_name)
        assembly.associated_frame = scene_frame_name(
            assembly.associated_component, assembly.associated_frame)
    return properties
