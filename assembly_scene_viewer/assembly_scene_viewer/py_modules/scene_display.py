"""Human-readable projections of ObjectScene messages for the Qt viewer."""

from html import escape


def _yes(value):
    return 'yes' if value else 'no'


def _pose(pose):
    return (
        f'position ({pose.position.x:.6f}, {pose.position.y:.6f}, '
        f'{pose.position.z:.6f})<br>'
        f'orientation ({pose.orientation.x:.6f}, {pose.orientation.y:.6f}, '
        f'{pose.orientation.z:.6f}, {pose.orientation.w:.6f})'
    )


def scene_overview_html(scene):
    occupied = '<br>'.join(
        f'&nbsp;&nbsp;{escape(name)}' for name in scene.occupied_spawning_frames)
    if not occupied:
        occupied = '&nbsp;&nbsp;none'
    object_names = ', '.join(escape(item.obj_name)
                             for item in scene.objects_in_scene) or 'none'
    frame_names = ', '.join(escape(item.frame_name)
                            for item in scene.ref_frames_in_scene) or 'none'
    axis_names = ', '.join(escape(item.axis_name)
                           for item in scene.axis_in_scene) or 'none'
    plane_names = ', '.join(escape(item.ref_plane_name)
                            for item in scene.planes_in_scene) or 'none'
    return (
        '<b>Scene contents</b><br>'
        f'Objects: {len(scene.objects_in_scene)}<br>'
        f'Reference frames: {len(scene.ref_frames_in_scene)}<br>'
        f'Axes: {len(scene.axis_in_scene)}<br>'
        f'Planes: {len(scene.planes_in_scene)}<br>'
        f'Plane instructions: {len(scene.assembly_instructions)}<br>'
        f'Frame instructions: {len(scene.frame_assembly_instructions)}<br><br>'
        f'<b>Objects</b><br>{object_names}<br><br>'
        f'<b>All reference frames</b><br>{frame_names}<br><br>'
        f'<b>All axes</b><br>{axis_names}<br><br>'
        f'<b>All planes</b><br>{plane_names}<br><br>'
        '<b>Occupied spawning frames</b><br>' + occupied
    )


def object_html(obj):
    props = obj.properties
    color = obj.apperance_color
    return (
        f'<b>Object: {escape(obj.obj_name)}</b><br>'
        f'Parent frame: {escape(obj.parent_frame)}<br>'
        f'UUID: {escape(obj.uuid)}<br>'
        f'Component type UUID: {escape(obj.component_type_uuid)}<br>'
        f'Component description: {escape(obj.parent_file_path)}<br>'
        f'CAD: {escape(obj.cad_data)}<br>'
        f'Collision CAD: {escape(obj.cad_data_collision)}<br><br>'
        f'<b>Pose</b><br>{_pose(obj.obj_pose)}<br><br>'
        '<b>State</b><br>'
        f'Gripped: {_yes(props.is_gripped)}<br>'
        f'Placed: {_yes(props.is_placed)}<br>'
        f'Assembled: {_yes(props.is_assembled)}<br>'
        f'On chuck: {_yes(props.is_on_chuck)}<br><br>'
        '<b>Appearance</b><br>'
        f'RGBA ({color.r:.3f}, {color.g:.3f}, {color.b:.3f}, {color.a:.3f})<br><br>'
        '<b>Owned scene elements</b><br>'
        f'Frames: {len(obj.ref_frames)}, axes: {len(obj.ref_axis)}, '
        f'planes: {len(obj.ref_planes)}'
    )


def frame_properties_html(frame):
    props = frame.properties
    rows = []
    vision = props.vision_frame_properties
    if vision.is_vision_frame:
        rows.append(f'<b>Vision</b>: measured {_yes(vision.has_been_measured)}')
    laser = props.laser_frame_properties
    if laser.is_laser_frame:
        rows.append(f'<b>Laser</b>: measured {_yes(laser.has_been_measured)}')
    glue = props.glue_pt_frame_properties
    if glue.is_glue_point:
        rows.append(
            '<b>Glue point</b>: placed {}, cured {}, time {} ms, offset {} mm'.format(
                _yes(glue.has_been_placed), _yes(glue.has_been_cured),
                glue.time_ms, glue.dispense_offset_mm))
    line = props.glue_line_frame_properties
    if line.is_glue_line_frame:
        rows.append(
            '<b>Glue line</b>: placed {}, G-code {}, 1K dispenser {}'.format(
                _yes(line.has_been_placed), escape(line.g_code_file_name),
                _yes(line.use_1k_dispenser)))
    gripping = props.gripping_frame_properties
    if gripping.is_gripping_frame:
        rows.append(
            '<b>Gripping</b>: grippers [{}], tips [{}]'.format(
                ', '.join(map(escape, gripping.compatible_grippers)),
                ', '.join(map(escape, gripping.compatible_gripper_tips))))
    assembly = props.assembly_frame_properties
    if assembly.is_assembly_frame or assembly.is_target_frame:
        roles = []
        if assembly.is_assembly_frame:
            roles.append('assembly')
        if assembly.is_target_frame:
            roles.append('target')
        rows.append(
            '<b>Assembly</b>: role {}, associated component {}, frame {}'.format(
                '/'.join(roles), escape(assembly.associated_component),
                escape(assembly.associated_frame)))
    return '<br>'.join(rows) if rows else 'No process properties'


def axis_html(axis):
    points = ', '.join(escape(name) for name in axis.point_names if name)
    return f'<b>Axis: {escape(axis.axis_name)}</b><br>Points: {points or "none"}'


def plane_html(plane):
    points = ', '.join(escape(name) for name in plane.point_names if name)
    axes = ', '.join(escape(name) for name in plane.axis_names if name)
    normal = plane.ideal_norm_vector
    return (
        f'<b>Plane: {escape(plane.ref_plane_name)}</b><br>'
        f'Points: {points or "none"}<br>Axes: {axes or "none"}<br>'
        f'Ideal normal: ({normal.x:.6f}, {normal.y:.6f}, {normal.z:.6f})'
    )


def _reference(reference):
    return f'{escape(reference.component)} / {escape(reference.frame)}'


def _prerequisites_html(prerequisites, scene):
    objects = {obj.obj_name: obj for obj in scene.objects_in_scene}
    frames = {frame.frame_name: frame for frame in scene.ref_frames_in_scene}
    rows = []
    for label, references, property_name in (
            ('Glue point', prerequisites.glue_prerequisites,
             'glue_pt_frame_properties'),
            ('Glue line', prerequisites.glue_line_prerequisites,
             'glue_line_frame_properties')):
        for reference in references:
            prefix = f'{reference.component}_'
            name = (reference.frame if reference.frame.startswith(prefix)
                    else f'{prefix}{reference.frame}')
            frame = frames.get(name)
            group = getattr(frame.properties, property_name) if frame else None
            met = bool(group and group.has_been_placed)
            rows.append(f'{label}: {_reference(reference)} — '
                        f'<b>{"met" if met else "unmet"}</b>')
    for component in prerequisites.component_prerequisites:
        obj = objects.get(component)
        met = bool(obj and obj.properties.is_assembled)
        rows.append(f'Component: {escape(component)} — '
                    f'<b>{"met" if met else "unmet"}</b>')
    return '<br>'.join(rows) if rows else 'None'


def instruction_html(instruction, scene):
    is_frame = hasattr(instruction, 'frame_match')
    result = [
        f'<b>Instruction: {escape(instruction.id)}</b>',
        f'Type: {"FrameMatch" if is_frame else "PlaneMatch"}',
        f'Activated: {_yes(instruction.activated)}',
        f'Source: {escape(instruction.parent_file_path)}',
    ]
    if is_frame:
        match = instruction.frame_match
        result.extend([
            f'Moving assembly frame: {_reference(match.assembly_frame)}',
            f'Stationary target frame: {_reference(match.target_frame)}',
        ])
    else:
        moving_component = (instruction.component_1
                            if instruction.component_1_is_moving_part
                            else instruction.component_2)
        result.extend([
            f'Component 1: {escape(instruction.component_1)}',
            f'Component 2: {escape(instruction.component_2)}',
            f'Moving component: {escape(moving_component)}',
            '<br><b>Plane matches</b>',
        ])
        for index in range(1, 4):
            match = getattr(instruction, f'plane_match_{index}')
            result.append(
                f'{index}: {escape(match.plane_name_component_1)} ↔ '
                f'{escape(match.plane_name_component_2)}, offset '
                f'{match.plane_offset:.6f} m, invert normal '
                f'{_yes(match.inv_normal_vector)}')
    result.extend([
        '<br><b>Prerequisites</b>',
        _prerequisites_html(instruction.prerequisites, scene),
    ])
    return '<br>'.join(result)
