"""Joint movement detection and downstream scene traversal without ROS I/O."""

import math
from xml.etree import ElementTree


class IncompleteHierarchyError(ValueError):
    """Scene and joint branches are not yet connected in the available graph."""


class JointTracker:
    """Compare positions to a fixed reference so gradual drift is detected too."""

    def __init__(self, threshold):
        if not math.isfinite(threshold) or threshold < 0:
            raise ValueError('Joint thresholds must be finite and nonnegative.')
        self.threshold = threshold
        self.reference = None

    def update(self, position):
        if not math.isfinite(position):
            return False
        if self.reference is None:
            self.reference = position
            return False
        if abs(position - self.reference) > self.threshold:
            self.reference = position
            return True
        return False


def robot_links(description):
    """Return joint-to-child and child-to-parent mappings from a URDF."""
    children, parents = {}, {}
    for joint in ElementTree.fromstring(description).findall('joint'):
        child, parent = joint.find('child'), joint.find('parent')
        if child is not None and parent is not None:
            children[joint.attrib['name']] = child.attrib['link']
            parents[child.attrib['link']] = parent.attrib['link']
    return children, parents


def is_downstream(frame, root, parents):
    """Include the root itself, and terminate safely for disconnected/cyclic trees."""
    visited = set()
    while frame and frame not in visited:
        if frame == root:
            return True
        visited.add(frame)
        frame = parents.get(frame)
    return False


def _tree_root(frame, parents):
    visited = set()
    while frame in parents and parents[frame]:
        if frame in visited:
            return None
        visited.add(frame)
        frame = parents[frame]
    return frame


def downstream_component_frames(scene, root, robot_parents):
    """Find component-owned reference frames below a joint's child link.

    Scene parenting takes precedence over TF, including reparented components
    and reference frames used as mounting points for further components.
    """
    parents = dict(robot_parents)
    for frame in scene.ref_frames_in_scene:
        parents[frame.frame_name] = frame.parent_frame
    for component in scene.objects_in_scene:
        parents[component.obj_name] = component.parent_frame
        for frame in component.ref_frames:
            parents[frame.frame_name] = frame.parent_frame
    root_ancestor = _tree_root(root, parents)
    if root_ancestor is None or any(
            _tree_root(component.obj_name, parents) != root_ancestor
            for component in scene.objects_in_scene):
        raise IncompleteHierarchyError(
            f'Waiting for TF to connect scene components to the tree containing {root}.')
    return sorted({
        frame.frame_name
        for component in scene.objects_in_scene
        for frame in component.ref_frames
        if is_downstream(frame.frame_name, root, parents)
    })
