# joint_listener

This ROS 2 node monitors selected positions in `sensor_msgs/JointState`. When a
joint moves beyond its configured threshold, it clears
`vision_frame_properties.has_been_measured` and
`laser_frame_properties.has_been_measured` on all component reference frames
below that joint's child link. Poses, frame roles, glue state, and all other
properties are preserved. The node uses the latest subscribed scene to request
only frames whose vision or laser measurement flag is true. If no downstream
frames are measured, it skips the service call.

Build from your workspace and start it alongside the assembly scene publisher:

```bash
colcon build --packages-select assembly_manager_interfaces assembly_scene_publisher joint_listener
source install/setup.bash
ros2 launch joint_listener joint_listener.launch.py config:=/absolute/path/to/joints.yaml
```

The supplied `config/joints.yaml` monitors all four PM robot goniometer axes
(left/right, stages 1/2), each with a threshold of 0.1 degrees.
Their child links are configured explicitly from `pm_robot_description`. Run
`ros2 launch joint_listener joint_listener.launch.py` to use this default file.

Example parameter file (replace joint names and thresholds for your robot):

```yaml
joint_listener:
  ros__parameters:
    joints: [linear_axis_joint, rotary_axis_joint]
    joints.linear_axis_joint.threshold: 0.00005
    joints.linear_axis_joint.threshold_unit: m
    joints.rotary_axis_joint.threshold: 0.1
    joints.rotary_axis_joint.threshold_unit: deg
    # Optional override if robot_description is unavailable or TF uses a prefix:
    joints.linear_axis_joint.child_frame: linear_axis_link
```

Thresholds are absolute position differences. Set `joints.<name>.threshold_unit`
to `deg` for degrees, `rad` for radians (the default), or `m` for metres on
prismatic joints. Degree thresholds are converted to radians internally to match
JointState positions. The first valid sample sets the
reference without invalidating measurements. Samples within the threshold
(including exactly the threshold) leave the reference unchanged. A crossing
invalidates measurements and sets a new reference to the current position. This
also detects slow motion made up of many small steps. Each joint is tracked
independently. NaN/infinite samples and joints absent from a message are ignored.
Continuous joints should publish unwrapped positions; a wrap across +/-pi is
conservatively treated as movement.

The node resolves joint names to child links from the transient-local
`/robot_description` topic (URDF, as published by robot_state_publisher), or uses
the explicit `joints.<name>.child_frame`. TF supplies runtime link ancestry;
scene parenting takes precedence for components and reference frames. This
includes components attached through tools, mounting frames, and other
components. Measurement frames on unrelated branches remain unchanged.

Optional parameters and their defaults:

| Parameter | Default |
| --- | --- |
| `joint_states_topic` | `/joint_states` |
| `scene_topic` | `/assembly_manager/scene` |
| `robot_description_topic` | `/robot_description` |
| `invalidate_service` | `/assembly_manager/invalidate_frame_measurements` |

Joint names must be listed in `joints`; per-joint thresholds default to zero and
must be finite and nonnegative. No joints are monitored unless configured.
Movement is queued while the scene, child-link mapping, complete TF ancestry, or
service is unavailable. Scene components and the joint must share a connected
frame tree before invalidation can be resolved.
Failed service requests are retried every 0.5 seconds using the latest scene.
The publisher's invalidation service changes only the two measurement flags on
the current frames, avoiding overwriting other properties from an old scene copy.
