# Description imports

Frame roles are explicit. Frame names never assign vision, laser, gripping,
glue-point, glue-line, assembly, or target roles. A frame without `properties`
is an ordinary reference frame, even if its name contains `Vision` or `Glue`.
The geometric JSON `type` (`point` / `frame`) does not assign a process role.

Component reference frames accept these property groups:

| JSON group | ROS property message |
| --- | --- |
| `visionProperties` | `VisionFrameProperties` |
| `laserProperties` | `LaserFrameProperties` |
| `grippingProperties` | `GrippingFrameProperties` |
| `gluePtProperties` | `GluePtFrameProperties` |
| `glueLineProperties` | `GlueLineFrameProperties` |
| `assemblyProperties` | `AssemblyFrameProperties` |

For example:

```json
{
  "properties": {
    "glueLineProperties": {
      "isGlueLineFrame": true,
      "gCodeFileName": "path.nc",
      "use_1K_dispenser": true
    }
  }
}
```

Glue lines also track `has_been_placed` in ROS, exposed as
`hasBeenPlaced` in JSON. JSON property imports accept the ROS
snake-case field spelling as well. Unknown property groups/fields are errors.

Constraints are optional and sparse: omit unused `centroid`, `inPlane`,
`orthogonal`, or `transform` objects. Omitted constraints remain inactive.
Transform quaternions use `X`, `Y`, `Z`, `W`.

Assembly component entries can supply `frameProperties`, keyed by local frame
name. These override only the specified fields after all components exist;
other frame roles and constraints are preserved. `associatedComponent` resolves
`associatedFrame` into a full scene frame name.

## Assembly matches

Plane and frame instructions have separate ROS messages and scene lists:

| Match | Message | `ObjectScene` list | Create service |
| --- | --- | --- | --- |
| Planes | `AssemblyInstruction` | `assembly_instructions` | `/assembly_manager/create_assembly_instructions` |
| Frames | `FrameAssemblyInstruction` | `frame_assembly_instructions` | `/assembly_manager/create_frame_assembly_instructions` |

Both use `/assembly_manager/calculate_assembly_instructions` for calculation
by ID. IDs are unique across the two lists so calculation, lookup, and
prerequisite checks are unambiguous. Updating an ID within the same list
replaces that instruction. Description JSON uses `type: "PlaneMatch"` or
`type: "FrameMatch"`; the importer routes each to its typed create service.

When `spawn_components` is false, description imports support incremental
assembly setup: instructions whose components are missing are skipped with a
warning, as are frame-property overrides on missing components. The import
succeeds if at least one instruction is created or updated; it fails if none
can be created. The response reports created/skipped counts and the last
successfully created instruction ID. Import again after spawning more
components to create the remaining instructions. Other creation errors still
fail the import, and `spawn_components: true` retains strict behavior.

Plane matches retain the three `plane_match_*` fields and generate their own
assembly and target frames. The JSON moving-part flag is `move_component_1`;
the previous `moveComponent_1` spelling and missing `type` remain supported.

Frame matches use `FrameAssemblyInstruction.frame_match` on the instruction message (the
`frame_match` field has message type `FrameMatch`):

```json
{
  "name": "place_module",
  "type": "FrameMatch",
  "component_1": "Module-1",
  "component_2": "Base-1",
  "assemblyFrame": {"component": "Module-1", "frame": "pickup"},
  "targetFrame": {"component": "Base-1", "frame": "site"}
}
```

`assemblyFrame` identifies the moving component; `targetFrame` identifies the
stationary component. Both frames must already exist on the specified
components. Creation assigns their assembly/target roles and reciprocal
associations, without duplicating frames or changing their poses/constraints.
A frame already associated with another frame is rejected.

For a frame match, calculation returns a world-space rigid displacement
`delta = world_T_target * inverse(world_T_assembly)`. Apply this displacement
to the moving component's world pose. Frame matching always aligns origins
and axes. The former `align_axes` ROS field has been removed; legacy
`alignAxes` JSON keys are ignored.
Creation itself does not require TF; calculating the displacement requires
the parent transforms to be available in TF.

## Shared prerequisites

Both instruction messages carry the same `AssemblyPrerequisites` field:

| JSON field | ROS field | Element |
| --- | --- | --- |
| `gluePrerequisites` | `glue_prerequisites` | `FrameReference` |
| `glueLinePrerequisite` | `glue_line_prerequisites` | `FrameReference` |
| `componentPrerequisites` | `component_prerequisites` | component name string |

```json
{
  "prerequisites": {
    "gluePrerequisites": [{"component": "Base-1", "frame": "dot"}],
    "glueLinePrerequisite": [{"component": "Base-1", "frame": "line"}],
    "componentPrerequisites": [{"component": "Previous_Module-1"}]
  }
}
```

`FrameReference` holds a component instance name and a local frame name. Empty
lists impose no prerequisites. JSON also accepts plural `glueLinePrerequisites`.

Prerequisites are stored and published with the scene. They do not prevent
preparing or calculating geometry. Call
`/assembly_manager/check_assembly_prerequisites` (`CheckAssemblyPrerequisites`)
with an `instruction_id` to query readiness. It returns `success`, `satisfied`,
`unmet_prerequisites`, and `message`. Readiness requires glue placement and
`is_assembled` for prerequisite components; curing is not implicitly required.
Missing references count as unmet; an unknown instruction is a service error.

The updated ROS messages change the wire format of `ObjectScene` and dependent
services. Rebuild dependent packages and restart all nodes consuming these
messages, including nodes in other workspaces, before using the new interfaces.
