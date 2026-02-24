
# ROS2 Assembly Manager

## 1. Description
This package provides services to spawn components (parts to be handled or assembled) and to interact with them. It also provides the methodology to calculate the transformation between components based on assembly mates defined in SolidWorks. For a convenient workflow its the easiest to use the SolidWorks exporter (SolidWorks_ASsembly_Instructor (SWASI) repository) that is able to extract the assembly information from CAD as a json, which can be used as input to the assembly manager. The package has been specifically designed to be used for micro-assembly processes as the assembly transformation is calculated based on tf_frames from ROS2. It also uses the “planning_scene_interface” from Moveit2 to visualize components in Rviz2 and to account for collision detection. For more details see sec 2.

## 2. Concept of the ROS Assembly Manager
In an assembly process, the assembly part's (also referred to as components/objects) geometry and position within the assembly machine are crucial information. To be able to assemble goods in a digital simulation of a micro-assembly machine, this package provides an interface to handle components and assemblies within ROS2. 

For more information on how an component is defined in this context see “4. Messages”. Besides geometry and pose of components, precision assembly processes generally rely on several teach points at which different tasks are performed. These teach points can be related to poses of metrology systems, assembly chucks or to specific assembly components (objects). To account for these teach points in ROS2 this package allows to create so called “reference frames” (ref_frames), which simply represent a transformation from a given parent frame to the reference_frame.  

When launched multiple nodes start that interact with each other via service calls and topic messages. The following figure gives an overview of all the involved nodes and the service interfaces between the users and the nodes themselves. 

* `Assembly Scene Publisher`:
A central node is the ’assembly scene publisher’, which handles and publishes all assembly information to the ROS2 topics space (/assembly_scene). It provides multiple services to create reference frames, axes, reference planes, but also assembly instructions, which are able to calculate the transformation of components to achieve an assembly. Additionally, the node also publishes all the registered reference frames and object transformations on the ’tf static’ (tf2) topic from were the information is accessible by the other involved nodes.

* `Assembly Manager`:
Objective of the ’assembly manager’ node is to ensure consistency over the registered components within the different nodes and to react to failed service calls. It serves as interface for spawning and destroying components, as this services have to be forwarted to the other involved nodes. It also provides a service with which a full assembly scene, which might consist of multiple components as well as their reference frames, axes planes and constraints, can be initiated in a single call. This is the intended way of using this package.

* `moveit component spawner`: This node serves as the interface to the planning_scene_interface, with which objects can be spawned in the planning_scene and thus be visulized in Rviz2. The objects are then also considered for collision detection. This node subscribes to the “tf_static” topic and thus updates objects pose information automatically, when a change in the object_publisher occurs.



## 3. Installation & Launch

### Prerequisites
Make sure you have the following packages installed:

**ROS 2 Humble:**
```bash
apt install ros-humble-moveit
```

**Python Dependencies:**
All required Python packages are listed in `requirements.txt`. Install them using:
```bash
pip install -r requirements.txt
```

This includes: numpy, scipy, sympy, matplotlib, vtk, PyQt6, PyYAML, graphviz, fitter, pytest, and setuptools.

### Launching the Assembly Manager

To launch the ROS2 Assembly Manager, use the following command:
```bash
ros2 launch assembly_manager assembly_manager.launch.py
```

### Future Enhancements
- Add launch argument for sim_time configuration (true/false)
- Make topic names ('tf_static', '/planning_scene') configurable via launch parameters
- Implement installation via rosdep

## 4. Messages
Messages and Services are sourced from the assembly_manager_interfaces package.

## 5. Services
Messages and Services are sourced from the spawn_object_interfaces package.
Most services are self-explanatory and take one of the above described messages as input.

The ros assembly manager offers the following services:
```
SpawnObject
```
(This service might be renamed to 'SpawnComponent' to adhere to the naming conventions.)
Call this service to spawn an component.
* `obj_name` (str): Name of the object to spawn. This should be a unique name (unique to the assembly scene and to the tf frames)!
* `parent_frame` (str): Name of the parent frame, the object should be connected to and spawned at
* `translation` (Vector3): Rel. translation at which the object should be spawned with respect to the parent_frame
* `rotation` (Quaternion): Rel. orientation at which the object should be spawned with respect to the parent_frame
* `cad_data` (str): Path to the stl-file of the object
------------------------
* `success` (bool): Returns true if spawning was successful

```
DestroyObject
```
(This service might be renamed to 'DestroyComponent' to adhere to the naming conventions.)
Call this to destroy an component. Calling this will also destroy all feature (frames, axis, planes) that are connected to the component.
* `obj_name` (str): Name of the object to destroy
------------------------
* `success` (bool): Returns true if destroying was successful

```
ChangeParentFrame
```
Call this to change the parent_frame of an components. This call can be only used for components and not for ref_frames. Use case is gripping of a component. 
* `obj_name`: Name of the object
* `parent_frame`: New parent_frame
------------------------
* `success` (bool): Returns true if successful
```
CreateAssemblyInstructions
```
The CreateAssemblyInstructions service is used to create assembly instructions.

* `assembly_instruction` (AssemblyInstruction): Assembly instruction message,
---
* `success` (bool): Success indicator
* `instruction_id` (bool): ID of the created instruction as specified in the assembly_instruction
* `message` (string): Additional information or error message
```
CalculateAssemblyInstructions
```
This service recalculated the assembly instruction specified by the given id
* `instruction_id` (string): Assembly instruction to be recalculated. It must already exist
---
* `success` (bool): Success indicator
* `assembly_transform` (geometry_msgs/Pose): Relative transformation between two components, that specifies how the two components have to be assembled.

```
ModifyPose
```
This service can be called to modify a the pose of an component or a ref_frame.
To-Do: This service has not been tested properly. Please report bugs.
* `frame_name`: Component or ref_frame name
* `rel_pose`: Relative pose to modify the current pose of the component/ref_frame. Translations will be added on current pose; rotations will be applied to current pose.
------------------------
* `success` (bool): Returns true if successful
```
GetScene
```
Call this service to get a full overview of the current assembly scene describte by the registered components and assembly instructions. 
* None
------------------------
* `scene` (ObjectScene): Object Scene message with contains al the registered object messages and assembly instructions.
* `success` (bool): Success indicator
```
SpawnComponentFromDescription
```
Call this service with the filepath to a json as input to spawn a component. The json for a component can be created using our SolidWorks exporter (SWASI). 
* `file_path` (string): Path to the json file describing the component.
* `component_name_override` (string): Overwrite for the component name with which it will be registered in the assembly scene. If left empthy, the name will be derived from the json (in this case you cannot spawn another as this would cause a naming conflict).
------------------------
* `success` (bool): Success indicator
```
SpawnAssemblyInstructionFromDescription
```
Call this service with the filepath to a json as input to spawn a assembly instruction. The json for the assembly can be created using our SolidWorks exporter (SWASI). 
* `file_path` (string): Path to the json file describing the assembly.
* `spawn_components` (bool): If set to true, the service will also spawn the components to the scene.
------------------------
* `success` (bool): Success indicator
* `instruction_id` (bool): ID of the created instruction

## 6. Intended Usage

This package is intended to be used in conjunction with the SWASI (SolidWorks Assembly Instructor) exporter, which outputs JSON files for each assembly and component. The assemblies must be mated together following specific rules defined in the SWASI repository.

### Typical Workflow

1. **Prepare and export** the assembly or component from SolidWorks using the SWASI exporter
2. **Choose spawning method:**
   - **Option A:** Call `SpawnComponentFromDescription` to spawn a single exported component
   - **Option B:** Call `SpawnAssemblyInstructionFromDescription` to spawn the assembly description and all involved components
3. **Measure reference frames** to correct component poses in the scene
4. **Recalculate assembly instructions** using `CalculateAssemblyInstructions` to update assembly transformations
5. **Align frames:** The 'assembly' frame (moving) and 'target' frame (static) are published on the tf_static topic. Align these frames to achieve the desired assembly transformation


## 7. Project Structure

```
ros_assembly_manager/
├── assembly_manager/              # Main assembly manager node
│   ├── assembly_manager/
│   ├── launch/
│   └── test/
├── assembly_scene_publisher/      # Central scene management node
│   ├── assembly_scene_publisher/
│   │   ├── py_modules/           # Core modules
│   │   └── testing/
│   ├── launch/
│   └── test/
├── assembly_scene_viewer/         # PyQt6-based scene visualization GUI
│   ├── assembly_scene_viewer/
│   │   └── py_modules/
│   └── test/
├── assembly_manager_interfaces/   # ROS2 message and service definitions
│   ├── msg/                       # Custom messages
│   └── srv/                       # Custom services
├── moveit_component_spawner/      # MoveIt2 planning scene interface
├── documentation/                 # Documentation and examples
└── requirements.txt               # Python package dependencies
```

## 8. Key Modules

- **AssemblyScene**: Main scene management with transformation calculations
- **AssemblySceneAnalyzer**: Query and analyze scene components, frames, and constraints
- **AssemblySceneModifier**: Modify scene components and their properties
- **FrameConstraintsHandler**: Manage and process frame constraints
- **GeometryFunctions**: Geometric calculations for assembly planning
- **STLViewerWidget**: 3D visualization of components using VTK

## 9. To-Do Items

1. Implement rosdep-based installation for better system integration
2. Add support for additional 3D model formats beyond STL
