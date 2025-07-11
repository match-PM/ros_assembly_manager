cmake_minimum_required(VERSION 3.8)
project(assembly_manager_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SpawnObject.srv"
  "srv/DestroyObject.srv"
  "srv/CreateRefPlane.srv"
  "srv/CreateAssemblyInstructions.srv"
  "srv/CreateRefFrame.srv"
  "srv/DeleteRefFrame.srv"
  "srv/GetScene.srv"
  "srv/ModifyPoseAbsolut.srv"
  "srv/ModifyPoseRelative.srv"
  "srv/ModifyPoseFromFrame.srv"
  "srv/ChangeParentFrame.srv"
  "srv/SetCollisionChecking.srv"
  "srv/CalculateAssemblyInstructions.srv"
  "srv/CreateAxis.srv"
  "srv/SpawnComponentFromDescription.srv"
  "srv/CreateAssemblyInstructionFromDescription.srv"
  "srv/SpawnFramesFromDescription.srv"
  "srv/FramesForComponent.srv"
  "srv/ClearScene.srv"

  "msg/AssemblyInstruction.msg"
  "msg/Object.msg"
  "msg/ObjectScene.msg"
  "msg/RefFrame.msg"
  "msg/Plane.msg"
  "msg/Axis.msg"
  "msg/ConstraintPlanes.msg"
  "msg/FrConstraintCentroid.msg"
  "msg/FrConstraints.msg"
  "msg/FrConstraintOrthogonal.msg"
  "msg/FrConstraintInPlane.msg"
  DEPENDENCIES geometry_msgs std_msgs# Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
