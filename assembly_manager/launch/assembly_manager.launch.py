import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchService
from moveit_configs_utils.launches import generate_move_group_launch
from launch_ros.actions import Node
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))  # noqa

def generate_launch_description():

    # Define Launch Description
    ld = LaunchDescription()
    sim_time = True
    assembly_manager = Node(
        name="assembly_manager",
        package="assembly_manager",
        executable="assembly_manager",
        #namespace = '',
        parameters=[{"use_sim_time": sim_time},
        ],
        emulate_tty=True
    )
    
    assembly_scene_publisher = Node(
        name="assembly_scene_publisher",
        package="assembly_scene_publisher",
        executable="assembly_scene_publisher",
        #namespace = '',
        #parameters=[{"use_sim_time": sim_time},],
        emulate_tty=True
    )

    moveit_component_spawner = Node(
        name="moveit_component_spawner",
        package="moveit_component_spawner",
        executable="moveit_component_spawner",
        parameters=[{"use_sim_time": sim_time},
        ],
        emulate_tty=True
    )

    ld.add_action(assembly_manager)
    ld.add_action(assembly_scene_publisher)
    ld.add_action(moveit_component_spawner)

    return ld

# if __name__ == '__main__':
#     # ls = LaunchService(argv=argv, debug=True)  # Use this instead to get more debug messages.

#     ls = LaunchService(argv=sys.argv[1:])
#     ls.include_launch_description(generate_launch_description())
#     print("this is a test")
#     sys.exit(ls.run())
    