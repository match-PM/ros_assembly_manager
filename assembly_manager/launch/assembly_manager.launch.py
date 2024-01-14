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
    object_spawner_manager = Node(
        name="object_spawner_manager",
        package="object_spawner_manager",
        executable="object_spawner_manager.py",
        #namespace = '',
        parameters=[{"use_sim_time": sim_time},
        ],
        emulate_tty=True
    )
    
    object_topic_publisher = Node(
        name="object_spawner_publisher",
        package="object_spawner_manager",
        executable="spawn_object_tf.py",
        #namespace = '',
        parameters=[{"use_sim_time": sim_time},
        ],
        emulate_tty=True
    )

    moveit_object_publisher = Node(
        name="object_spawner_moveit",
        package="object_spawner_manager",
        executable="spawn_object_in_moveit_V",
        parameters=[{"use_sim_time": sim_time},
        ],
        emulate_tty=True
    )

    ld.add_action(object_spawner_manager)
    ld.add_action(object_topic_publisher)
    ld.add_action(moveit_object_publisher)

    return ld

if __name__ == '__main__':
    # ls = LaunchService(argv=argv, debug=True)  # Use this instead to get more debug messages.

    ls = LaunchService(argv=sys.argv[1:])
    ls.include_launch_description(generate_launch_description())
    print("this is a test")
    sys.exit(ls.run())
    