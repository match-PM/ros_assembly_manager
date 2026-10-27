#!/usr/bin/env python3

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class AssemblyManagerNode(Node):
    """
    Compatibility node for the historic assembly-manager process.

    Assembly services are implemented by ``assembly_scene_publisher``. The
    node is intentionally kept so existing launch files and process monitors
    that expect an ``assembly_manager`` executable continue to work.
    """

    def __init__(self):
        super().__init__('assembly_manager')
        self.get_logger().info(
            'Assembly manager started; services are provided by '
            'assembly_scene_publisher.'
        )


def main(args=None):
    rclpy.init(args=args)
    node = AssemblyManagerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
