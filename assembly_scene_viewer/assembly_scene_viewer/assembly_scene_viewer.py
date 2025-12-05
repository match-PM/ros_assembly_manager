import rclpy
from rclpy.node import Node
from PyQt6.QtWidgets import QApplication
import sys
from rclpy.executors import MultiThreadedExecutor
from rosidl_runtime_py.convert import message_to_ordereddict, get_message_slot_types
from rosidl_runtime_py.set_message import set_message_fields
from rosidl_runtime_py.utilities import get_message, get_service, get_interface
from rqt_py_common import message_helpers
from threading import Thread 

from assembly_scene_viewer.py_modules.AssemblySceneViewer import AssemblySceneViewerMainWindow

class TfViewerNode(Node):

    def __init__(self):
        super().__init__('assembly_scene_viewer_node')
        self.get_logger().info('Assembly Scene Viewer Node has been started.')

        self.qt_window = AssemblySceneViewerMainWindow(self)
        
def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor(num_threads=6) 

    app = QApplication(sys.argv)

    tf_viewer_node = TfViewerNode()
    executor.add_node(tf_viewer_node)

    thread = Thread(target=executor.spin)
    thread.start()
    
    try:
        tf_viewer_node.qt_window.show()
        sys.exit(app.exec())

    finally:
        tf_viewer_node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()
    