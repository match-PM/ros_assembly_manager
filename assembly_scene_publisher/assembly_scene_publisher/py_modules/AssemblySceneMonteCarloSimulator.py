from assembly_scene_publisher.py_modules.AssemblySceneAnalyzer import AssemblySceneAnalyzer
from rclpy.node import Node
import assembly_manager_interfaces.msg as ami_msg
import assembly_manager_interfaces.srv as ami_srv
import assembly_manager_interfaces.action as ami_action
from typing import Any
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
import time
from copy import deepcopy
from geometry_msgs.msg import Transform, Vector3, Quaternion, Pose
from assembly_scene_publisher.py_modules.geometry_functions import multiply_ros_transforms, inverse_ros_transform
from ament_index_python.packages import get_package_share_directory

import numpy as np
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Pose, Transform
from assembly_scene_publisher.py_modules.AssemblyScene import AssemblyManagerScene

class MonteCarloSimulationParameters:
    def __init__(self, num_simulations: int, param_file: str):
        self.num_simulations = num_simulations
        self.param_file = param_file

class AssemblySceneMonteCarloSimulator:
    def __init__(self, assembly_manager_scene: AssemblyManagerScene):
        
        self.node = assembly_manager_scene.node
        self.assembly_manager_scene = assembly_manager_scene
        self._current_scene_file_name = None


    def execute_monte_carlo_simulation(self, simulation_params: MonteCarloSimulationParameters):
        """Execute a single Monte Carlo simulation with the given parameters."""
        # TODO: Implement the actual Monte Carlo simulation logic
        pass

    def save_recent_scene(self):
        """Save the current assembly scene to a file."""
        path = get_package_share_directory('assembly_scene_publisher') + "/scenes/tmp_scene"

        success, full_path = self.assembly_manager_scene.save_scene_to_file(path)
        if not success:
            raise Exception("Failed to save the current scene for Monte Carlo simulation")
        
        self._current_scene_file_name = full_path

    def load_recent_scene(self):
        """Load an assembly scene from a file."""
        self.assembly_manager_scene.load_scene_from_file(self._current_scene_file_name)

    async def execute_action(self, goal_handle: Any) -> ami_action.MonteCarloSimulation.Result:
        """Callback for the Monte Carlo simulation action server."""
        goal = goal_handle.request
        
        self.node.get_logger().info(f"Starting Monte Carlo simulation with {goal.num_simulations} simulations")
        
        # Create result
        result = ami_action.MonteCarloSimulation.Result()
        
        simulation_params = MonteCarloSimulationParameters(num_simulations=goal.num_simulations, param_file=goal.param_file)

        try:
            self.save_recent_scene()
            # Run simulations
            for i in range(goal.num_simulations):
                # Check if cancel was requested
                if goal_handle.is_cancel_requested:
                    self.node.get_logger().info("Monte Carlo simulation cancelled")
                    goal_handle.cancel()
                    result.success = False
                    result.message = "Simulation cancelled"
                    return result
                
                # Execute the simulator
                self.execute_monte_carlo_simulation(simulation_params=simulation_params)
                
                # Send feedback
                feedback = ami_action.MonteCarloSimulation.Feedback()
                feedback.simulations_completed = i + 1
                feedback.total_simulations = goal.num_simulations
                goal_handle.publish_feedback(feedback)
                
                self.node.get_logger().info(f"Completed simulation {i + 1}/{goal.num_simulations}")
            
            # Simulation completed successfully
            goal_handle.succeed()
            result.success = True
            result.message = f"Completed {goal.num_simulations} simulations successfully"
            self.load_recent_scene()  # Load the original scene back after simulations

        except Exception as e:
            self.node.get_logger().error(f"Error during Monte Carlo simulation: {e}")
            goal_handle.abort()
            result.success = False
            result.message = f"Error: {str(e)}"
        
        return result
