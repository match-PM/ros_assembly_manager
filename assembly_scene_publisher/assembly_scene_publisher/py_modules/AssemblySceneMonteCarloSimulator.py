from importlib_metadata import distribution

from assembly_scene_publisher.py_modules.AssemblySceneAnalyzer import AssemblySceneAnalyzer
from rclpy.node import Node
import assembly_manager_interfaces.msg as ami_msg
import assembly_manager_interfaces.srv as ami_srv
import assembly_manager_interfaces.action as ami_action
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from typing import Any, Dict
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
import time
from copy import deepcopy
from geometry_msgs.msg import Transform, Vector3, Quaternion, Pose
from assembly_scene_publisher.py_modules.geometry_functions import multiply_ros_transforms, inverse_ros_transform
from ament_index_python.packages import get_package_share_directory
import yaml
import numpy as np
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Pose, Transform
from assembly_scene_publisher.py_modules.AssemblyScene import AssemblyManagerScene
from assembly_scene_publisher.py_modules.scene_errors import *
import os
from dataclasses import dataclass
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from dataclasses import dataclass, asdict
from typing import Dict
from numpy import mean, std
import random

def pose_to_dict(pose:Pose)->dict:
    return {
        "position": {
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "z": float(pose.position.z)
        },
        "orientation": {
            "x": float(pose.orientation.x),
            "y": float(pose.orientation.y),
            "z": float(pose.orientation.z),
            "w": float(pose.orientation.w)
        }
    }

def pose_dict_to_pose(pose_dict: dict) -> Pose:
    pose = Pose()
    pose.position.x = pose_dict["position"]["x"]
    pose.position.y = pose_dict["position"]["y"]
    pose.position.z = pose_dict["position"]["z"]
    pose.orientation.x = pose_dict["orientation"]["x"]
    pose.orientation.y = pose_dict["orientation"]["y"]
    pose.orientation.z = pose_dict["orientation"]["z"]
    pose.orientation.w = pose_dict["orientation"]["w"]
    return pose

def compare_poses(pose1: Pose, pose2: Pose, position_tolerance: float = 1e-6, orientation_tolerance: float = 1e-6) -> bool:
    """Compare two poses with given tolerances for position and orientation."""
    position_close = (abs(pose1.position.x - pose2.position.x) <= position_tolerance and
                      abs(pose1.position.y - pose2.position.y) <= position_tolerance and
                      abs(pose1.position.z - pose2.position.z) <= position_tolerance)
    
    orientation_close = (abs(pose1.orientation.x - pose2.orientation.x) <= orientation_tolerance and
                         abs(pose1.orientation.y - pose2.orientation.y) <= orientation_tolerance and
                         abs(pose1.orientation.z - pose2.orientation.z) <= orientation_tolerance and
                         abs(pose1.orientation.w - pose2.orientation.w) <= orientation_tolerance)
    
    return position_close and orientation_close

   
    
class MonteCarloSimulationParameters:
    def __init__(self, num_simulations: int, param_file: str):
        self.num_simulations = num_simulations
        self._instruction: ami_msg.AssemblyInstruction = None
        self._instruction_id=None
        self.param_file = param_file
        self.tol_unit = "mm"
        self.vision_mes_std_dev = 0.0
        self.laser_mes_std_dev = 0.0
        self.vision_active = True
        self.laser_active = True
        self.load_from_file(param_file)
        self.exclude_frames_vision:list[str] = []
        self.exclude_frames_laser:list[str] = []
        self.exclude_components:list[str] = []

    def set_instruction(self, instruction: ami_msg.AssemblyInstruction):
        self._instruction = instruction

    def get_instruction(self) -> ami_msg.AssemblyInstruction:
        if self._instruction is None:
            raise Exception("Instruction must be set before getting it")
        return self._instruction
    
    def get_instruction_id(self) -> str:
        if self._instruction_id is None:
            raise Exception("Instruction ID must be set before getting it")
        return self._instruction_id

    def load_from_file(self, file_path: str):
        """
        mc_parameters:
        num_simulations: 1000
        instruction_id: "Description_Glas_Platelet_Paper_UFC_Paper"
        exclude_components: []
        tol_unit: "mm"
        vision:
            active: true
            mes_std_dev: 0.01
            exclude_frames: []
        laser:
            active: true
            mes_std_dev: 0.005
            exclude_frames: []
        """ 
        try:
            with open(file_path, 'r') as f:
                data = yaml.safe_load(f)
            self.num_simulations = data['mc_parameters']['num_simulations']
            self._instruction_id = data['mc_parameters']['instruction_id']
            self.vision_active = data['mc_parameters']['vision'].get('active', True)  # Default to active if not specified
            self.vision_mes_std_dev = data['mc_parameters']['vision']['mes_std_dev']
            self.laser_mes_std_dev = data['mc_parameters']['laser']['mes_std_dev']
            self.laser_active = data['mc_parameters']['laser'].get('active', True)  # Default to active if not specified
            self.tol_unit = data['mc_parameters'].get('tol_unit', 'mm')
            self.exclude_frames_vision = data['mc_parameters']['vision'].get('exclude_frames', [])
            self.exclude_frames_laser = data['mc_parameters']['laser'].get('exclude_frames', [])
            self.exclude_components = data['mc_parameters'].get('exclude_components', [])
        except Exception as e:
            raise Exception(f"Failed to load Monte Carlo simulation parameters from file: {e}")


class ToleranceApplicator:


    @staticmethod
    def gen_value_gauss(std:float):
        value = random.gauss(0, std)
        return value
    
    @staticmethod
    def gen_gausss_radius(std:float):
        r = np.abs(np.random.normal(0, std)) 
        theta = np.random.uniform(0, 2 * np.pi)

        x = r * np.cos(theta)
        y = r * np.sin(theta)
        return x, y

    @staticmethod
    def gen_gaussian_2d(std: float):
        x, y = np.random.normal(0, std, size=2)
        return x, y
    
    @staticmethod
    def get_multiplier_for_tol_unit(tol_unit: str) -> float:
        if tol_unit == "mm":
            return 0.001
        elif tol_unit == "cm":
            return 0.01
        elif tol_unit == "um":
            return 0.000001
        elif tol_unit == "m":
            return 1.0
        else:
            raise Exception(f"Unsupported tolerance unit: {tol_unit}")
    
    @staticmethod
    def apply_tol_to_component_frames(component_frames: list[ami_msg.RefFrame], simulation_params: MonteCarloSimulationParameters) -> None:
        for frame in component_frames:
            ToleranceApplicator.apply_tol_to_ref_frame(frame, simulation_params)

    @staticmethod
    def apply_tol_to_ref_frame(ref_frame: ami_msg.RefFrame, simulation_params: MonteCarloSimulationParameters) -> None:
        ToleranceApplicator.apply_tol_to_vision_frame(ref_frame, simulation_params)
        ToleranceApplicator.apply_tol_to_laser_frame(ref_frame, simulation_params)

    @staticmethod
    def apply_tol_to_vision_frame(ref_frame: ami_msg.RefFrame, simulation_params: MonteCarloSimulationParameters) -> None:
        if not ref_frame.properties.vision_frame_properties.is_vision_frame:
            return 
        
        if ref_frame.frame_name in simulation_params.exclude_frames_vision:
            return 
        
        if not simulation_params.vision_active:
            return
        
        vision_tolerance = simulation_params.vision_mes_std_dev
        multiplier = ToleranceApplicator.get_multiplier_for_tol_unit(simulation_params.tol_unit)
        
        if False:
            tol_value = ToleranceApplicator.gen_value_gauss(vision_tolerance) * multiplier
            # turn sign of tolerance value randomly to simulate both positive and negative measurement errors
            if random.choice([True, False]):
                tol_value = -tol_value

        if True:
            # radiant gauss
            x, y = ToleranceApplicator.gen_gaussian_2d(vision_tolerance * multiplier)
            # flip sign
            if random.choice([True, False]):
                x = -x
            if random.choice([True, False]):
                y = -y

        # Apply tolerance to vision frame position
        ref_frame.pose.position.x += x
        ref_frame.pose.position.y += y

    @staticmethod
    def apply_tol_to_laser_frame(ref_frame: ami_msg.RefFrame, simulation_params: MonteCarloSimulationParameters) -> None:
        if not ref_frame.properties.laser_frame_properties.is_laser_frame:
            return 
        
        if ref_frame.frame_name in simulation_params.exclude_frames_laser:
            return 
        
        if not simulation_params.laser_active:
            return
        
        laser_tolerance = simulation_params.laser_mes_std_dev   
        multiplier = ToleranceApplicator.get_multiplier_for_tol_unit(simulation_params.tol_unit)
        
        # Apply laser tolerance to z position
        tol_value = ToleranceApplicator.gen_value_gauss(laser_tolerance) * multiplier
        if random.choice([True, False]):
            tol_value = -tol_value
        ref_frame.pose.position.z += tol_value
        

@dataclass
class ComponentStatistics:
    """Statistics for a single component (x, y, z or roll, pitch, yaw)."""
    values: list[float]
    mean: float
    std_dev: float

@dataclass
class ResultValues:
    """Optimized result values with grouped position and orientation statistics."""
    position: Dict[str, ComponentStatistics]  # 'x', 'y', 'z'
    orientation: Dict[str, ComponentStatistics]  # 'roll', 'pitch', 'yaw'

    @classmethod
    def from_distributions(cls, transform_distributions: list[Pose]) -> 'ResultValues':
        """Create ResultValues from transform distributions."""
        
        mult = 1e6 # convert from meters to micrometers for more intuitive statistics
        # Extract position values
        x = [t.position.x * mult for t in transform_distributions]
        y = [t.position.y * mult for t in transform_distributions]
        z = [t.position.z * mult for t in transform_distributions]

        # Extract rotation values
        rolls, pitches, yaws = [], [], []
        for t in transform_distributions:
            r = R.from_quat([t.orientation.x, t.orientation.y, t.orientation.z, t.orientation.w])
            roll, pitch, yaw = r.as_euler('xyz', degrees=True)
            rolls.append(roll)
            pitches.append(pitch)
            yaws.append(yaw)

        # Create statistics objects
        position = {
            'x': ComponentStatistics(x, float(np.mean(x)), float(np.std(x))),
            'y': ComponentStatistics(y, float(np.mean(y)), float(np.std(y))),
            'z': ComponentStatistics(z, float(np.mean(z)), float(np.std(z)))
        }
        
        orientation = {
            'roll': ComponentStatistics(rolls, float(np.mean(rolls)), float(np.std(rolls))),
            'pitch': ComponentStatistics(pitches, float(np.mean(pitches)), float(np.std(pitches))),
            'yaw': ComponentStatistics(yaws, float(np.mean(yaws)), float(np.std(yaws)))
        }
        
        return cls(position=position, orientation=orientation)

    def to_dict(self, include_distributions: bool = False) -> dict:
        """Convert to dictionary format for serialization."""
        result = {}
        
        for component_type, components in [('position', self.position), ('orientation', self.orientation)]:
            for name, stats in components.items():
                key_prefix = f"{component_type}_{name}" if component_type != 'position' else name
                result[f"mean_{key_prefix}"] = stats.mean
                result[f"std_dev_{key_prefix}"] = stats.std_dev
                if include_distributions:
                    result[f"{component_type}_{name}_values"] = stats.values
        
        return result

    def get_summary(self) -> dict:
        """Get summary statistics without distribution values."""
        return self.to_dict(include_distributions=False)

@dataclass
class InstructionResultValues:
    def __init__(self, 
                 assembly_results: ResultValues = None,
                 target_results: ResultValues = None,
                 transform_results: ResultValues = None):
        self.assembly_results = assembly_results
        self.target_results = target_results
        self.transform_results = transform_results

    def set_assembly_distributions(self, assembly_distributions: list[Pose]):
        self.assembly_results = ResultValues.from_distributions(assembly_distributions)
    
    def set_target_distributions(self, target_distributions: list[Pose]):
        self.target_results = ResultValues.from_distributions(target_distributions)

    def set_transform_distributions(self, transform_distributions: list[Pose]):
        self.transform_results = ResultValues.from_distributions(transform_distributions)

    def to_dict(self, include_distributions: bool = False) -> dict:
        return {
            "assembly_results": self.assembly_results.to_dict(include_distributions) if self.assembly_results else None,
            "target_results": self.target_results.to_dict(include_distributions) if self.target_results else None,
            "transform_results": self.transform_results.to_dict(include_distributions) if self.transform_results else None
        }
class MonteCarloSimulationResults:
    POSES_FILE_NAME = "poses.yaml"
    RESULTS_FILE_NAME = "results.yaml"

    def __init__(self, instruction: ami_msg.AssemblyInstruction,
                 file_path:str,
                 logger: Any = None):
        
        self.logger = logger
        self._file_path = file_path
        self._initial_assembly_pose: Pose = None
        self._initial_target_pose: Pose = None
        self.assembly_pose_distributions: list[Pose] = []
        self.target_pose_distributions: list[Pose] = []
        self._transform_distributions: list[Transform] = []
        self._tick_times: list[float] = []
        self._total_simulation_time: float = 0.0
        self._failure_count: int = 0
        self.prev_avg_tick_time: float = None
        self.instruction: ami_msg.AssemblyInstruction = instruction
        self._file_name: str = os.path.basename(file_path)
        self._file_dir: str = os.path.dirname(file_path)
        self._results_dir: str = os.path.join(self._file_dir, f"results_{self._file_name.split('.')[0]}")
        self._iterations_ran: int = 0
        self._cached_result_values: InstructionResultValues = None  # Cache for result values to avoid recalculation
        self.load_prev_poses()  # Attempt to load previous poses if they exist, otherwise start with empty distributions
        self.load_prev_results()

    def get_iterations_ran(self) -> int:
        return self._iterations_ran
    
    def increment_iterations_ran(self):
        self._iterations_ran += 1

    def save_results_to_file(self):
        results_dict = self.get_results()

        #create a folder results if it doesn't exist
        os.makedirs(self._results_dir, exist_ok=True)

        try:
            with open(self._file_path, 'w') as f:
                yaml.dump(results_dict, f, default_flow_style=False)
        except Exception as e:
            raise Exception(f"Failed to save Monte Carlo simulation results to file: {e}")
    
    def load_results_from_file(self):
        try:
            with open(self._file_path, 'r') as f:
                data = yaml.safe_load(f)
            self.assembly_pose_distributions = data['assembly_pose_distributions']
            self.target_pose_distributions = data['target_pose_distributions']
            self._transform_distributions = data['transform_distributions']
        except Exception as e:
            raise Exception(f"Failed to load Monte Carlo simulation results from file: {e}")
        
    def get_failure_count(self) -> int:
        return self._failure_count
    
    def increment_failure_count(self):
        self._failure_count += 1
    
    def set_initial_assembly_pose(self, pose: Pose):
        if self._initial_assembly_pose is not None:
            if not compare_poses(self._initial_assembly_pose, pose):
                raise Exception(f"Initial assembly pose is already set and does not match the new pose being set - existing: {self._initial_assembly_pose}, new: {pose}")
        self._initial_assembly_pose = pose

    def get_initial_assembly_pose(self) -> Pose:
        return self._initial_assembly_pose
    
    def set_initial_target_pose(self, pose: Pose):
        if self._initial_target_pose is not None:
            if not compare_poses(self._initial_target_pose, pose):
                raise Exception(f"Initial target pose is already set and does not match the new pose being set - existing: {self._initial_target_pose}, new: {pose}")
        self._initial_target_pose = pose

    def get_initial_target_pose(self) -> Pose:
        return self._initial_target_pose

    def add_to_assembly_pose_distribution(self, pose: Pose):
        self.assembly_pose_distributions.append(pose)

    def add_to_target_pose_distribution(self, pose: Pose):
        self.target_pose_distributions.append(pose)

    def add_tick_time(self, tick_time: float):
        self._tick_times.append(tick_time)

    def set_total_simulation_time(self, total_time: float, append = True):
        if append:
            self._total_simulation_time += total_time
        else:
            self._total_simulation_time = total_time

    def get_total_simulation_time(self) -> float:
        return self._total_simulation_time
    
    def _calc_transform_distribution(self)-> InstructionResultValues:
        """Calculate transform distribution and cache the result."""
        # Return cached result if available
        if self._cached_result_values is not None:
            return self._cached_result_values
        
        if len(self.assembly_pose_distributions) != len(self.target_pose_distributions):
            raise Exception("Assembly and target pose distributions must have the same number of samples to calculate transform distribution")
        
        self._transform_distributions: list[Pose] = []

        for assembly_pose, target_pose in zip(self.assembly_pose_distributions, self.target_pose_distributions):
            # Calculate the transform from assembly to target
            transform = multiply_ros_transforms(inverse_ros_transform(assembly_pose,output_type=Pose), target_pose, output_type=Pose)
            self._transform_distributions.append(transform)
        
        result_values = InstructionResultValues()

        result_values.set_assembly_distributions(self.assembly_pose_distributions)
        result_values.set_target_distributions(self.target_pose_distributions)
        result_values.set_transform_distributions(self._transform_distributions)

        # Cache and return the result
        self._cached_result_values = result_values
        return self._cached_result_values

    def get_results(self) -> dict:
        """Return the Monte Carlo simulation results as a dictionary."""
        result_values = self._calc_transform_distribution()  # Calculate transform distribution

        avg_tick_time = np.mean(self._tick_times) if self._tick_times else 0.0
        
        return {
            "failure_count": self.get_failure_count(),
            "total_simulation_time": self.get_total_simulation_time(),
            "avg_tick_time": str(round(avg_tick_time, 3)),
            "total_iterations": self.get_iterations_ran(),
            "result_values": result_values.to_dict(include_distributions=False)
        }

    def save_poses(self):
        """Save the assembly and target pose distributions to separate files."""
        # create a folder results if it doesn't exist
        os.makedirs(self._results_dir, exist_ok=True)

        # create dict including assembly and target pose distributions
        poses_dict = {
            "initial_assembly_pose": pose_to_dict(self._initial_assembly_pose),
            "initial_target_pose": pose_to_dict(self._initial_target_pose),
            "assembly_pose_distributions": [pose_to_dict(pose) for pose in self.assembly_pose_distributions],
            "target_pose_distributions": [pose_to_dict(pose) for pose in self.target_pose_distributions]
            }
        try:
            with open(os.path.join(self._results_dir, self.POSES_FILE_NAME), 'w') as f:
                yaml.dump(poses_dict, f, default_flow_style=False)
        except Exception as e:
            raise Exception(f"Failed to save Monte Carlo simulation poses to file: {e}")

    def save_results(self):
        """Save the full Monte Carlo simulation results to a file."""
        results_dict = self.get_results()
        # create a folder results if it doesn't exist
        os.makedirs(self._results_dir, exist_ok=True)

        try:
            with open(os.path.join(self._results_dir, f"full_results.yaml"), 'w') as f:
                yaml.dump(results_dict, f, default_flow_style=False)
        except Exception as e:
            raise Exception(f"Failed to save Monte Carlo simulation results to file: {e}")
    
    def load_prev_poses(self):
        """Load Monte Carlo simulation poses from a file."""
        # check if file exists, if not, return
        if not os.path.exists(os.path.join(self._results_dir, self.POSES_FILE_NAME)):
            return
        
        try:
            with open(os.path.join(self._results_dir, self.POSES_FILE_NAME), 'r') as f:
                data = yaml.safe_load(f)
           
            loaded_initial_assembly_pose = pose_dict_to_pose(data['initial_assembly_pose'])
            loaded_initial_target_pose = pose_dict_to_pose(data['initial_target_pose'])

            self.set_initial_assembly_pose(loaded_initial_assembly_pose)
            self.set_initial_target_pose(loaded_initial_target_pose)

            self.assembly_pose_distributions = [pose_dict_to_pose(pose) for pose in data['assembly_pose_distributions']]
            self.target_pose_distributions = [pose_dict_to_pose(pose) for pose in data['target_pose_distributions']]
            self._iterations_ran = len(self.assembly_pose_distributions)  # Assuming each pose corresponds to one iteration

        except Exception as e:
            raise Exception(f"Failed to load Monte Carlo simulation poses from file: {e}")
    
    def load_prev_results(self):
        """Load Monte Carlo simulation results from a file."""
        if not os.path.exists(os.path.join(self._results_dir, self.RESULTS_FILE_NAME)):
            return
        try:
            with open(os.path.join(self._results_dir, self.RESULTS_FILE_NAME), 'r') as f:
                data = yaml.safe_load(f)

            self._failure_count = data['failure_count']
            self.set_total_simulation_time(data['total_simulation_time'], append=True)
            self._iterations_ran = data['total_iterations']
            self.prev_avg_tick_time = float(data['avg_tick_time'])
        except Exception as e:
            raise Exception(f"Failed to load Monte Carlo simulation results from file: {e}")


    def plot_distributions(self, results: ResultValues = None):     
        """Plot the position and orientation distributions from Monte Carlo simulation results."""
        # Use cached results if available, otherwise calculate them
        if results is None:
            results = self._cached_result_values

        if results is None:
            raise ValueError("No results available to plot")

        # Extract values from the optimized ResultValues dataclass
        transform_x_values = results.position['x'].values
        transform_y_values = results.position['y'].values
        transform_z_values = results.position['z'].values
        roll_values = results.orientation['roll'].values
        pitch_values = results.orientation['pitch'].values
        yaw_values = results.orientation['yaw'].values
        
        # Center the values around zero
        transform_x_centered = [x - results.position['x'].mean for x in transform_x_values]
        transform_y_centered = [y - results.position['y'].mean for y in transform_y_values]
        transform_z_centered = [z - results.position['z'].mean for z in transform_z_values]
        roll_centered = [r - results.orientation['roll'].mean for r in roll_values]
        pitch_centered = [p - results.orientation['pitch'].mean for p in pitch_values]
        yaw_centered = [y - results.orientation['yaw'].mean for y in yaw_values]
        
        # Plot the distributions
        plt.figure(figsize=(12, 8))
        
        plt.subplot(2, 3, 1)
        plt.hist(transform_x_centered, bins=30, color='blue', alpha=0.7)
        plt.title(f'Transform X Distribution\n(μ={results.position["x"].mean:.4f}, σ={results.position["x"].std_dev:.4f})')
        plt.xlabel('Transform X (centered)')
        plt.ylabel('Frequency')
        
        plt.subplot(2, 3, 2)
        plt.hist(transform_y_centered, bins=30, color='green', alpha=0.7)
        plt.title(f'Transform Y Distribution\n(μ={results.position["y"].mean:.4f}, σ={results.position["y"].std_dev:.4f})')
        plt.xlabel('Transform Y (centered)')
        plt.ylabel('Frequency')
        
        plt.subplot(2, 3, 3)
        plt.hist(transform_z_centered, bins=30, color='red', alpha=0.7)
        plt.title(f'Transform Z Distribution\n(μ={results.position["z"].mean:.4f}, σ={results.position["z"].std_dev:.4f})')
        plt.xlabel('Transform Z (centered)')
        plt.ylabel('Frequency')
        
        plt.subplot(2, 3, 4)
        plt.hist(roll_centered, bins=30, color='purple', alpha=0.7)
        plt.title(f'Roll Distribution\n(μ={results.orientation["roll"].mean:.4f}, σ={results.orientation["roll"].std_dev:.4f})')
        plt.xlabel('Roll (centered rad)')
        plt.ylabel('Frequency')
        
        plt.subplot(2, 3, 5)
        plt.hist(pitch_centered, bins=30, color='orange', alpha=0.7)
        plt.title(f'Pitch Distribution\n(μ={results.orientation["pitch"].mean:.4f}, σ={results.orientation["pitch"].std_dev:.4f})')
        plt.xlabel('Pitch (centered rad)')
        plt.ylabel('Frequency')
        
        plt.subplot(2, 3, 6)
        plt.hist(yaw_centered, bins=30, color='cyan', alpha=0.7)
        plt.title(f'Yaw Distribution\n(μ={results.orientation["yaw"].mean:.4f}, σ={results.orientation["yaw"].std_dev:.4f})')
        plt.xlabel('Yaw (centered rad)')
        plt.ylabel('Frequency')
        
        plt.tight_layout()
        # Save the figure to results directory
        output_path = os.path.join(self._results_dir, 'distributions.png')
        os.makedirs(self._results_dir, exist_ok=True)
        plt.savefig(output_path, dpi=150)
        self.logger.info(f"Distribution plot saved to {output_path}")
        plt.close()

class AssemblySceneMonteCarloSimulator:
    def __init__(self, assembly_manager_scene: AssemblyManagerScene):
        
        self.node = assembly_manager_scene.node
        self.assembly_manager_scene = assembly_manager_scene
        self._current_scene_file_name = None
    

    def monte_carlo_simulation_tick(self, simulation_params: MonteCarloSimulationParameters,
                                    simulation_results: MonteCarloSimulationResults) -> None:
        """Execute a single Monte Carlo simulation with the given parameters."""
        # TODO: Implement the actual Monte Carlo simulation logic

        # start tick time
        start_time = time.time()
        # Modify frames
        component_1 = simulation_params.get_instruction().component_1
        component_2 = simulation_params.get_instruction().component_2

        c1_frames = []
        c2_frames = []
        vision_frames_c1 = self.assembly_manager_scene.assembly_scene_analyzer.get_vision_frames_of_component(component_1)
        vision_frames_c2 = self.assembly_manager_scene.assembly_scene_analyzer.get_vision_frames_of_component(component_2)
        c1_frames.extend(vision_frames_c1)
        c2_frames.extend(vision_frames_c2)
        laser_frames_c1 = self.assembly_manager_scene.assembly_scene_analyzer.get_laser_frames_of_component(component_1) 
        laser_frames_c2 = self.assembly_manager_scene.assembly_scene_analyzer.get_laser_frames_of_component(component_2)
        c1_frames.extend(laser_frames_c1)
        c2_frames.extend(laser_frames_c2)

        if not (component_1 in simulation_params.exclude_components):
            self.node.get_logger().info(f"Applying tolerance to component {component_1} frames: {[frame.frame_name for frame in c1_frames]}")
            ToleranceApplicator.apply_tol_to_component_frames(c1_frames, simulation_params)

        if not (component_2 in simulation_params.exclude_components):
            self.node.get_logger().info(f"Applying tolerance to component {component_2} frames: {[frame.frame_name for frame in c2_frames]}")
            ToleranceApplicator.apply_tol_to_component_frames(c2_frames, simulation_params)

        self.assembly_manager_scene.update_scene_with_constraints()
        try:
            self.assembly_manager_scene.calculate_assembly_transformation(simulation_params.get_instruction())
        
        except Exception as e:
            self.node.get_logger().error(f"FATAL ERROR during assembly transformation calculation: {e}")
            simulation_results.increment_failure_count()
            return

        assembly_frame = self.assembly_manager_scene.assembly_scene_analyzer.get_assembly_frame_for_instruction(simulation_params.get_instruction().id)  # Validate that the instruction has a valid frame
        target_frame = self.assembly_manager_scene.assembly_scene_analyzer.get_target_frame_for_instruction(simulation_params.get_instruction().id)  # Validate that the instruction has a valid target frame
        
        simulation_results.add_to_assembly_pose_distribution(assembly_frame.pose)
        simulation_results.add_to_target_pose_distribution(target_frame.pose)

        # end tick time
        end_time = time.time()
        tick_time = end_time - start_time
        simulation_results.add_tick_time(tick_time)
        simulation_results.increment_iterations_ran()

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

    def pre_simulation(self):
        """Handle any pre-simulation tasks such as saving the current scene."""
        self.save_recent_scene()
        self.start_time = time.time()

    async def execute_action(self, goal_handle: Any) -> ami_action.MonteCarloSimulation.Result:
        """Callback for the Monte Carlo simulation action server."""
        goal = goal_handle.request
        result = ami_action.MonteCarloSimulation.Result()
        self.node.get_logger().info(f"Starting Monte Carlo simulation with {goal.num_simulations} ticks.")

        try:
            self.pre_simulation()

            # Preparation
            simulation_params = MonteCarloSimulationParameters(num_simulations=goal.num_simulations, 
                                                            param_file=goal.param_file)

            instruction: ami_msg.AssemblyInstruction = self.assembly_manager_scene.assembly_scene_analyzer.get_assembly_instruction_by_name(simulation_params.get_instruction_id())  # Validate instruction name
            
            if not (self.assembly_manager_scene.assembly_scene_analyzer.check_component_exists(instruction.component_1) and
                    self.assembly_manager_scene.assembly_scene_analyzer.check_component_exists(instruction.component_2)):
                raise Exception("One or both components in the instruction do not exist in the current scene")
            
            simulation_params.set_instruction(instruction)

            monte_carlo_results = MonteCarloSimulationResults(instruction=instruction,
                                                                file_path=goal.param_file,
                                                                logger=self.node.get_logger())

            assembly_frame = self.assembly_manager_scene.assembly_scene_analyzer.get_assembly_frame_for_instruction(simulation_params.get_instruction().id)  # Validate that the instruction has a valid frame
            target_frame = self.assembly_manager_scene.assembly_scene_analyzer.get_target_frame_for_instruction(simulation_params.get_instruction().id)  # Validate that the instruction has a valid target frame
            
            monte_carlo_results.set_initial_assembly_pose(assembly_frame.pose)
            monte_carlo_results.set_initial_target_pose(target_frame.pose)   

            if monte_carlo_results.prev_avg_tick_time is not None:
                estimated_total_time = monte_carlo_results.prev_avg_tick_time * goal.num_simulations
                self.node.get_logger().info(f"Estimated total simulation time based on previous runs: {estimated_total_time:.2f} seconds")

            # Run simulations
            for i in range(goal.num_simulations):
                # Check if cancel was requested
                if goal_handle.is_cancel_requested:
                    self.node.get_logger().info("Monte Carlo simulation cancelled")
                    goal_handle.canceled()
                    result.success = False
                    result.message = "Simulation cancelled"
                    self.post_simulation(monte_carlo_results)  # Handle any post-simulation tasks such as saving results
                    return result
                
                # Execute the simulator
                self.monte_carlo_simulation_tick(simulation_params=simulation_params, 
                                                 simulation_results=monte_carlo_results)
                
                self.load_recent_scene()  # Load the original scene back after each tick to ensure independence of simulations

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

            self.post_simulation(monte_carlo_results)  # Handle any post-simulation tasks such as saving results

        except (Exception,
                AssemblyInstructionNotFoundError) as e:
            self.node.get_logger().error(f"Error during Monte Carlo simulation: {e}")
            goal_handle.abort()
            result.success = False
            result.message = f"Error: {str(e)}"
        
        return result

    def post_simulation(self, simulation_results: MonteCarloSimulationResults):
        """Handle post-simulation tasks such as saving results to a file."""
        self.load_recent_scene()  # Load the original scene back after simulations
        end_time = time.time()
        total_time = end_time - self.start_time
        simulation_results.set_total_simulation_time(total_time, append=True)
        simulation_results.save_poses()
        simulation_results.save_results()
        self.node.get_logger().info(f"Starting to plot distributions...")
        simulation_results.plot_distributions(simulation_results._cached_result_values.target_results)
        
    def goal_callback(self, goal_request: ami_action.MonteCarloSimulation.Goal):
        #self.node.get_logger().info(f"Received goal: {str(goal_request)}")
        # Accept all goals
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.node.get_logger().warn("Cancel request received")
        return CancelResponse.ACCEPT

if __name__ == "__main__":
    pass

