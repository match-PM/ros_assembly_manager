import json
import os
import random
from ros_sequential_action_programmer.submodules.RosSequentialActionProgrammer import RosSequentialActionProgrammer
from rclpy.node import Node
import rclpy
from rclpy.executors import MultiThreadedExecutor
import statistics
from geometry_msgs.msg import Vector3, Quaternion
import numpy as np
from assembly_scene_publisher.py_modules.geometry_type_functions import quaternion_to_euler
from ros_sequential_action_programmer.submodules.rsap_modules.RsapConfig import ExecutionLog
import numpy as np
import time 
import datetime
from assembly_manager_interfaces.msg import ObjectScene
from geometry_msgs.msg import Pose
from ros_sequential_action_programmer.submodules.action_classes.ServiceAction import ServiceAction

from assembly_scene_publisher.py_modules.frame_constraints import get_constraint_frames_for_frame, get_identification_order
from assembly_scene_publisher.py_modules.scene_functions import get_frames_for_planes_of_component, get_ref_frame_by_name

class SimulationParameter():
    def __init__(self, std_camera:float,    # um
                 std_laser:float,           # um
                 num_iterations:int, 
                 approx_time:int,
                 results_name:str,
                 use_radiant_gauss:bool = True,
                comments:list[str] = []
                 ) -> None:
        
        self.std_camera = std_camera
        self.std_laser = std_laser
        self.num_iterations = num_iterations
        self.approx_time = approx_time
        self.results_name = results_name
        self.use_radiant_gauss = use_radiant_gauss
        self.total_iterations = 0
        self.comments = comments
        self.failure_count = 0

    def to_dict(self)->dict:
        return {
            "std_camera": self.std_camera,
            "std_laser": self.std_laser,
            "num_iterations": self.num_iterations,
            "approx_time": self.approx_time,
            "use_radiant_gauss": self.use_radiant_gauss,
            "total_iterations": self.total_iterations,
            "failure_count": self.failure_count,
            "comments": self.comments
        }
    def increment_failure_count(self):
        self.failure_count += 1
    
class TolMeasurment():
    SCOPE_FRAME = "assembly_frame_Description_Glas_6D_tol-1_UFC_6D_tol-1"
    TARGET_FRAME = 'target_frame_Description_Glas_6D_tol-1_UFC_6D_tol-1'
    COMPONENT = 'UFC_6D_tol-1'
    
    def __init__(self, ros_node:Node) -> None:
        self.ros_node = ros_node
        self.programmer = RosSequentialActionProgrammer(ros_node)

        # # Run R01
        # self.sim_parameters = SimulationParameter(std_camera        =   1,
        #                                           std_laser         =   0,
        #                                           num_iterations    =   500,
        #                                           approx_time       =   None,
        #                                           results_name      =   'R01',
        #                                           use_radiant_gauss =   False,
        #                                           comments          =   ["x und y equal in normal distribution",
        #                                                                    "cameras: x = 1um y = 1um; laser: z= 1um "
        #                                                                 ])
        
        # # Run R02
        # self.sim_parameters = SimulationParameter(std_camera        =   1,
        #                                           std_laser         =   0,
        #                                           num_iterations    =   500,
        #                                           approx_time       =   None,
        #                                           results_name      =   'R02',
        #                                           use_radiant_gauss =   True,
        #                                           comments          =   ["Testing of camera influence without laser influence",
        #                                                                     "Distribution of camera is according to radius with normal distribution",
        #                                                                     "Uses the ideal ref_points with laser points at the default position"])
        # Run R03
        self.sim_parameters = SimulationParameter(std_camera        =   0,
                                                  std_laser         =   10,
                                                  num_iterations    =   3,
                                                  approx_time       =   None,
                                                  results_name      =   'R03',
                                                  use_radiant_gauss =   True,
                                                  comments          =   ["Testing of laser influence without camera influence",
                                                                        "Uses the ideal ref_points with laser points at the default position"])

        use_mll     = True
        use_niklas  = False
        use_pm_room = False

        if use_mll:
            self.instruction_json = '/home/mll/ros2_ws/src/ros_assembly_manager/documentation/tolerance_analyses/SWASI_Exports/assemblies/Assembly_UFC_Glas_6D_tol.json'
            self.programmer.load_from_JSON('/home/mll/ros2_ws/src/ros_assembly_manager/documentation/tolerance_analyses/rsap_description.json')
            self.results_path = '/home/mll/ros2_ws/src/ros_assembly_manager/documentation/tolerance_analyses/logs'
            approx_time = 20
        elif use_niklas:
            self.programmer.load_from_JSON('/home/niklas/ros2_ws/src/ros_assembly_manager/documentation/tolerance_analyses/rsap_description.json')
            self.instruction_json = '/home/niklas/ros2_ws/src/ros_assembly_manager/documentation/tolerance_analyses/SWASI_Exports/assemblies/Assembly_UFC_Glas_6D_tol.json'
            self.results_path = '/home/niklas/ros2_ws/src/ros_assembly_manager/documentation/tolerance_analyses/logs'
            approx_time = 15
        elif use_pm_room:
            self.programmer.load_from_JSON('/home/niklas/ros2_ws/src/ros_assembly_manager/documentation/tolerance_analyses/rsap_description.json')
            self.instruction_json = '/home/niklas/ros2_ws/src/ros_assembly_manager/documentation/tolerance_analyses/SWASI_Exports/assemblies/Assembly_UFC_Glas_6D_tol.json'
            self.results_path = '/home/niklas/ros2_ws/src/ros_assembly_manager/documentation/tolerance_analyses/logs'
            approx_time = 15
        else:
            raise ValueError("No valid path for instruction json file")

        self.init_spawning_action()

        self.scene = None
        self.ros_node.create_subscription(ObjectScene, '/assembly_manager/scene', self.get_scene_callback, 10)
        
        self.results_calculated_poses:list[Pose] = []
        self.recalculation_action = None
        self.list_of_times = []
        print(f"Running {self.sim_parameters.num_iterations} iterations. The program will take approx {(self.sim_parameters.num_iterations*approx_time)/60} min to finish.")
        approx_finish_time = datetime.datetime.now() + datetime.timedelta(seconds=self.sim_parameters.num_iterations*approx_time)
        print(f"Approx finish time: {approx_finish_time}")
        self.file_name = f'poses_list_{self.sim_parameters.results_name}.json'

    def init_spawning_action(self):
        self.spawning_action = ServiceAction(node = self.ros_node,
                                            client = '/assembly_manager/create_assembly_instruction_from_description',
                                            service_type = 'assembly_manager_interfaces/srv/CreateAssemblyInstructionFromDescription',
                                            name = '1_Create Assembly Instruction')
        set_success = self.spawning_action.set_srv_req_dict_value_from_key(path_key='file_path', 
                                                                           new_value=self.instruction_json, 
                                                                           override_to_implicit=False)
        
        self.spawning_action.set_srv_req_dict_value_from_key(path_key='spawn_components', 
                                                             new_value=True, 
                                                             override_to_implicit=False)
        
        self.spawning_action.set_service_bool_identifier('success')

    def call_spawning_action(self)-> bool:
        call_success = self.spawning_action.execute()
        #self.ros_node.get_logger().info(f"Spawning action called with success: {call_success}")
        response = self.spawning_action.service_response
        self.instruction_id = response.instruction_id
        return call_success
    
    def get_relevant_frames(self)->list[str]:
        self.call_spawning_action()
        list_of_frames = set()
        # get the current frame
        plane_frames = get_frames_for_planes_of_component(self.scene, self.COMPONENT)
        for frame in plane_frames:
            if not frame.constraints.centroid.is_active and not frame.constraints.orthogonal.is_active:
                list_of_frames.add(frame.frame_name)
                continue
            

        # frame = get_ref_frame_by_name(self.scene, self.SCOPE_FRAME)
        # list_of_frames = get_constraint_frames_for_frame(self.scene, frame)
        
        return list_of_frames

    def init_recalculation_action(self):
        if self.recalculation_action is not None:
            return
        self.recalculation_action = ServiceAction(node = self.ros_node,
                                            client = '/assembly_manager/calculate_assembly_instructions',
                                            service_type = 'assembly_manager_interfaces/srv/CalculateAssemblyInstructions',
                                            name = 'X_Calculate Assembly Instruction')
        
        set_success = self.recalculation_action.set_srv_req_dict_value_from_key(path_key='instruction_id',
                                                                           new_value=self.instruction_id, 
                                                                           override_to_implicit=False)
        self.recalculation_action.set_service_bool_identifier('success')
    
        #self.programmer.save_to_JSON()
    def get_scene_callback(self, msg:ObjectScene):
        self.scene = msg
        #self.ros_node.get_logger().info("Scene received")

    def call_recalculation_action(self)-> bool:
        call_success = self.recalculation_action.execute()
        return call_success
    
    def execute_simulation(self):
        self.iterations = self.sim_parameters.num_iterations
        success = True
        success = self.execute_n_iterations(self.iterations)
        if success:
            #self.assess_results_intersections()
            #self.assess_results()
            print("Calculations done!!!")

        self.ros_node.destroy_node()

    def execute_sequence(self):
        self.programmer.execute_action_list(index_start=0, log_mode=ExecutionLog.LOG_NEVER)
    
    def gen_value_gauss(self, std:float):
        value = random.gauss(0, std)
        #print(f"Generated value: {value}")
        return value
    
    def gen_gausss_radius(self, std:float):
        r = np.abs(np.random.normal(0, std)) 
        theta = np.random.uniform(0, 2 * np.pi)

        x = r * np.cos(theta)
        y = r * np.sin(theta)
        #print(f"Generated x: {x} y: {y}")
        return x, y
    
    def modify_relatives(self):
        
        multiplier = 1e-6
        for i in range(0, len(self.programmer.action_list)):

            if self.sim_parameters.use_radiant_gauss:
                x_value, y_value = self.gen_gausss_radius(self.sim_parameters.std_camera)
            else:
                x_value = self.gen_value_gauss(self.sim_parameters.std_camera)
                y_value = self.gen_value_gauss(self.sim_parameters.std_camera)

            z_value = self.gen_value_gauss(self.sim_parameters.std_laser)

            action = self.programmer.get_action_at_index(i)
            if action.client == '/assembly_manager/modify_frame_relative':
                name = action.get_action_name()
                if 'Vision' in name:
                    if self.sim_parameters.std_camera == 0:
                        action.set_active(False)
                        continue
                    else:
                        action.set_active(True)
                    #self.ros_node.get_logger().info(f"Action {name} modified")
                    action.set_srv_req_dict_value_from_key(path_key='rel_position.x', 
                                                           new_value=x_value*multiplier, 
                                                           override_to_implicit=False)
                    
                    action.set_srv_req_dict_value_from_key(path_key='rel_position.y', 
                                                           new_value=y_value*multiplier, 
                                                           override_to_implicit=False)

                    action.set_srv_req_dict_value_from_key(path_key='rel_position.z', 
                                                           new_value=0.0, 
                                                           override_to_implicit=False)
                    
                    action.set_srv_req_dict_value_from_key(path_key='not_relativ_to_parent_but_child', 
                                                        new_value=True, 
                                                        override_to_implicit=False)
                                    
                elif 'Laser' in name:
                    if self.sim_parameters.std_laser == 0:
                        action.set_active(False)
                        continue
                    else:
                        action.set_active(True)

                    action.set_srv_req_dict_value_from_key(path_key='rel_position.x', 
                                                           new_value=0.0, 
                                                           override_to_implicit=False)
                    
                    action.set_srv_req_dict_value_from_key(path_key='rel_position.y', 
                                                           new_value=0.0, 
                                                           override_to_implicit=False)
                    #self.ros_node.get_logger().info(f"Action {name} modified")
                    action.set_srv_req_dict_value_from_key(path_key='rel_position.z', 
                                                           new_value=z_value*multiplier, 
                                                           override_to_implicit=False)
                    
                    action.set_srv_req_dict_value_from_key(path_key='not_relativ_to_parent_but_child', 
                                    new_value=True, 
                                    override_to_implicit=False)
                    
                else:
                    self.ros_node.get_logger().info(f"Action {name} not modified")
  
    
    def execute_n_iterations(self, iterations:int)->bool:
        
        for iteration in range(iterations):
            start_time = time.time()
            
            self.call_spawning_action()
            self.init_recalculation_action()

            self.ros_node.get_logger().info(f"Iteration {iteration}")
            
            self.modify_relatives()
            
            self.execute_sequence()
            recalculate_success = self.call_recalculation_action()

            if not recalculate_success:
                self.ros_node.get_logger().error("Recalculation failed")
                self.sim_parameters.increment_failure_count()
                continue

            # get the current frame
            frame = get_ref_frame_by_name(self.scene, self.SCOPE_FRAME)
            
            target_frame = get_ref_frame_by_name(self.scene, self.TARGET_FRAME)
            
            if frame is None:
                self.ros_node.get_logger().error(f"Frame {self.SCOPE_FRAME} not found in scene!")
                self.sim_parameters.increment_failure_count()
                continue
            
            if target_frame is None:
                self.ros_node.get_logger().error(f"Frame {self.TARGET_FRAME} not found in scene!")
                self.sim_parameters.increment_failure_count()
                continue
            
            #self.results_calculated_poses.append(frame.pose)
            frame.pose.orientation = target_frame.pose.orientation
            

            self.save_pose_to_file(frame.pose, self.file_name)
            
            duration = time.time() - start_time
            self.list_of_times.append(duration)
            self.ros_node.get_logger().info(f"Duration: {duration} s")
        
        average_time = sum(self.list_of_times)/len(self.list_of_times)
        print(f"Average time: {average_time} s")
        self.sim_parameters.approx_time = average_time
        self.save_results_to_file(self.file_name)
        
        return True
            #self.programmer.save_to_JSON()
    
    def plot_distributions(self):
        poses_dict = self.load_poses_from_file(self.file_name)
        transform_x_values = []
        transform_y_values = []
        transform_z_values = []
        roll_values = []
        pitch_values = []
        yaw_values = []
        
        for pose in poses_dict:
            pose:dict
            transform_x_values.append(pose['position']['x']*1e6)
            transform_y_values.append(pose['position']['y']*1e6)
            transform_z_values.append(pose['position']['z']*1e6)

            quat = Quaternion()
            quat.w = pose['orientation']['w']
            quat.x = pose['orientation']['x']
            quat.y = pose['orientation']['y']
            quat.z = pose['orientation']['z']
            # this function does not work poperly
            roll, pitch, yaw = quaternion_to_euler(quat)
            roll_values.append(roll)
            pitch_values.append(pitch)
            yaw_values.append(yaw)
        
        transform_x_values = [x - np.mean(transform_x_values) for x in transform_x_values]
        transform_y_values = [y - np.mean(transform_y_values) for y in transform_y_values]
        transform_z_values = [z - np.mean(transform_z_values) for z in transform_z_values]
        roll_values = [r - np.mean(roll_values) for r in roll_values]
        pitch_values = [p - np.mean(pitch_values) for p in pitch_values]
        yaw_values = [y - np.mean(yaw_values) for y in yaw_values]
        # Plot the distributions
        import matplotlib.pyplot as plt
        plt.figure(figsize=(12, 8))
        
        plt.subplot(2, 3, 1)
        plt.hist(transform_x_values, bins=30, color='blue', alpha=0.7)
        plt.title('Transform X Distribution')
        plt.xlabel('Transform X (um)')
        plt.ylabel('Frequency')
        
        plt.subplot(2, 3, 2)
        plt.hist(transform_y_values, bins=30, color='green', alpha=0.7)
        plt.title('Transform Y Distribution')
        plt.xlabel('Transform Y (um)')
        plt.ylabel('Frequency')
        
        plt.subplot(2, 3, 3)
        plt.hist(transform_z_values, bins=30, color='red', alpha=0.7)
        plt.title('Transform Z Distribution')
        plt.xlabel('Transform Z (um)')
        plt.ylabel('Frequency')
        
        plt.subplot(2, 3, 4)
        plt.hist(roll_values, bins=30, color='purple', alpha=0.7)
        plt.title('Roll Distribution')
        plt.xlabel('Roll (rad)')
        plt.ylabel('Frequency')
        
        plt.subplot(2, 3, 5)
        plt.hist(pitch_values, bins=30, color='orange', alpha=0.7)
        plt.title('Pitch Distribution')
        plt.xlabel('Pitch (rad)')
        plt.ylabel('Frequency')
        
        plt.subplot(2, 3, 6)
        plt.hist(yaw_values, bins=30, color='cyan', alpha=0.7)
        plt.title('Yaw Distribution')
        plt.xlabel('Yaw (rad)')
        plt.ylabel('Frequency')
        
        plt.tight_layout()
        # save the figure
        plt.savefig(f'{self.results_path}/distributions_{self.sim_parameters.results_name}.png')
        plt.show()
        
    def assess_results(self,poses_dict:list[dict]=None)->tuple:
        transform_x_values = []
        transform_y_values = []
        transform_z_values = []

        roll_values = []
        pitch_values = []
        yaw_values = []

        if len(poses_dict) == 0 or len(poses_dict) == 1:
            self.ros_node.get_logger().info("No results calculated")
            return (0,0,0)
        
        for pose in poses_dict:
            pose:dict
            transform_x_values.append(pose['position']['x']*1e6)
            transform_y_values.append(pose['position']['y']*1e6)
            transform_z_values.append(pose['position']['z']*1e6)

            quat = Quaternion()
            quat.w = pose['orientation']['w']
            quat.x = pose['orientation']['x']
            quat.y = pose['orientation']['y']
            quat.z = pose['orientation']['z']
            # this function does not work poperly
            roll, pitch, yaw = quaternion_to_euler(quat)
            roll_values.append(roll)
            pitch_values.append(pitch)
            yaw_values.append(yaw)

        print(f"Total number of poses: {len(poses_dict)}")

        self.sim_parameters.total_iterations = len(poses_dict)

        mean_transform_x = sum(transform_x_values)/len(transform_x_values)
        mean_transform_y = sum(transform_y_values)/len(transform_y_values)
        mean_transform_z = sum(transform_z_values)/len(transform_z_values)

        mean_roll = sum(roll_values)/len(roll_values)
        mean_pitch = sum(pitch_values)/len(pitch_values)
        mean_yaw = sum(yaw_values)/len(yaw_values)

        transform_x_values_mean_cleaned = [(x - mean_transform_x) for x in transform_x_values]
        transform_y_values_mean_cleaned = [(y - mean_transform_y) for y in transform_y_values]
        transform_z_values_mean_cleaned = [(z - mean_transform_z) for z in transform_z_values]

        #print(f"X-Vector: {transform_x_values_mean_cleaned} um")
        #print(f"Y-Vector: {transform_y_values_mean_cleaned} um")
        #print(f"Z-Vector: {transform_z_values_mean_cleaned} um")
        
        mean_transform_x_values_mean_cleaned = sum(transform_x_values_mean_cleaned)/len(transform_x_values_mean_cleaned)
        mean_transform_y_values_mean_cleaned = sum(transform_y_values_mean_cleaned)/len(transform_y_values_mean_cleaned)
        mean_transform_z_values_mean_cleaned = sum(transform_z_values_mean_cleaned)/len(transform_z_values_mean_cleaned)
        
        print(f"Mean transform x: {mean_transform_x_values_mean_cleaned} um")
        print(f"Mean transform y: {mean_transform_y_values_mean_cleaned} um")
        print(f"Mean transform z: {mean_transform_z_values_mean_cleaned} um")
        
        roll_values_mean_cleaned = [(r - mean_roll) for r in roll_values]
        pitch_values_mean_cleaned = [(p - mean_pitch) for p in pitch_values]
        yaw_values_mean_cleaned = [(y - mean_yaw) for y in yaw_values]
        
        mean_roll_values_mean_cleaned = sum(roll_values_mean_cleaned)/len(roll_values_mean_cleaned)
        mean_pitch_values_mean_cleaned = sum(pitch_values_mean_cleaned)/len(pitch_values_mean_cleaned)
        mean_yaw_values_mean_cleaned = sum(yaw_values_mean_cleaned)/len(yaw_values_mean_cleaned)

        print(f"Mean transform x: {mean_transform_x} um")
        print(f"Mean transform y: {mean_transform_y} um")
        print(f"Mean transform z: {mean_transform_z} um")

        if len(transform_x_values_mean_cleaned) == 0:
            self.ros_node.get_logger().info("No results calculated")
            return (0,0,0,0,0,0)
        
        sdt_transform_x = statistics.stdev(transform_x_values_mean_cleaned)
        sdt_transform_y = statistics.stdev(transform_y_values_mean_cleaned)
        sdt_transform_z = statistics.stdev(transform_z_values_mean_cleaned)

        sdt_transform_roll = statistics.stdev(roll_values_mean_cleaned)*180/np.pi
        sdt_transform_pitch = statistics.stdev(pitch_values_mean_cleaned)*180/np.pi
        sdt_transform_yaw = statistics.stdev(yaw_values_mean_cleaned)*180/np.pi

        print (f"Standard deviation transform x: {sdt_transform_x} um")
        print (f"Standard deviation transform y: {sdt_transform_y} um")
        print (f"Standard deviation transform z: {sdt_transform_z} um")
        #sdt_roll =statistics
        return (sdt_transform_x, 
                sdt_transform_y, 
                sdt_transform_z,
                sdt_transform_roll,
                sdt_transform_pitch,
                sdt_transform_yaw)


    def pose_to_dict(self,pose:Pose)->dict:
        return {
            "position": {
                "x": pose.position.x,
                "y": pose.position.y,
                "z": pose.position.z
            },
            "orientation": {
                "x": pose.orientation.x,
                "y": pose.orientation.y,
                "z": pose.orientation.z,
                "w": pose.orientation.w
            }
        }

    def save_pose_to_file(self, pose:Pose, file_path: str):
        
        old_list = []
        #load from the file
        if os.path.exists(f'{self.results_path}/{file_path}'):
            with open(f'{self.results_path}/{file_path}', 'r') as json_file:
                data = json.load(json_file)
            # add the new data to the file
            old_list = data.get("calculated_poses", [])

        serializable_pose = self.pose_to_dict(pose) 
        # add the new data to the file
        
        file_dict = {}
        old_list.append(serializable_pose)
        file_dict["calculated_poses"] = old_list

        with open(f'{self.results_path}/{file_path}', 'w') as json_file:
            json.dump(file_dict, json_file, indent=3)

    def save_results_to_file(self, file_path: str):
        
        dict_list = self.load_poses_from_file(file_path)
        
        x_sdt, y_sdt, z_sdt, roll_sdt, pitch_sdt, yaw_sdt = self.assess_results(dict_list)
        sdt_dict = {
            "x": x_sdt,
            "y": y_sdt,
            "z": z_sdt,
            "roll": roll_sdt,
            "pitch": pitch_sdt,
            "yaw": yaw_sdt
        }
        data = {}

        if os.path.exists(f'{self.results_path}/{file_path}'):
            with open(f'{self.results_path}/{file_path}', 'r') as json_file:
                data = json.load(json_file)
        
        data["sdt"] = sdt_dict
        data["sim_parameters"] = self.sim_parameters.to_dict()
        
        with open(f'{self.results_path}/{file_path}', 'w') as json_file:
            json.dump(data, json_file, indent=3)
            
        # add the new data to the file
        
            
    def load_poses_from_file(self, file_path: str)->list[dict]:
        # load from the file
        if os.path.exists(f'{self.results_path}/{file_path}'):
            with open(f'{self.results_path}/{file_path}', 'r') as json_file:
                data = json.load(json_file)
            return data.get("calculated_poses", [])
        else:
            print("File not found")
            return None 
        

if __name__ == '__main__':  
    # create ros node
    rclpy.init()
    executor = MultiThreadedExecutor(num_threads=6) 
    just_a_node = Node('my_Node')
    executor.add_node(just_a_node)
    tol_measurment = TolMeasurment(just_a_node)
    #tol_measurment.call_spawning_action()
    #frames = tol_measurment.get_relevant_frames()
    #print(frames)
    tol_measurment.execute_simulation()
    tol_measurment.plot_distributions()
    rclpy.shutdown()
    
