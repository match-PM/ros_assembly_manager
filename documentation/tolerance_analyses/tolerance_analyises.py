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
from assembly_manager_interfaces.msg import ObjectScene
from geometry_msgs.msg import Pose
from ros_sequential_action_programmer.submodules.action_classes.ServiceAction import ServiceAction

from assembly_scene_publisher.py_modules.frame_constraints import get_constraint_frames_for_frame, get_identification_order
from assembly_scene_publisher.py_modules.scene_functions import get_frames_for_planes_of_component, get_ref_frame_by_name

class TolMeasurment():
    STD_CAMERA= 5 #um
    STD_LASER = 1 #um
    NUM_ITERATIONS = 50

    SCOPE_FRAME = "assembly_frame_Description_Glas_6D_tol-1_UFC_6D_tol-1"
    TARGET_FRAME = 'target_frame_Description_Glas_6D_tol-1_UFC_6D_tol-1'
    COMPONENT = 'UFC_6D_tol-1'
    
    def __init__(self, ros_node:Node) -> None:
        self.ros_node = ros_node
        self.programmer = RosSequentialActionProgrammer(ros_node)
        self.programmer.load_from_JSON('/home/niklas/ros2_ws/src/ros_assembly_manager/documentation/tolerance_analyses/rsap_description.json')
        #self.programmer.load_from_JSON('/home/mll/ros2_ws/src/ros_assembly_manager/documentation/tolerance_analyses/rsap_description.json')

        self.instruction_json = '/home/niklas/ros2_ws/src/ros_assembly_manager/documentation/tolerance_analyses/SWASI_Exports/assemblies/Assembly_UFC_Glas_6D_tol.json'
        #self.instruction_json = '/home/mll/ros2_ws/src/ros_assembly_manager/documentation/tolerance_analyses/SWASI_Exports/assemblies/Assembly_UFC_Glas_6D_tol.json'
        #self.results_path = '/home/mll/ros2_ws/src/ros_assembly_manager/documentation/tolerance_analyses/logs'
        self.results_path = '/home/niklas/ros2_ws/src/ros_assembly_manager/documentation/tolerance_analyses/logs'

        self.init_spawning_action()

        #self.programmer.config.execution_log.set_execution_log_mode(1)
        #self.components_path = '/home/niklas/Documents/SolidWorks_ASsembly_Instructor/examples/SWASI_Paper_Demonstrator_V2/SWASI_Exports/components'
        #self.results_path = '/home/niklas/Documents/SolidWorks_ASsembly_Instructor/examples/SWASI_Paper_Demonstrator_V2/logs/'
        #self.results_path = '/home/niklas/ros2_ws/src/ros_assembly_manager/documentation/tolerance_analyses/logs'
        self.scene = None
        self.ros_node.create_subscription(ObjectScene, '/assembly_manager/scene', self.get_scene_callback, 10)
        
        self.results_calculated_poses:list[Pose] = []
        self.recalculation_action = None

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
        self.iterations = self.NUM_ITERATIONS
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
        return value
    
    def modify_relatives(self):
        
        multiplier = 1e-6
        for i in range(0, len(self.programmer.action_list)):
            x_value = self.gen_value_gauss(self.STD_CAMERA)
            y_value = self.gen_value_gauss(self.STD_CAMERA)
            z_value = self.gen_value_gauss(self.STD_LASER)
            action = self.programmer.get_action_at_index(i)
            if action.client == '/assembly_manager/modify_frame_relative':
                name = action.get_action_name()
                if 'Vision' in name:
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

        # print("TEST")
        # for i in range(0, len(self.programmer.action_list)-1):

        #     action = self.programmer.get_action_at_index(i)
        #     if action.client == '/assembly_manager/modify_frame_relative':
        #         print(action.service_request)
        # save_success = self.programmer.save_to_JSON()
        # print(f"Save success: {save_success}")       
    
    def execute_n_iterations(self, iterations:int)->bool:
        
        for iteration in range(iterations):
            start_time = time.time()
            
            self.call_spawning_action()
            self.init_recalculation_action()

            self.ros_node.get_logger().info(f"Iteration {iteration}")
            
            self.modify_relatives()
            
            self.execute_sequence()
            self.call_recalculation_action()

            # get the current frame
            frame = get_ref_frame_by_name(self.scene, self.SCOPE_FRAME)
            
            target_frame = get_ref_frame_by_name(self.scene, self.TARGET_FRAME)
            
            if frame is None:
                self.ros_node.get_logger().info("Frame not found")
                return False
            
            #self.results_calculated_poses.append(frame.pose)
            frame.pose.orientation = target_frame.pose.orientation
            
            self.save_pose_to_file(frame.pose, f'poses_list.json')
            
            duration = time.time() - start_time
            self.ros_node.get_logger().info(f"Duration: {duration} s")
        
        self.save_results_to_file(f'poses_list.json')
        
        return True
            #self.programmer.save_to_JSON()
    
    def plot_distributions(self):
        poses_dict = self.load_poses_from_file(f'poses_list.json')
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
        plt.savefig(f'{self.results_path}/distributions.png')
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

        mean_transform_x = sum(transform_x_values)/len(transform_x_values)
        mean_transform_y = sum(transform_y_values)/len(transform_y_values)
        mean_transform_z = sum(transform_z_values)/len(transform_z_values)

        mean_roll = sum(roll_values)/len(roll_values)
        mean_pitch = sum(pitch_values)/len(pitch_values)
        mean_yaw = sum(yaw_values)/len(yaw_values)

        transform_x_values_mean_cleaned = [(x - mean_transform_x) for x in transform_x_values]
        transform_y_values_mean_cleaned = [(y - mean_transform_y) for y in transform_y_values]
        transform_z_values_mean_cleaned = [(z - mean_transform_z) for z in transform_z_values]

        print(f"X-Vector: {transform_x_values_mean_cleaned} um")
        print(f"Y-Vector: {transform_y_values_mean_cleaned} um")
        print(f"Z-Vector: {transform_z_values_mean_cleaned} um")
        
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
    
