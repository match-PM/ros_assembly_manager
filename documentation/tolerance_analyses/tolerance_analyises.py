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

from assembly_scene_publisher.py_modules.scene_functions import get_ref_frame_by_name
class TolMeasurment():
    STD_CAMERA= 5 #um
    STD_LASER = 1 #um
    
    def __init__(self, ros_node:Node) -> None:
        self.ros_node = ros_node
        self.programmer = RosSequentialActionProgrammer(just_a_node)
        self.programmer.load_from_JSON('/home/niklas/ros2_ws/src/ros_assembly_manager/documentation/tolerance_analyses/rsap_description.json')
        self.programmer.config.execution_log.set_execution_log_mode(1)
        #self.components_path = '/home/niklas/Documents/SolidWorks_ASsembly_Instructor/examples/SWASI_Paper_Demonstrator_V2/SWASI_Exports/components'
        #self.results_path = '/home/niklas/Documents/SolidWorks_ASsembly_Instructor/examples/SWASI_Paper_Demonstrator_V2/logs/'
        self.results_path = '/home/niklas/ros2_ws/src/ros_assembly_manager/documentation/tolerance_analyses/logs'
        self.scene = None
        self.ros_node.create_subscription(ObjectScene, '/assembly_manager/scene', self.get_scene_callback, 10)
        
        self.results_calculated_poses:list[Pose] = []

    def get_scene_callback(self, msg:ObjectScene):
        self.scene = msg
        #self.ros_node.get_logger().info("Scene received")

    def execute_simulation(self):
        self.iterations = 20
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
        
        action = self.programmer.get_action_at_index(1)
        multiplier = 1e-6
        for i in range(1, len(self.programmer.action_list)):
            x_value = self.gen_value_gauss(self.STD_CAMERA)
            y_value = self.gen_value_gauss(self.STD_CAMERA)
            z_value = self.gen_value_gauss(self.STD_LASER)
            action = self.programmer.get_action_at_index(i)
            if action.client == '/assembly_manager/modify_frame_relative':
                name = action.get_action_name()
                if 'Vision' in name:
                    #self.ros_node.get_logger().info(f"Action {name} modified")
                    action.set_srv_req_dict_value_from_key(path_key='rel_position.x', new_value=x_value*multiplier, override_to_implicit=True)
                    action.set_srv_req_dict_value_from_key(path_key='rel_position.y', new_value=y_value*multiplier, override_to_implicit=True)
                elif 'Laser' in name:
                    #self.ros_node.get_logger().info(f"Action {name} modified")
                    action.set_srv_req_dict_value_from_key(path_key='rel_position.z', new_value=z_value*multiplier, override_to_implicit=True)
                else:
                    self.ros_node.get_logger().info(f"Action {name} not modified")
        #self.programmer.save_to_JSON()       
    
    def execute_n_iterations(self, iterations:int)->bool:
        
        for iteration in range(iterations):
            self.ros_node.get_logger().info(f"Iteration {iteration}")
            start_time = time.time()
            self.modify_relatives()
            self.execute_sequence()
            
            # get the current frame
            frame = get_ref_frame_by_name(self.scene, "assembly_frame_Description_Glas_6D_tol-1_UFC_6D_tol-1")
            
            if frame is None:
                self.ros_node.get_logger().info("Frame not found")
                return False
            
            self.results_calculated_poses.append(frame.pose)
            
            self.save_poses_to_file(f'poses_list.json')
            
            duration = time.time() - start_time
            self.ros_node.get_logger().info(f"Duration: {duration} s")
        
        self.save_results_to_file(f'poses_list.json')
        
        return True
            #self.programmer.save_to_JSON()
    
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
        
        print(f"Mean transform x: {mean_transform_x} um")
        print(f"Mean transform y: {mean_transform_y} um")
        print(f"Mean transform z: {mean_transform_z} um")

        if len(transform_x_values_mean_cleaned) == 0:
            self.ros_node.get_logger().info("No results calculated")
            return (0,0,0)
        sdt_transform_x = statistics.stdev(transform_x_values_mean_cleaned)
        sdt_transform_y = statistics.stdev(transform_y_values_mean_cleaned)
        sdt_transform_z = statistics.stdev(transform_z_values_mean_cleaned)

        print (f"Standard deviation transform x: {sdt_transform_x} um")
        print (f"Standard deviation transform y: {sdt_transform_y} um")
        print (f"Standard deviation transform z: {sdt_transform_z} um")
        #sdt_roll =statistics
        return (sdt_transform_x, sdt_transform_y, sdt_transform_z)


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

    def save_poses_to_file(self, file_path: str):
        
        old_list = []
        # load from the file
        # if os.path.exists(f'{self.results_path}/{file_path}'):
        #     with open(f'{self.results_path}/{file_path}', 'r') as json_file:
        #         data = json.load(json_file)
        #     # add the new data to the file
        #     old_list = data.get("calculated_poses", [])

        serializable_poses = [self.pose_to_dict(pose) for pose in self.results_calculated_poses]
        # add the new data to the file
        
        file_dict = {}
        old_list.extend(serializable_poses)
        file_dict["calculated_poses"] = old_list

        with open(f'{self.results_path}/{file_path}', 'w') as json_file:
            json.dump(file_dict, json_file, indent=3)

    def save_results_to_file(self, file_path: str):
        
        dict_list = self.load_poses_from_file(file_path)
        
        x_sdt, y_sdt, z_sdt = self.assess_results(dict_list)
        sdt_dict = {
            "x": x_sdt,
            "y": y_sdt,
            "z": z_sdt
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
        
    # def assess_results(self):
    #     transform_x_values = []
    #     transform_y_values = []
    #     transform_z_values = []

    #     roll_values = []
    #     pitch_values = []
    #     yaw_values = []
    #     action_name = "18_/assembly_manager/get_scene"
    #     ind = 18
        
    #     for file in os.listdir(self.results_path):
    #         if file is None:
    #             return
    #         if file.endswith('.json'):
    #             with open(f"{self.results_path}/{file}", 'r') as json_file:
    #                 data:dict = json.load(json_file)

    #             transform_x_values.append(data["action_log"][ind][action_name]["srv_response"]["assembly_transform"]["position"]["x"]*1e6)
    #             transform_y_values.append(data["action_log"][ind][action_name]["srv_response"]["assembly_transform"]["position"]["y"]*1e6)
    #             transform_z_values.append(data["action_log"][ind][action_name]["srv_response"]["assembly_transform"]["position"]["z"]*1e6)
    #             quat = Quaternion()
    #             quat.w = data["action_log"][ind][action_name]["srv_response"]["assembly_transform"]["orientation"]["w"]
    #             quat.x = data["action_log"][ind][action_name]["srv_response"]["assembly_transform"]["orientation"]["x"]
    #             quat.y = data["action_log"][ind][action_name]["srv_response"]["assembly_transform"]["orientation"]["y"]
    #             quat.z = data["action_log"][ind][action_name]["srv_response"]["assembly_transform"]["orientation"]["z"]
    #             # this function does not work poperly
    #             roll, pitch, yaw = quaternion_to_euler(quat)
    #             roll_values.append(roll)
    #             pitch_values.append(pitch)
    #             yaw_values.append(yaw)

    #     mean_transform_x = sum(transform_x_values)/len(transform_x_values)
    #     mean_transform_y = sum(transform_y_values)/len(transform_y_values)
    #     mean_transform_z = sum(transform_z_values)/len(transform_z_values)

    #     mean_roll = sum(roll_values)/len(roll_values)
    #     mean_pitch = sum(pitch_values)/len(pitch_values)
    #     mean_yaw = sum(yaw_values)/len(yaw_values)

    #     transform_x_values_mean_cleaned = [(x - mean_transform_x) for x in transform_x_values]
    #     transform_y_values_mean_cleaned = [(y - mean_transform_y) for y in transform_y_values]
    #     transform_z_values_mean_cleaned = [(z - mean_transform_z) for z in transform_z_values]

    #     print(f"Y-Vector: {transform_y_values_mean_cleaned} um")
    #     #print(f"Z-Vector: {transform_z_values_mean_cleaned} um")
    #     #print(f"X-Vector: {transform_x_values_mean_cleaned} um")

    #     roll_values_mean_cleaned = [(r - mean_roll) for r in roll_values]
    #     pitch_values_mean_cleaned = [(p - mean_pitch) for p in pitch_values]
    #     yaw_values_mean_cleaned = [(y - mean_yaw) for y in yaw_values]

    #     #print(f"Roll: {[x*180/np.pi for x in roll_values_mean_cleaned]} deg")
    #     #print(f"Pitch: {[x*180/np.pi for x in pitch_values_mean_cleaned]} deg")
    #     #print(f"Yaw: {[x*180/np.pi for x in yaw_values_mean_cleaned]} deg")
        
    #     print(f"Mean transform x: {mean_transform_x} um")
    #     print(f"Mean transform y: {mean_transform_y} um")
    #     print(f"Mean transform z: {mean_transform_z} um")

    #     sdt_transform_x =statistics.stdev(transform_x_values_mean_cleaned)
    #     sdt_transform_y =statistics.stdev(transform_y_values_mean_cleaned)
    #     sdt_transform_z =statistics.stdev(transform_z_values_mean_cleaned)

    #     sdt_roll =statistics.stdev(roll_values_mean_cleaned)
    #     sdt_pitch =statistics.stdev(pitch_values_mean_cleaned)
    #     sdt_yaw =statistics.stdev(yaw_values_mean_cleaned)

    #     print(f"Standard deviation transform x: {sdt_transform_x} um")
    #     print(f"Standard deviation transform y: {sdt_transform_y} um")
    #     print(f"Standard deviation transform z: {sdt_transform_z} um")

    #     print(f"Standard deviation roll: {sdt_roll*180/np.pi} deg")
    #     print(f"Standard deviation pitch: {sdt_pitch*180/np.pi} deg")
    #     print(f"Standard deviation yaw: {sdt_yaw*180/np.pi} deg")

    # def get_pose_from_frame_log(self, file_path:str)->tuple:
    #     with open(file_path, 'r') as json_file:
    #         data:dict = json.load(json_file)
        
    #     # frames = data.get("action_log")[2]["2_Get Scene"]["srv_response"]["scene"]["objects_in_scene"][0]["ref_frames"]
    #     # keyword = "assembly_frame"

    #     frames = data.get("action_log")[2]["2_Get Scene"]["srv_response"]["scene"]["objects_in_scene"][1]["ref_frames"]
    #     keyword = "target_frame"
        
    #     for frame in frames:
    #         if keyword in frame["frame_name"]:
    #             transform_x = frame["pose"]["position"]["x"]
    #             transform_y = frame["pose"]["position"]["y"]
    #             transform_z = frame["pose"]["position"]["z"]

    #             quat = Quaternion()
    #             quat.w = frame["pose"]["orientation"]["w"]
    #             quat.x = frame["pose"]["orientation"]["x"]
    #             quat.y = frame["pose"]["orientation"]["y"]
    #             quat.z = frame["pose"]["orientation"]["z"]
    #             roll, pitch, yaw = quaternion_to_euler(quat)
    #             print(quat)
    #             return (transform_x, transform_y, transform_z, roll, pitch, yaw)
        
    #     return None

    
    # def assess_results_intersections(self):
    #     pose_x_values = []
    #     pose_y_values = []
    #     pose_z_values = []

    #     roll_values = []
    #     pitch_values = []
    #     yaw_values = []

    #     for file in os.listdir(self.results_path):
    #         if file is None:
    #             return
    #         if file.endswith('.json'):
    #             _pose_x, pose_y, pose_z, pose_roll, pose_pitch, pose_yaw = self.get_pose_from_frame_log(f"{self.results_path}/{file}")

    #             pose_x_values.append(_pose_x*1e6)
    #             pose_y_values.append(pose_y*1e6)
    #             pose_z_values.append(pose_z*1e6)
    #             roll_values.append(pose_roll)
    #             pitch_values.append(pose_pitch)
    #             yaw_values.append(pose_yaw)

    #     mean_transform_x = sum(pose_x_values)/len(pose_x_values)
    #     mean_transform_y = sum(pose_y_values)/len(pose_y_values)
    #     mean_transform_z = sum(pose_z_values)/len(pose_z_values)

    #     mean_roll = sum(roll_values)/len(roll_values)
    #     mean_pitch = sum(pitch_values)/len(pitch_values)
    #     mean_yaw = sum(yaw_values)/len(yaw_values)

    #     transform_x_values_mean_cleaned = [(x - mean_transform_x) for x in pose_x_values]
    #     transform_y_values_mean_cleaned = [(y - mean_transform_y) for y in pose_y_values]
    #     transform_z_values_mean_cleaned = [(z - mean_transform_z) for z in pose_z_values]

    #     roll_values_mean_cleaned = [(r - mean_roll) for r in roll_values]
    #     pitch_values_mean_cleaned = [(p - mean_pitch) for p in pitch_values]
    #     yaw_values_mean_cleaned = [(y - mean_yaw) for y in yaw_values]
        
    #     print(f"Mean transform x: {mean_transform_x} um")
    #     print(f"Mean transform y: {mean_transform_y} um")
    #     print(f"Mean transform z: {mean_transform_z} um")

    #     sdt_transform_x =statistics.stdev(transform_x_values_mean_cleaned)
    #     sdt_transform_y =statistics.stdev(transform_y_values_mean_cleaned)
    #     sdt_transform_z =statistics.stdev(transform_z_values_mean_cleaned)

    #     sdt_roll =statistics.stdev(roll_values_mean_cleaned)
    #     sdt_pitch =statistics.stdev(pitch_values_mean_cleaned)
    #     sdt_yaw =statistics.stdev(yaw_values_mean_cleaned)

    #     print(f"Standard deviation transform x: {sdt_transform_x} um")
    #     print(f"Standard deviation transform y: {sdt_transform_y} um")
    #     print(f"Standard deviation transform z: {sdt_transform_z} um")

    #     print(f"Standard deviation roll: {sdt_roll*180/np.pi} deg")
    #     print(f"Standard deviation pitch: {sdt_pitch*180/np.pi} deg")
    #     print(f"Standard deviation yaw: {sdt_yaw*180/np.pi} deg")

if __name__ == '__main__':  
    # create ros node
    rclpy.init()
    executor = MultiThreadedExecutor(num_threads=6) 
    just_a_node = Node('my_Node')
    executor.add_node(just_a_node)
    tol_measurment = TolMeasurment(just_a_node)
    tol_measurment.execute_simulation()
    rclpy.shutdown()
    
