import json
import os
import random
from ros_sequential_action_programmer.submodules.RosSequentialActionProgrammer import RosSequentialActionProgrammer, LOG_AT_END
from rclpy.node import Node
import rclpy
from rclpy.executors import MultiThreadedExecutor
import statistics
from geometry_msgs.msg import Vector3, Quaternion
import numpy as np
from assembly_scene_publisher.py_modules.geometry_type_functions import quaternion_to_euler


class TolMeasurment():
    def __init__(self, ros_node:Node) -> None:
        self.ros_node = ros_node
        self.programmer = RosSequentialActionProgrammer(just_a_node)
        self.programmer.load_from_JSON('/home/niklas/Documents/SolidWorks_ASsembly_Instructor/examples/SWASI_Paper_Demonstrator_V2/Tolerance_Measurement_SWASI_Paper.json')
        self.components_path = '/home/niklas/Documents/SolidWorks_ASsembly_Instructor/examples/SWASI_Paper_Demonstrator_V2/SWASI_Exports/components'
        #self.results_path = '/home/niklas/Documents/SolidWorks_ASsembly_Instructor/examples/SWASI_Paper_Demonstrator_V2/logs/'
        self.results_path = '/home/mll/SolidWorks_ASsembly_Instructor/examples/SWASI_Paper_Demonstrator_V2/logs/'

        self.random_max_x = 0.002
        self.random_max_y = 0.002
        self.random_max_z = 0.002

        self.iterations = 100
        success = True
        #success = self.execute_n_iterations(self.iterations)
        if success:
            self.assess_results_intersections()
            #self.assess_results()
            print("Calculations done!!!")
        self.ros_node.destroy_node()

    def execute_n_iterations(self, iterations:int):
        for iteration in range(iterations):
            success = self.execute_one_tol_measurement()
            print(f"Iteration {iteration} done, success: {success}")
            if not success:
                print(f"Tolerance measurement failed at iteration {iteration}!")
                return False
            
        print(f"Tolerance measurement succeeded {iterations} times!")
        return True

    def execute_one_tol_measurement(self) -> bool:
        self.modify_components()
        (success, index) = self.programmer.execute_action_list(index_start=0, log_mode=LOG_AT_END)
        #print(f"success: {success}, index: {index}")
        return success
    
    def modify_components(self):
        components_names_list = ["UFC_Paper_MODTOL.json","Glas_Platelet_Paper_MODTOL.json"]
        ideal_components_list = ["UFC_Paper.json","Glas_Platelet_Paper.json"]

        for index in range(len(components_names_list)):
            component = components_names_list[index]
            ideal_component = ideal_components_list[index]

            with open(f"{self.components_path}/{ideal_component}", 'r') as json_file:
                ideal_data: dict = json.load(json_file)
                ideal_frames = ideal_data.get("mountingDescription").get("mountingReferences").get("ref_frames")

            with open(f"{self.components_path}/{component}", 'r') as json_file:
                data:dict = json.load(json_file)
            frames = data.get("mountingDescription").get("mountingReferences").get("ref_frames")

            for index, frame in enumerate(frames):
                if 'Vision' in frame.get("name"):
                    frame["transformation"]["translation"]["X"] = ideal_frames[index]["transformation"]["translation"]["X"] + random.uniform(-self.random_max_x, self.random_max_x)
                    frame["transformation"]["translation"]["Y"] = ideal_frames[index]["transformation"]["translation"]["Y"] + random.uniform(-self.random_max_y, self.random_max_y)
                    frame["transformation"]["translation"]["Z"] = ideal_frames[index]["transformation"]["translation"]["Z"] + random.uniform(-self.random_max_z, self.random_max_z)
            # save file
            with open(f"{self.components_path}/{component}", 'w') as json_file:
                json.dump(data, json_file, indent=4)

    def assess_results(self):
        transform_x_values = []
        transform_y_values = []
        transform_z_values = []

        roll_values = []
        pitch_values = []
        yaw_values = []

        for file in os.listdir(self.results_path):
            if file is None:
                return
            if file.endswith('.json'):
                with open(f"{self.results_path}/{file}", 'r') as json_file:
                    data:dict = json.load(json_file)
                transform_x_values.append(data["action_log"][1]["1_Calculate Assembly Instruction"]["srv_response"]["assembly_transform"]["position"]["x"]*1e6)
                transform_y_values.append(data["action_log"][1]["1_Calculate Assembly Instruction"]["srv_response"]["assembly_transform"]["position"]["y"]*1e6)
                transform_z_values.append(data["action_log"][1]["1_Calculate Assembly Instruction"]["srv_response"]["assembly_transform"]["position"]["z"]*1e6)
                quat = Quaternion()
                quat.w = data["action_log"][1]["1_Calculate Assembly Instruction"]["srv_response"]["assembly_transform"]["orientation"]["w"]
                quat.x = data["action_log"][1]["1_Calculate Assembly Instruction"]["srv_response"]["assembly_transform"]["orientation"]["x"]
                quat.y = data["action_log"][1]["1_Calculate Assembly Instruction"]["srv_response"]["assembly_transform"]["orientation"]["y"]
                quat.z = data["action_log"][1]["1_Calculate Assembly Instruction"]["srv_response"]["assembly_transform"]["orientation"]["z"]
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

        print(f"Y-Vector: {transform_y_values_mean_cleaned} um")
        #print(f"Z-Vector: {transform_z_values_mean_cleaned} um")
        #print(f"X-Vector: {transform_x_values_mean_cleaned} um")

        roll_values_mean_cleaned = [(r - mean_roll) for r in roll_values]
        pitch_values_mean_cleaned = [(p - mean_pitch) for p in pitch_values]
        yaw_values_mean_cleaned = [(y - mean_yaw) for y in yaw_values]

        #print(f"Roll: {[x*180/np.pi for x in roll_values_mean_cleaned]} deg")
        #print(f"Pitch: {[x*180/np.pi for x in pitch_values_mean_cleaned]} deg")
        #print(f"Yaw: {[x*180/np.pi for x in yaw_values_mean_cleaned]} deg")
        
        print(f"Mean transform x: {mean_transform_x} um")
        print(f"Mean transform y: {mean_transform_y} um")
        print(f"Mean transform z: {mean_transform_z} um")

        sdt_transform_x =statistics.stdev(transform_x_values_mean_cleaned)
        sdt_transform_y =statistics.stdev(transform_y_values_mean_cleaned)
        sdt_transform_z =statistics.stdev(transform_z_values_mean_cleaned)

        sdt_roll =statistics.stdev(roll_values_mean_cleaned)
        sdt_pitch =statistics.stdev(pitch_values_mean_cleaned)
        sdt_yaw =statistics.stdev(yaw_values_mean_cleaned)

        print(f"Standard deviation transform x: {sdt_transform_x} um")
        print(f"Standard deviation transform y: {sdt_transform_y} um")
        print(f"Standard deviation transform z: {sdt_transform_z} um")

        print(f"Standard deviation roll: {sdt_roll*180/np.pi} deg")
        print(f"Standard deviation pitch: {sdt_pitch*180/np.pi} deg")
        print(f"Standard deviation yaw: {sdt_yaw*180/np.pi} deg")

    def get_pose_from_frame_log(self, file_path:str)->tuple:
        with open(file_path, 'r') as json_file:
            data:dict = json.load(json_file)
        
        frames = data.get("action_log")[2]["2_Get Scene"]["srv_response"]["scene"]["objects_in_scene"][0]["ref_frames"]
        keyword = "assembly_frame"

        #frames = data.get("action_log")[2]["2_Get Scene"]["srv_response"]["scene"]["objects_in_scene"][1]["ref_frames"]
        #keyword = "target_frame"
        
        for frame in frames:
            if keyword in frame["frame_name"]:
                transform_x = frame["pose"]["position"]["x"]
                transform_y = frame["pose"]["position"]["y"]
                transform_z = frame["pose"]["position"]["z"]

                quat = Quaternion()
                quat.w = frame["pose"]["orientation"]["w"]
                quat.x = frame["pose"]["orientation"]["x"]
                quat.y = frame["pose"]["orientation"]["y"]
                quat.z = frame["pose"]["orientation"]["z"]
                roll, pitch, yaw = quaternion_to_euler(quat)
                print(quat)
                return (transform_x, transform_y, transform_z, roll, pitch, yaw)
        
        return None

    
    def assess_results_intersections(self):
        pose_x_values = []
        pose_y_values = []
        pose_z_values = []

        roll_values = []
        pitch_values = []
        yaw_values = []

        for file in os.listdir(self.results_path):
            if file is None:
                return
            if file.endswith('.json'):
                _pose_x, pose_y, pose_z, pose_roll, pose_pitch, pose_yaw = self.get_pose_from_frame_log(f"{self.results_path}/{file}")

                pose_x_values.append(_pose_x*1e6)
                pose_y_values.append(pose_y*1e6)
                pose_z_values.append(pose_z*1e6)
                roll_values.append(pose_roll)
                pitch_values.append(pose_pitch)
                yaw_values.append(pose_yaw)

        mean_transform_x = sum(pose_x_values)/len(pose_x_values)
        mean_transform_y = sum(pose_y_values)/len(pose_y_values)
        mean_transform_z = sum(pose_z_values)/len(pose_z_values)

        mean_roll = sum(roll_values)/len(roll_values)
        mean_pitch = sum(pitch_values)/len(pitch_values)
        mean_yaw = sum(yaw_values)/len(yaw_values)

        transform_x_values_mean_cleaned = [(x - mean_transform_x) for x in pose_x_values]
        transform_y_values_mean_cleaned = [(y - mean_transform_y) for y in pose_y_values]
        transform_z_values_mean_cleaned = [(z - mean_transform_z) for z in pose_z_values]

        roll_values_mean_cleaned = [(r - mean_roll) for r in roll_values]
        pitch_values_mean_cleaned = [(p - mean_pitch) for p in pitch_values]
        yaw_values_mean_cleaned = [(y - mean_yaw) for y in yaw_values]
        
        print(f"Mean transform x: {mean_transform_x} um")
        print(f"Mean transform y: {mean_transform_y} um")
        print(f"Mean transform z: {mean_transform_z} um")

        sdt_transform_x =statistics.stdev(transform_x_values_mean_cleaned)
        sdt_transform_y =statistics.stdev(transform_y_values_mean_cleaned)
        sdt_transform_z =statistics.stdev(transform_z_values_mean_cleaned)

        sdt_roll =statistics.stdev(roll_values_mean_cleaned)
        sdt_pitch =statistics.stdev(pitch_values_mean_cleaned)
        sdt_yaw =statistics.stdev(yaw_values_mean_cleaned)

        print(f"Standard deviation transform x: {sdt_transform_x} um")
        print(f"Standard deviation transform y: {sdt_transform_y} um")
        print(f"Standard deviation transform z: {sdt_transform_z} um")

        print(f"Standard deviation roll: {sdt_roll*180/np.pi} deg")
        print(f"Standard deviation pitch: {sdt_pitch*180/np.pi} deg")
        print(f"Standard deviation yaw: {sdt_yaw*180/np.pi} deg")

if __name__ == '__main__':  
    # create ros node
    rclpy.init()
    executor = MultiThreadedExecutor(num_threads=6) 
    just_a_node = Node('my_Node')
    executor.add_node(just_a_node)
    tol_measurment = TolMeasurment(just_a_node)
    rclpy.shutdown()
    
