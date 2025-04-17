import numpy as np
import matplotlib.pyplot as plt
import json
import os
import random

def gen_gausss_radius(std:float):
    r = np.abs(np.random.normal(0, std)) 
    theta = np.random.uniform(0, 2 * np.pi)

    x = r * np.cos(theta)
    y = r * np.sin(theta)
    return x, y

def gen_value_gauss(std:float):
    #x = random.gauss(0, std)
    #y = random.gauss(0, std)
    x = np.random.normal(0, std)
    y = np.random.normal(0, std)
    return x, y

def get_x_y_values_from_file(file_name):
    with open(file_name, 'r') as f:
        data = json.load(f)

    x_values_result = []
    y_values_result = []


    poses = data.get('calculated_poses',{} )

    if not poses:
        print("No poses found in the JSON file.")
        exit()

    for pose in poses:
        position = pose.get('position', {})
        if not position:
            continue
        x = position.get('x', 0)
        y = position.get('y', 0)
        x_values_result.append(x)
        y_values_result.append(y)

    # clean mean
    x_mean = np.mean(x_values_result)
    y_mean = np.mean(y_values_result)
    x_values_result = [x - x_mean for x in x_values_result]
    y_values_result = [y - y_mean for y in y_values_result]
    std_x = np.std(x_values_result)
    std_y = np.std(y_values_result)
    return  (   x_values_result, 
                y_values_result,
                std_x, 
                std_y)

#file_name = "documentation/tolerance_analyses/logs/poses_list.json"
#file_name = "documentation/tolerance_analyses/logs/poses_list_R01.json"


x_list = []
y_list = []

for i in range(10000):
    #x, y = gen_value_gauss(1)
    x, y = gen_gausss_radius(1e-6)
    x_list.append(x)
    y_list.append(y)

sdt_x_list = np.std(x_list)
sdt_y_list = np.std(y_list)

print(f"std_x: {sdt_x_list} std_y: {sdt_y_list}")

x_list_file, y_list_file, sdt_x, sdt_y = get_x_y_values_from_file(file_name)

print(f"std_x: {sdt_x} std_y: {sdt_y}")

plt.figure(figsize=(10, 10))
plt.scatter(x_list, y_list, c='blue', alpha=0.5, label='Generated Points')
plt.scatter(x_list_file, y_list_file, c='red', alpha=0.5, label='File Points')
#draw sdt as circle
circle = plt.Circle((0, 0), sdt_x, color='yellow', fill=False, linestyle='dotted', label='STD_FILE',linewidth=3)
plt.gca().add_artist(circle)
circle = plt.Circle((0, 0), sdt_x_list, color='black', fill=False, linestyle='dotted', label='STD_GENERATED',linewidth=3)
plt.gca().add_artist(circle)
#plt.gca().set_aspect('equal', adjustable='box')
plt.title('Scatter Plot of Generated Points vs File Points')
plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.axhline(0, color='black', lw=0.5)
plt.axvline(0, color='black', lw=0.5)
plt.xlim(-4e-6, 4e-6)
plt.ylim(-4e-6, 4e-6)
plt.grid()
plt.legend()
plt.show()



