import re
import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
import json
import glob
import os
import math
import lgsvl
from environs import Env
import matplotlib.pyplot as plt

def convert_speed(speed_str):
    if not isinstance(speed_str, str):
        speed_str = str(speed_str)
    if "KMPH" in speed_str:
        number_str = speed_str.split("KMPH")[0]
        speed_int = int(number_str)
        return speed_int
    elif "MPH" in speed_str:
        number_str = speed_str.split("MPH")[0]
        speed_int = int(number_str)
        return speed_int
    elif "mph" in speed_str:
        number_str = speed_str.split("mph")[0]
        speed_int = int(number_str)
        return speed_int
    else:
        return int(speed_str)
    
def normalize(weights):
    if weights == [0,0,0,0]:
        return [1,0,0,0]
    total = sum(weights)
    normalized_weights = [w / total for w in weights]
    return normalized_weights
    
def extract_numbers(s):
    s = str(s)
    # 使用正则表达式找到所有连续的数字
    numbers = re.findall(r'\d+', s)
    # 将提取的字符串数字转换为整数
    return int(numbers[0])

def is_continuous_subsequence(sub, main):
    sub_len = len(sub)
    main_len = len(main)
    
    # 如果子列表长度大于主列表，直接返回 False
    if sub_len > main_len:
        return False
    
    # 遍历主列表，寻找子列表匹配的位置
    for i in range(main_len - sub_len + 1):
        # 如果在主列表从 i 开始的子段与子列表相等，则返回 True
        if main[i:i + sub_len] == sub:
            return True

    return False

def visualize_graph(G):
    plt.figure(figsize=(16, 10))
    # 设置绘图布局，可以使用 spring_layout, circular_layout 等
    pos = nx.spring_layout(G)  # 使用 spring 布局，使节点均匀分布

    # 绘制节点
    nx.draw_networkx_nodes(G, pos, node_size=300, node_color='lightblue', alpha=0.9)
    
    # 绘制有向边，箭头大小可以调整
    nx.draw_networkx_edges(G, pos, arrowstyle='-|>', arrowsize=15, edge_color='black', connectionstyle='arc3,rad=0.1')

    # # 绘制节点标签
    # nx.draw_networkx_labels(G, pos, font_size=12, font_color='black')

    # # 为每条边添加标签
    # edge_labels = nx.get_edge_attributes(G, 'description') 
    # nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, font_color='red')

    # 显示图形
    plt.title("Directed Graph Visualization")
    plt.axis('off')  # 关闭坐标轴
    plt.show()

def report_max_lanecount():
    # path = "/home/lsm/SFTSG_NME/src/approach/combined_reports_strict_rule/n_merged"
    path = "/home/lsm/SFTSG_NME/src/approach/information_ex_results_pro"
    json_files = glob.glob(os.path.join(path, '*.json'))
    n =0
    for json_file in json_files:
        with open(json_file, 'r') as f:
            data = json.load(f)
        if data["laneCount"] > 2:
            n += 1
            # print("The maximum lane count is: ", data["laneCount"])
    print(n/len(json_files))
    # 42% of one report's maximum lanecount > 2
    # 34.9% of n_merged reports' maximum lanecount > 2
    # 42% of two merged reports' maximum lanecount > 2

def maxlanecount_used_in_report():
    path = "/home/lsm/SFTSG_NME/src/approach/combined_reports_strict_rule/two_merged"
    # path = "/home/lsm/SFTSG_NME/src/approach/information_ex_results_pro"
    json_files = glob.glob(os.path.join(path, '*.json'))
    n =0
    for json_file in json_files:
        with open(json_file, 'r') as f:
            data = json.load(f)
        for key in data["carInformation"].keys():
            behaviors = data["carInformation"][key]["behaviors"]
            for behavior in behaviors:
                # print(behavior)
                flag = 0
                if len(behavior)>1:
                    if behavior[1]>2:
                        n += 1
                        flag = 1
                if flag == 1:
                    break
    print(n, len(json_files))            
    print(n/len(json_files))
    # 6.6% of one report's behaviors'maximum lanecount > 2 
    # 8.7% of n_merged reports' behaviors' maximum lanecount > 2
    # 9.3% of two merged reports' behaviors' maximum lanecount > 2

def Normalize(vector):
    """Normalize a vector and handle the zero-vector case."""
    norm = np.linalg.norm(vector)
    if norm < 1e-8:  # handle near-zero vectors
        return np.zeros_like(vector)
    return vector / norm

def quaternion_conjugate(q):
    """Compute the conjugate of a quaternion."""
    return np.array([q[0], -q[1], -q[2], -q[3]])

def quaternion_multiply(q1, q2):
    """Multiply two quaternions."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    ])

def calculate_rotation_to_target(current_position, next_position, current_quaternion):
    # Step 1: Calculate the relative direction vector
    direction_vector = next_position - current_position
    
    # Step 2: Normalize the direction vector
    target_direction = Normalize(direction_vector)
    if np.allclose(target_direction, 0):
        print("Next position is the same as the current position.")
        return current_quaternion  # No rotation needed

    # Step 3: Calculate the current heading vector from the quaternion
    default_forward = np.array([1, 0, 0])  # Assuming forward is along X-axis
    v_current = quaternion_multiply(
        quaternion_multiply(current_quaternion, np.append([0], default_forward)),
        quaternion_conjugate(current_quaternion)
    )[1:]  # Take the vector part (ignore the scalar part)

    # Step 4: Calculate the rotation axis and angle
    rotation_axis = np.cross(v_current, target_direction)
    if np.allclose(rotation_axis, 0):
        # Handle the case where the direction is the same or exactly opposite
        if np.allclose(v_current, target_direction):
            print("Already facing the target direction.")
            return current_quaternion  # No rotation needed
        else:
            # Opposite direction: 180 degrees rotation around any perpendicular axis
            rotation_axis = np.array([1, 0, 0]) if abs(v_current[0]) < 1e-8 else np.array([0, 1, 0])

    rotation_axis = Normalize(rotation_axis)
    cos_theta = np.dot(v_current, target_direction)
    cos_theta = np.clip(cos_theta, -1.0, 1.0)  # Clamp to avoid numerical errors
    theta = np.arccos(cos_theta)

    # Step 5: Create the rotation quaternion
    q_rotation = np.array([
        np.cos(theta / 2),
        rotation_axis[0] * np.sin(theta / 2),
        rotation_axis[1] * np.sin(theta / 2),
        rotation_axis[2] * np.sin(theta / 2)
    ])

    # Calculate the new orientation quaternion
    new_quaternion = quaternion_multiply(q_rotation, current_quaternion)
    return new_quaternion




def toNum(val):
    val = str(val)
    if val[-1] == '?':
        val = val[:-1]
    return float(val)

def aa():
    print(int(1/2-0.5))
    print(int(3/2-0.5))
    print(int(5/2-0.5))

def unitnormalize(lst):
    # 计算欧几里得范数（L2范数）
    norm = math.sqrt(sum(x ** 2 for x in lst))
    
    # 检查范数是否为0，以避免除以0的错误
    if norm == 0:
        # 如果范数为0，返回一个全0的列表
        return [0] * len(lst)
    else:
        # 单位化列表
        normalized_lst = [x / norm for x in lst]
        return normalized_lst
    
def dianji(list1,list2):
    # 计算两个列表的点积
    return sum(a*b for a,b in zip(list1,list2))
    
def get_distance(list1,list2):
    # 计算两个列表的欧几里得距离
    return math.sqrt(sum((a-b)**2 for a,b in zip(list1,list2)))

def draw_map_and_trajs(all_vehs_waypoints):
    # map_lon_lat = [[(592662,4134409),(592692,4134440),(592701,4134439),(592741,4134402)],
    #                 [(592750,4134415),(592710,4134452),(592714,4134464),(592748,4134500)],
    #                 [(592735,4134509),(592701,4134473),(592688,4134474),(592655,4134505)],
    #                 [(592646,4134492),(592679,4134461),(592681,4134451),(592651,4134420)],
    #                 [(592654,4134414),(592684,4134445),(592692,4134440)],
    #                 [(592745,4134409),(592705,4134446),(592710,4134452)],
    #                 [(592740,4134504),(592706,4134468),(592701,4134473)],
    #                 [(592650,4134500),(592683,4134469),(592679,4134461)],
    #                 [(592658,4134411.5),(592688,4134442.5)],[(592743,4134405.5),(592703,4134442.5)],
    #                 [(592747.5,4134412),(592707.5,4134449)],[(592710,4134466),(592744,4134502)],
    #                 [(592703.5,4134470.5),(592737.5,4134506.5)],[(592685.5,4134471.5),(592652.5,4134502.5)],
    #                 [(592648,4134496),(592681,4134465)],[(592682.5,4134448),(592652.5,4134417)]
    #             ]
    # map_lon_lat = [[[70.0000152587891, -57.9999923706055], [39.0000076293945, -27.9999961853027], [40.0000076293945, -18.9999961853027], [77.0000076293945, 21.0000076293945]], 
    #                [[64, 30.0000114440918], [27.0000038146973, -9.99999618530273], [15.0000028610229, -5.99999809265137], [-21.0000057220459, 28.0000019073486]], 
    #                [[-30.0000057220459, 14.9999961853027], [6.00000286102295, -19], [5.00000429153442, -32.0000038146973], [-25.9999961853027, -65.0000152587891]], 
    #                [[-12.9999942779541, -74.0000076293945], [18.0000057220459, -41], [28.0000076293945, -38.9999961853027], [59.0000152587891, -68.9999923706055]], 
    #                [[65.0000152587891, -65.9999923706055], [34.0000076293945, -35.9999961853027], [39.0000076293945, -27.9999961853027]], 
    #                [[70, 25.0000114440918], [33.0000076293945, -14.9999961853027], [27.0000038146973, -9.99999618530273]], 
    #                [[-25.0000057220459, 19.9999961853027], [11.0000028610229, -14.0000009536743], [6.00000286102295, -19]], 
    #                [[-20.9999942779541, -70.0000076293945], [10.0000047683716, -37.0000038146973], [18.0000057220459, -41]],
    #                  [[67.5000152587891, -61.9999923706055], [36.5000076293945, -31.9999961853027]], 
    #                  [[73.5000076293945, 23.0000076293945], [36.5000076293945, -16.9999961853027]], 
    #                  [[67, 27.5000114440918], [30.0000057220459, -12.4999961853027]], 
    #                  [[13.0000028610229, -9.99999809265137], [-23.0000057220459, 24.0000019073486]], 
    #                  [[8.50000286102295, -16.5], [-27.5000057220459, 17.4999961853027]], 
    #                  [[7.50000476837158, -34.5000038146973], [-23.4999942779541, -67.5000076293945]],
    #                    [[-16.9999942779541, -72.0000076293945], [14.0000057220459, -39]], 
    #                    [[31.0000076293945, -37.4999961853027], [62.0000152587891, -67.4999923706055]]]
    
    # map = [ [[0,0],[0,0],[0,0],[0,0]],
    #        [[0,0],[0,0],[0,0],[0,0]],
    #        [[0,0],[0,0],[0,0],[0,0]],
    #        [[0,0],[0,0],[0,0],[0,0]],
    #        [[0,0],[0,0],[0,0]],
    #        [[0,0],[0,0],[0,0]],
    #        [[0,0],[0,0],[0,0]],
    #        [[0,0],[0,0],[0,0]],

    #        [[0,0],[0,0]],
    #        [[0,0],[0,0]],
    #        [[0,0],[0,0]],
    #        [[0,0],[0,0]],
    #        [[0,0],[0,0]],
    #        [[0,0],[0,0]],
    #        [[0,0],[0,0]],
    #        [[0,0],[0,0]]

    # ]
    # env = Env()
    # SIMULATOR_HOST = env.str("LGSVL__SIMULATOR_HOST", "127.0.0.1")
    # SIMULATOR_PORT = env.int("LGSVL__SIMULATOR_PORT", 8181)
    # vehicle_conf = env.str("LGSVL__VEHICLE_0", lgsvl.wise.DefaultAssets.ego_lincoln2017mkz_apollo6_modular)
    # scene_name = env.str("LGSVL__MAP", "12da60a7-2fc9-474d-a62a-5cc08cb97fe8")
    # sim = lgsvl.Simulator(SIMULATOR_HOST, SIMULATOR_PORT)
    # if sim.current_scene == scene_name:
    #     sim.reset()
    # else:
    #     sim.load(scene_name)
    # for i in range(0,4):
    #     map[i][0][0] = sim.map_from_gps(easting=map_lon_lat[i][0][0],northing=map_lon_lat[i][0][1],altitude=10).position.x
    #     map[i][0][1] = sim.map_from_gps(easting=map_lon_lat[i][0][0],northing=map_lon_lat[i][0][1],altitude=10).position.z

    #     map[i][1][0] = sim.map_from_gps(easting=map_lon_lat[i][1][0],northing=map_lon_lat[i][1][1],altitude=10).position.x
    #     map[i][1][1] = sim.map_from_gps(easting=map_lon_lat[i][1][0],northing=map_lon_lat[i][1][1],altitude=10).position.z

    #     map[i][2][0] = sim.map_from_gps(easting=map_lon_lat[i][2][0],northing=map_lon_lat[i][2][1],altitude=10).position.x
    #     map[i][2][1] = sim.map_from_gps(easting=map_lon_lat[i][2][0],northing=map_lon_lat[i][2][1],altitude=10).position.z

    #     map[i][3][0] = sim.map_from_gps(easting=map_lon_lat[i][3][0],northing=map_lon_lat[i][3][1],altitude=10).position.x
    #     map[i][3][1] = sim.map_from_gps(easting=map_lon_lat[i][3][0],northing=map_lon_lat[i][3][1],altitude=10).position.z

    # for i in range(4,8):
    #     map[i][0][0] = sim.map_from_gps(easting=map_lon_lat[i][0][0],northing=map_lon_lat[i][0][1],altitude=10).position.x
    #     map[i][0][1] = sim.map_from_gps(easting=map_lon_lat[i][0][0],northing=map_lon_lat[i][0][1],altitude=10).position.z
    #     map[i][1][0] = sim.map_from_gps(easting=map_lon_lat[i][1][0],northing=map_lon_lat[i][1][1],altitude=10).position.x
    #     map[i][1][1] = sim.map_from_gps(easting=map_lon_lat[i][1][0],northing=map_lon_lat[i][1][1],altitude=10).position.z
    #     map[i][2][0] = sim.map_from_gps(easting=map_lon_lat[i][2][0],northing=map_lon_lat[i][2][1],altitude=10).position.x
    #     map[i][2][1] = sim.map_from_gps(easting=map_lon_lat[i][2][0],northing=map_lon_lat[i][2][1],altitude=10).position.z

    # for i in range(8,16):
    #     map[i][0][0] = sim.map_from_gps(easting=map_lon_lat[i][0][0],northing=map_lon_lat[i][0][1],altitude=10).position.x
    #     map[i][0][1] = sim.map_from_gps(easting=map_lon_lat[i][0][0],northing=map_lon_lat[i][0][1],altitude=10).position.z
    #     map[i][1][0] = sim.map_from_gps(easting=map_lon_lat[i][1][0],northing=map_lon_lat[i][1][1],altitude=10).position.x
    #     map[i][1][1] = sim.map_from_gps(easting=map_lon_lat[i][1][0],northing=map_lon_lat[i][1][1],altitude=10).position.z

    
    # map = [ [tuple(inner_list) for inner_list in sublist] for sublist in map ]
    # print(map)  
    map = [[(70.0000152587891, -57.9999923706055), (39.0000076293945, -27.9999961853027), (40.0000076293945, -18.9999961853027), (77.0000076293945, 21.0000076293945)], 
     [(64, 30.0000114440918), (27.0000038146973, -9.99999618530273), (15.0000028610229, -5.99999809265137), (-21.0000057220459, 28.0000019073486)], 
     [(-30.0000057220459, 14.9999961853027), (6.00000286102295, -19), (5.00000429153442, -32.0000038146973), (-25.9999961853027, -65.0000152587891)],
       [(-12.9999942779541, -74.0000076293945), (18.0000057220459, -41), (28.0000076293945, -38.9999961853027), (59.0000152587891, -68.9999923706055)],
       [(65.0000152587891, -65.9999923706055), (34.0000076293945, -35.9999961853027), (39.0000076293945, -27.9999961853027)], 
       [(70, 25.0000114440918), (33.0000076293945, -14.9999961853027), (27.0000038146973, -9.99999618530273)], 
       [(-25.0000057220459, 19.9999961853027), (11.0000028610229, -14.0000009536743), (6.00000286102295, -19)],
         [(-20.9999942779541, -70.0000076293945), (10.0000047683716, -37.0000038146973), (18.0000057220459, -41)], 
         [(67.5000152587891, -61.9999923706055), (36.5000076293945, -31.9999961853027)], 
         [(73.5000076293945, 23.0000076293945), (36.5000076293945, -16.9999961853027)],
           [(67, 27.5000114440918), (30.0000057220459, -12.4999961853027)], 
           [(13.0000028610229, -9.99999809265137), (-23.0000057220459, 24.0000019073486)], 
           [(8.50000286102295, -16.5), (-27.5000057220459, 17.4999961853027)], 
           [(7.50000476837158, -34.5000038146973), (-23.4999942779541, -67.5000076293945)],
             [(-16.9999942779541, -72.0000076293945), (14.0000057220459, -39)],
             [(31.0000076293945, -37.4999961853027), (62.0000152587891, -67.4999923706055)]
             ]     

    
    # Separate the x and y coordinates for each list
    x0, y0 = zip(*map[0])
    x1, y1 = zip(*map[1])
    x2, y2 = zip(*map[2])
    x3, y3 = zip(*map[3])
    x4, y4 = zip(*map[4])
    x5, y5 = zip(*map[5])
    x6, y6 = zip(*map[6])
    x7, y7 = zip(*map[7])
    x8, y8 = zip(*map[8])
    x9, y9 = zip(*map[9])
    x10, y10 = zip(*map[10])
    x11, y11 = zip(*map[11])
    x12, y12 = zip(*map[12])
    x13, y13 = zip(*map[13])
    x14, y14 = zip(*map[14])
    x15, y15 = zip(*map[15])
    
    
    waypoints = []
    for veh in all_vehs_waypoints.keys():
        waypoints.append([])
    i = 0
    for veh in all_vehs_waypoints.keys():
        for wp in all_vehs_waypoints[veh]:
            if wp not in [["start"],["follow lane"],["turn left"],["turn right"],["turn around"],["change lane"],["go across"],["drive into"],["drive off"],["retrograde"],["stop"]]:
                # print(wp)
                waypoints[i].append((wp[0], wp[2]))
        i += 1
    # print(waypoints)
    waypoints_dict = {}
    i = 1
    for veh in waypoints:
        x,y = zip(*veh)
        waypoints_dict[f"veh_x_{i}"] = x
        waypoints_dict[f"veh_y_{i}"] = y
        i += 1

    # Create a plot
    plt.figure(figsize=(12, 8))
    plt.plot(x0, y0, marker='o', color='black',label='List 0')
    plt.plot(x1, y1, marker='o', color='black',label='List 1')
    plt.plot(x2, y2, marker='o', color='black',label='List 2')
    plt.plot(x3, y3, marker='o', color='black',label='List 3')
    plt.plot(x4, y4, marker='o', color='black',label='List 4')
    plt.plot(x5, y5, marker='o', color='black',label='List 5')
    plt.plot(x6, y6, marker='o', color='black',label='List 6')
    plt.plot(x7, y7, marker='o', color='black',label='List 7')
    plt.plot(x8, y8, marker='o', color='black',linestyle='--',label='List 8')
    plt.plot(x9, y9, marker='o', color='black',linestyle='--',label='List 9')
    plt.plot(x10, y10, marker='o', color='black',linestyle='--',label='List 10')
    plt.plot(x11, y11, marker='o', color='black',linestyle='--',label='List 11')
    plt.plot(x12, y12, marker='o', color='black',linestyle='--',label='List 12')
    plt.plot(x13, y13, marker='o', color='black',linestyle='--',label='List 13')
    plt.plot(x14, y14, marker='o', color='black',linestyle='--',label='List 14')
    plt.plot(x15, y15, marker='o', color='black',linestyle='--',label='List 15')

    for i in range(1,len(all_vehs_waypoints)+1):
        plt.plot(waypoints_dict[f"veh_x_{i}"], waypoints_dict[f"veh_y_{i}"], marker='o', linestyle='--',label=f"Vehicle {i}")
    

    plt.gca().axis('off')
    plt.title("Connected Points from Two Lists")
    plt.legend()
    plt.show()

def test():
    a = {"a":1, "b":2, "c":3}
    print(a.keys()-"a")
    for k in a.keys()-"a":
        print(k)
# print(calculate_rotation_to_target(np.array([0,0,0]), np.array([1,0,0]), np.array([0,-1,0,0])))
# aa()
# print(get_distance([37.700008392334, 10, -30],[14.0000028610229, 10, -7.99999809265137]))

# draw_map_and_trajs()
test()