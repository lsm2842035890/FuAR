import os
import sys
import threading
import atexit
import json
import glob
import random
import time
import shutil
import lgsvl
from lgsvl.dreamview import CoordType
from lgsvl import Transform, Vector
from datetime import datetime
from environs import Env
import numpy as np
import pandas as pd
# 获取当前文件的绝对路径
current_file_path = os.path.abspath(__file__)
# 获取当前文件所在目录（RQ1）
current_dir = os.path.dirname(current_file_path)
# 获取父目录（RQ）
parent_dir = os.path.dirname(current_dir)
# 获取祖父目录（approach）[3,6](@ref)
approach_dir = os.path.dirname(parent_dir)

# 将approach目录添加到模块搜索路径[1,7](@ref)
if approach_dir not in sys.path:
    sys.path.insert(0, approach_dir) 
from utils_lsm import *
from solvewaypoints import *
from simulate_report import *
from CyberBridge import CyberBridge
from CyberBridge import Topics
from lgsvl_method import CyberBridgeInstance

def fitness(runtime_data_folder_path,file_name,fianl_pos_lon_lat):
    fitness_score = 0
    ego_npc_distance_score = 0
    ego_speed_score = 0
    ego_fastaccl_hbrake_score = 0
    ego_unreach_score = 0
    try:
        with open(f"{runtime_data_folder_path}/{file_name}_scenario.json", "r") as f:
            runtime_data = json.load(f)
            ego_npc_distance_score = cal_ego_npc_distance_score(runtime_data['Ego'], runtime_data['Obstacles'],0.1)
            ego_speed_score = cal_ego_speed_score(runtime_data['Ego'],1)
            ego_fastaccl_hbrake_score = cal_fastaccl_hbrake_score(runtime_data['Ego'],1)
            ego_unreach_score = cal_unreach_score(runtime_data['Ego'], fianl_pos_lon_lat,1)
        fitness_score = ego_npc_distance_score + ego_speed_score + ego_fastaccl_hbrake_score + ego_unreach_score
    except Exception as e:
        pass
    return fitness_score

def cal_ego_npc_distance_score(ego_data: dict, npc_data: dict, interval=0.1):
    """
    分析时间窗口内的碰撞风险
    返回: [(时间戳, 最小距离, 综合分数), ...]
    """
    results = []
    max_risk_score = 0
    ego_times = sorted(map(float, ego_data.keys()))
    npc_times = sorted(map(float, npc_data.keys()))
    
    current_time = max(min(ego_times), min(npc_times))
    end_time = min(max(ego_times), max(npc_times))
    
    while current_time <= end_time:
        # 时间对齐
        ego_time = min(ego_times, key=lambda t: abs(t-current_time))
        npc_time = min(npc_times, key=lambda t: abs(t-current_time))
        
        # 计算所有NPC距离
        distances = calculate_distances(
            ego_data[str(ego_time)]['position'],
            npc_data[str(npc_time)]
        )
        
        # 转换为综合分数
        scores = distance_to_scores(distances)
        agg_score = aggregate_scores(scores, method='harmonic')
        
        # results.append((current_time, min(distances), agg_score))
        current_time += interval
        max_risk_score = max(max_risk_score,agg_score)
    
    return max_risk_score

def calculate_distances(ego_pos: dict, npc_list: list):
    """计算ego与所有NPC的欧氏距离"""
    return [np.linalg.norm([
        ego_pos['x'] - npc['position']['x'],
        ego_pos['y'] - npc['position']['y'],
        ego_pos['z'] - npc['position']['z']
    ]) for npc in npc_list]

def distance_to_scores(distances: list, max_distance=100.0):
    """将距离列表转换为风险分数列表（距离越小分数越高）"""
    return [max(0, 1 - d/max_distance) for d in distances]

def aggregate_scores(scores: list, method='min') :
    """聚合多个NPC的分数为当前时刻综合分数"""
    if not scores:  # 空列表处理
        return 0.0
    
    if method == 'min':
        return min(scores) if scores else 0.0
    elif method == 'harmonic':
        # 处理全零分数的情况
        if all(s == 0 for s in scores):
            return 0.0
        # 添加小量防止除零
        harmonic_sum = sum(1/(s+1e-6) for s in scores)
        return len(scores) / harmonic_sum if harmonic_sum != 0 else 0.0
    else:
        return np.mean(scores) if scores else 0.0

def calculate_speed(velocity: dict) :
    """计算三维速度标量（km/h）"""
    mps = math.sqrt(velocity['x']**2 + velocity['y']**2 + velocity['z']**2)
    return mps * 3.6

def interval_sampling(ego_data: dict, interval_sec: float) :
    """
    按固定时间间隔采样数据
    参数:
        interval_sec: 采样间隔(秒)
    """
    df = pd.DataFrame.from_dict(ego_data, orient='index')
    df.index = pd.to_datetime(df.index, unit='s')
    resampled = df.resample(f'{interval_sec}S').first()
    return {str(t.timestamp()): data.to_dict() for t, data in resampled.iterrows()}

def cal_ego_speed_score(ego_data: dict, speed_limit=80.0, sample_interval=1.0) :
    """
    改进版超速分析（带间隔采样）
    参数:
        sample_interval: 采样间隔(秒)
    """
    sampled_data = interval_sampling(ego_data, sample_interval)
    
    speeding_count = total_samples = 0
    for timestamp, data in sampled_data.items():
        speed = calculate_speed(data['linearVelocity'])
        total_samples += 1
        if speed > speed_limit:
            speeding_count += 1
    
    return (speeding_count / total_samples) * 100 if total_samples > 0 else 0  

def calculate_acceleration_magnitude(accel: dict) :
    """计算三维加速度标量（m/s²）"""
    return math.sqrt(accel['x']**2 + accel['y']**2 + accel['z']**2)

def interval_sampling(ego_data: dict, interval_sec: float):
    """
    按固定时间间隔采样数据
    参数:
        interval_sec: 采样间隔(秒)
    返回:
        采样后的数据字典
    """
    # 转换为DataFrame并设置时间索引
    df = pd.DataFrame.from_dict(ego_data, orient='index')
    df.index = pd.to_datetime(df.index, unit='s')
    
    # 重采样取每个间隔内的第一个数据点
    resampled = df.resample(f'{interval_sec}S').first()
    
    # 转换回原始字典格式
    return {str(t.timestamp()): data.to_dict() 
            for t, data in resampled.iterrows()}

def cal_fastaccl_hbrake_score(ego_data: dict, threshold=4.0, sample_interval=1.0) :
    """
    改进版加速度分析（带间隔采样）
    参数:
        sample_interval: 采样间隔(秒)
    """
    # 先进行间隔采样
    sampled_data = interval_sampling(ego_data, sample_interval)
    
    pos_over = neg_over = total = 0
    for timestamp, data in sampled_data.items():
        accel = data['linearAcceleration']
        magnitude = calculate_acceleration_magnitude(accel)
        total += 1
        
        # 判断加速度方向
        vel = data['linearVelocity']
        direction = 1 if (accel['x']*vel['x'] + accel['y']*vel['y']) >= 0 else -1
        
        if direction * magnitude > threshold:
            pos_over += 1
        elif direction * magnitude < -threshold:
            neg_over += 1
    
    risk_score = ((pos_over + neg_over) / total) * 100 if total > 0 else 0
    return risk_score

def calculate_distance(pos1: dict, pos2) :
    """计算三维欧几里得距离"""
    return math.sqrt((pos1['x']-pos2[0])**2 + 
                    (pos1['y']-pos2[0])**2)

def cal_unreach_score(ego_data: dict, target_pos, sample_interval=1.0) :
    """
    查找距离目标位置最近的采样点
    参数:
        target_pos: 目标位置字典 {'x':x, 'y':y, 'z':z}
        sample_interval: 采样间隔(秒)
    返回:
        (最近距离, 对应时间戳, 风险分数)
    """
    # 按时间间隔采样
    timestamps = sorted(map(float, ego_data.keys()))
    sampled_ts = [ts for i, ts in enumerate(timestamps) 
                 if i % int(sample_interval/(timestamps[1]-timestamps[0])) == 0]
    
    # 计算各采样点距离
    min_dist = float('inf')
    closest_ts = None
    for ts in sampled_ts:
        dist = calculate_distance(ego_data[str(ts)]['position'], target_pos)
        if dist < min_dist:
            min_dist = dist
            closest_ts = ts
    
    # 风险分数（距离越大分数越高，使用反比例函数）
    risk_score = 100 * (1 - math.exp(-min_dist/1000))  # 缩放因子1000可调整
    
    return risk_score

def genetic_algorithm(folder_path,target_path,runtime_data_folder_path):
    seed_dict = {}
     # 确保目标文件夹存在
    os.makedirs(target_path, exist_ok=True)
    
    # 获取两个文件夹中的文件列表
    target_files = [f for f in os.listdir(target_path) if f.endswith('.json')]
    folder_files = [f for f in os.listdir(folder_path) if f.endswith('.json')]
    
    if target_files:
        # 情况1：target_path中有文件，创建seed_dict
        for filename in target_files:
            file_path = os.path.join(target_path, filename)
            with open(file_path, 'r') as f:
                data = json.load(f)
                if 'fitness' in data:
                    # 获取不带.json的文件名作为key
                    key = os.path.splitext(filename)[0]
                    seed_dict[key] = data['fitness']
    else:
        # 情况2：target_path中没有文件，复制并添加fitness字段
        for filename in folder_files:
            src_path = os.path.join(folder_path, filename)
            dst_path = os.path.join(target_path, filename)
            
            # 复制文件
            shutil.copy2(src_path, dst_path)
            
            # 添加fitness字段
            with open(dst_path, 'r+') as f:
                data = json.load(f)
                data['fitness'] = 0  # 初始化为0
                f.seek(0)
                json.dump(data, f, indent=4)
                f.truncate()

            # 添加到seed_dict
            key = os.path.splitext(filename)[0]
            seed_dict[key] = 0  # 默认值为0
    
    map,map_lon_lat =search_in_map(None)
    start_time = time.time()
    total_time = 0
    while total_time < 7200*2:
        total_time = time.time() - start_time
        print(seed_dict)
        sorted_items = sorted(seed_dict.items(), key=lambda x: x[1], reverse=True)
        # 计算前10%的数量（至少保留1项）
        total = len(sorted_items)
        top_count = max(1, int(round(total * 0.1)))  # 四舍五入处理小数
        # 提取键名并添加.json后缀
        top_files = [f"{item[0]}" for item in sorted_items[:top_count]]

        # rand = random.random()
        rand = 0.81
        if rand < 0.8:
            # 交叉
            parents = random.sample(top_files, k=2) if len(top_files) >= 2 else (top_files[0], top_files[0])
            parent1, parent2 = parents
            print(f"交叉：{parent1} {parent2}")
            child1_name, child2_name,child1_score,child2_score = crossover(parent1, parent2, target_path, runtime_data_folder_path, map, map_lon_lat,True)
            with open(f"{target_path}/{child1_name}.json", "r+") as f:
                data = json.load(f)
                data['fitness'] = child1_score
                f.seek(0)
                json.dump(data, f, indent=4)
                f.truncate()
            with open(f"{target_path}/{child2_name}.json", "r+") as f:
                data = json.load(f)
                data['fitness'] = child2_score
                f.seek(0)
                json.dump(data, f, indent=4)
                f.truncate()
            seed_dict[child1_name] = child1_score
            seed_dict[child2_name] = child2_score
        elif rand < 0.9:
            # 变异
            parent = random.choice(top_files)
            print(f'变异：{parent}')
            child_name,child_score = mutate_individual(parent, target_path, runtime_data_folder_path, map, map_lon_lat,True)
            with open(f"{target_path}/{child_name}.json", "r+") as f:
                data = json.load(f)
                data['fitness'] = child_score
                f.seek(0)
                json.dump(data, f, indent=4)
                f.truncate()
            seed_dict[child_name] = child_score
        else:
            parent = random.choice(top_files)
            print(f'保留：{parent}')
            with open(f"{target_path}/{parent}.json", "r") as f:
                parent_data = json.load(f)
            parent_data = get_merged_report_reasonable(parent_data)
            all_vehs_symbol = trajectory_analysis(parent_data)
            all_vehs_waypoints = trajectory_calculators(all_vehs_symbol, map)
            location_thread = threading.Thread(target=run_cyberbridgeinstance,args=(runtime_data_folder_path,parent))
            location_thread.daemon = True
            location_thread.start()  # 启动线程
            final_pos_lon_lat = start_svl_simulation_nooryes_apollo(all_vehs_waypoints,all_vehs_symbol,map,map_lon_lat,parent_data,True)
            if cyber:
                cyber.save_pose_perception_json()  # 执行保存操作
            seed_dict[parent] = fitness(runtime_data_folder_path,parent,final_pos_lon_lat)     
            with open(f"{target_path}/{parent}.json", "r+") as f:
                data = json.load(f)
                data['fitness'] = seed_dict[parent]
                f.seek(0)
                json.dump(data, f, indent=4)
                f.truncate()

def crossover(parent1, parent2, target_path, runtime_data_folder_path, map, map_lon_lat,flag_apollo=True):
    child1_name = f'{parent1}_{parent2}'
    child2_name = f'{parent2}_{parent1}'
    child1_score = 0
    child2_score = 0
    # 读取json文件
    parent1_data = {}
    parent2_data = {}
    with open(f"{target_path}/{parent1}.json", "r") as f:
        parent1_data = json.load(f)
    with open(f"{target_path}/{parent2}.json", "r") as f:
        parent2_data = json.load(f)
    #crossover
    p1_veh_info = parent1_data["carInformation"]
    p2_veh_info = parent2_data["carInformation"]
    p1_veh_id = list(p1_veh_info.keys())
    p2_veh_id = list(p2_veh_info.keys())
    p1_crossover_point = random.randint(0,len(p1_veh_id)-1)
    p2_crossover_point = random.randint(0,len(p2_veh_id)-1)
    print(p1_crossover_point,p2_crossover_point)
    temp = parent1_data["carInformation"][p1_veh_id[p1_crossover_point]]
    parent1_data["carInformation"][p1_veh_id[p1_crossover_point]] = parent2_data["carInformation"][p2_veh_id[p2_crossover_point]]
    parent2_data["carInformation"][p2_veh_id[p2_crossover_point]] = temp
    with open(f"{target_path}/{child1_name}.json", "w") as f:
        json.dump(parent1_data, f, indent=4)
    with open(f"{target_path}/{child2_name}.json", "w") as f:
        json.dump(parent2_data, f, indent=4)
    
    child1_data = parent1_data
    child2_data = parent2_data

    #run simulation child1
    child1_data = get_merged_report_reasonable(child1_data)
    # print(child1_data)
    all_vehs_symbol = trajectory_analysis(child1_data)
    # print(all_vehs_symbol)
    all_vehs_waypoints = trajectory_calculators(all_vehs_symbol, map)
    # print(all_vehs_waypoints)
    location_thread = threading.Thread(target=run_cyberbridgeinstance,args=(runtime_data_folder_path,child1_name))
    location_thread.daemon = True
    location_thread.start()  # 启动线程

    final_pos_lon_lat = start_svl_simulation_nooryes_apollo(all_vehs_waypoints,all_vehs_symbol,map,map_lon_lat,child1_data,flag_apollo)

    if cyber:
        cyber.save_pose_perception_json()  # 执行保存操作

    child1_score = fitness(runtime_data_folder_path,child1_name,final_pos_lon_lat)

    #run simulation child2
    child2_data = get_merged_report_reasonable(child2_data)
    all_vehs_symbol = trajectory_analysis(child2_data)
    all_vehs_waypoints = trajectory_calculators(all_vehs_symbol, map)
    location_thread = threading.Thread(target=run_cyberbridgeinstance,args=(runtime_data_folder_path,child2_name))
    location_thread.daemon = True
    location_thread.start()  # 启动线程

    final_pos_lon_lat = start_svl_simulation_nooryes_apollo(all_vehs_waypoints,all_vehs_symbol,map,map_lon_lat,child2_data,flag_apollo)

    if cyber:
        cyber.save_pose_perception_json()  # 执行保存操作

    child2_score = fitness(runtime_data_folder_path,child2_name,final_pos_lon_lat)

    return child1_name, child2_name, child1_score, child2_score

def mutate_individual(parent, target_path, runtime_data_folder_path, map, map_lon_lat,flag_apollo=True):
    child_name = f'{parent}_child'
    child_score = 0
    parent_data = {}
    child_data = {}
    # 读取json文件
    with open(f"{target_path}/{parent}.json", "r") as f:
        parent_data = json.load(f)
    # mutate point
    veh_info = parent_data["carInformation"]
    veh_id = list(veh_info.keys())
    mutate_point = random.randint(0,len(veh_id)-1)
    veh_behaviors = veh_info[veh_id[mutate_point]]["behaviors"]
    length_behaviors = len(veh_behaviors)
    if length_behaviors==0:
        mutate_behavior = 0
    else:
        mutate_behavior = random.randint(0,length_behaviors-1)
    behavior_type = random.choice([['turn left', random.randint(1,2)],['turn right', random.randint(1,2)]])
    parent_data['carInformation'][veh_id[mutate_point]]['behaviors'].insert(mutate_behavior,behavior_type)
    child_data = parent_data
    with open(f"{target_path}/{child_name}.json", "w") as f:
        json.dump(child_data, f, indent=4)
    #run simulation child
    child_data = get_merged_report_reasonable(child_data)
    all_vehs_symbol = trajectory_analysis(child_data)
    all_vehs_waypoints = trajectory_calculators(all_vehs_symbol, map)
    location_thread = threading.Thread(target=run_cyberbridgeinstance,args=(runtime_data_folder_path,child_name))
    location_thread.daemon = True
    location_thread.start()  # 启动线程

    final_pos_lon_lat = start_svl_simulation_nooryes_apollo(all_vehs_waypoints,all_vehs_symbol,map,map_lon_lat,child_data,flag_apollo)

    if cyber:
        cyber.save_pose_perception_json()  # 执行保存操作

    child_score = fitness(runtime_data_folder_path,child_name,final_pos_lon_lat)

    return child_name, child_score

def run_cyberbridgeinstance(directory_name,name):
    global cyber
    cyber = CyberBridgeInstance()
    cyber.register(None,[0,0],[],"san_francisco",True,directory_name,name)



if __name__ == '__main__':
    genetic_algorithm('/home/lsm/SFTSG_NME/src/approach/information_ex_results_pro','/home/lsm/SFTSG_NME/src/approach/RQ/RQ1/fuar_seed','/home/lsm/SFTSG_NME/src/approach/RQ/RQ1/fuar_genetic_runtime_data')