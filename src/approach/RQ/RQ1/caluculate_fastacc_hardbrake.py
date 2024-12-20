import os
import json
import glob
import math

def calculate_fastacc_hardbrake_count(json_path):
    json_files = glob.glob(os.path.join(json_path, '*.json'))
    fastacc_count = 0
    hardbrake_count = 0
    exceed_speed_count = 0
    for json_file in json_files:
        flag =0
        with open(json_file, 'r') as f:
            data = json.load(f)
            data_ego = data['Ego']
            for key in data_ego.keys():
                x = data_ego[key]['linearVelocity']['x']
                y = data_ego[key]['linearVelocity']['y']
                z = data_ego[key]['linearVelocity']['z']
                speed = math.sqrt(x**2 + y**2 + z**2)
                if speed >60:
                    exceed_speed_count += 1
                    flag = 1
                if flag == 1:
                    break    
    for json_file in json_files:
        flag = 0
        with open(json_file, 'r') as f:
            data = json.load(f)
            data_ego = data['Ego']
            for key in data_ego.keys():
                a1 = data_ego[key]["linearAcceleration"]["x"]
                a2 = data_ego[key]["linearAcceleration"]["y"]
                a3 = data_ego[key]["linearAcceleration"]["z"]
                acc = math.sqrt(a1**2 + a2**2 + a3**2)
                if data_ego[key]['linearVelocity']['x']>0 and data_ego[key]['linearVelocity']['y']>0 and acc >4:
                    fastacc_count += 1
                    flag = 1
                if flag == 1:
                    break
    for json_file in json_files:
        flag = 0
        with open(json_file, 'r') as f:
            data = json.load(f)
            data_ego = data['Ego']
            for key in data_ego.keys():
                a1 = data_ego[key]["linearAcceleration"]["x"]
                a2 = data_ego[key]["linearAcceleration"]["y"]
                a3 = data_ego[key]["linearAcceleration"]["z"]
                acc = math.sqrt(a1**2 + a2**2 + a3**2)
                if data_ego[key]['linearVelocity']['x']<0 and data_ego[key]['linearVelocity']['y']<0 and acc >4:
                    hardbrake_count += 1
                    flag = 1
                if flag == 1:
                    break
    return fastacc_count, hardbrake_count, exceed_speed_count

if __name__ == '__main__':
    json_path = '/home/lsm/SFTSG_NME/src/approach/runtime_scenaio_data/n_merged'
    fastacc_count, hardbrake_count, exceed_speed_count = calculate_fastacc_hardbrake_count(json_path)
    print('fastacc_count:', fastacc_count)
    print('hardbrake_count:', hardbrake_count)
    print('exceed_speed_count:', exceed_speed_count)