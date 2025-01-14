import os
import random
import shutil
import json
import sys

def generate_random_reports(source_dir, random_report_output_dir, num_of_reports, fuse_of_num):
 
    # 获取文件夹中所有的 JSON 文件
    all_files = [file for file in os.listdir(source_dir) if file.endswith('.json')]

    if len(all_files) < num_of_reports:
        raise ValueError("文件夹中的 JSON 文件数量不足以组成一组！")

    unique_groups = set()
    selected_groups = []

    while len(selected_groups) < fuse_of_num:
        # 随机选择 n 个文件
        group = tuple(sorted(random.sample(all_files, num_of_reports)))

        # 确保选择的组与之前的组不完全相同
        if group not in unique_groups:
            unique_groups.add(group)
            selected_groups.append(group)
    file_index = 0
    result = selected_groups
    for i, group in enumerate(result):
        print(group)
        
        data_in_group = []
        for j in range(num_of_reports):
            with open(os.path.join(source_dir, group[j]), 'r') as f:
                data_in_group.append(json.load(f))
        random_fuse_data = {}
        
        random_fuse_data["weather"] = random.choice([item["weather"] for item in data_in_group])
        random_fuse_data["lighting"] = random.choice([item["lighting"] for item in data_in_group])
        random_fuse_data["speedlimit"] = random.choice([item["speedlimit"] for item in data_in_group])
        random_fuse_data["intersection"] = random.choice([item["intersection"] for item in data_in_group])
        random_fuse_data["T"] = random.choice([item["T"] for item in data_in_group])
        random_fuse_data["laneCount"] = random.choice([item["laneCount"] for item in data_in_group])
        random_fuse_data["striker"] = random.choice([item["striker"] for item in data_in_group])
        random_fuse_data["At-Fault-Vehicle"] = random.choice([item["At-Fault-Vehicle"] for item in data_in_group])
        random_fuse_data["impactPart"] = random.choice([item["impactPart"] for item in data_in_group])
        random_fuse_data["ID"] = [item["ID"] for item in data_in_group]
        random_fuse_data["description"] = [item["description"] for item in data_in_group]

        random_fuse_data["carInformation"] = {}
        index = 1
        for item in data_in_group:
            for key in item["carInformation"].keys():
                KEY = "V"+str(index)
                random_fuse_data["carInformation"][KEY] = item["carInformation"][key]
                index += 1
        random_fuse_data["carCount"] = len(random_fuse_data["carInformation"].keys())
        with open(os.path.join(random_report_output_dir, fr"{num_of_reports}_{file_index}.json"), 'w') as f:
            json.dump(random_fuse_data, f, indent=4)
        file_index += 1



if __name__ == '__main__':
    source_dir = "/home/lsm/SFTSG_NME/src/approach/RQ/RQ2/raw_data"
    random_report_output_dir = "/home/lsm/SFTSG_NME/src/approach/RQ/RQ2/random_fused_data/random_report"
    generate_random_reports(source_dir, random_report_output_dir, 5, 10)
