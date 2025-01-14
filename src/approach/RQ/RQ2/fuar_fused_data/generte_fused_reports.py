import os
import random
import shutil
import json
import sys
sys.path.append('/home/lsm/SFTSG_NME/src/approach')
from utils_lsm import *

def random_select_json_files(source_folder, target_folder, num_files):
    # 确保目标文件夹存在
    if not os.path.exists(target_folder):
        os.makedirs(target_folder)

    # 获取源文件夹中所有的 JSON 文件
    json_files = [f for f in os.listdir(source_folder) if f.endswith('.json')]

    # 检查文件数量是否足够
    if len(json_files) < num_files:
        raise ValueError(f"源文件夹中的 JSON 文件数量不足，共有 {len(json_files)} 个，无法选择 {num_files} 个。")

    # 随机选择指定数量的文件
    selected_files = random.sample(json_files, num_files)

    # 将选中的文件复制到目标文件夹
    for file_name in selected_files:
        source_path = os.path.join(source_folder, file_name)
        target_path = os.path.join(target_folder, file_name)
        shutil.copy(source_path, target_path)

    print(f"已成功将 {num_files} 个 JSON 文件复制到 {target_folder}。")

class report_node:
    def __init__(self, json_file):
        self.data = None
        self.load_json(json_file)
    
    def load_json(self, json_file):
        try:
            with open(json_file, 'r', encoding='utf-8') as file:
                self.data = json.load(file)
        except FileNotFoundError:
            print(f"文件未找到：{json_file}")
        except json.JSONDecodeError:
            print(f"JSON文件格式错误:{json_file}")

    def get_data(self):
        return self.data
    
    def satify_rule(self, other: "report_node"):
        self_data = self.get_data()
        other_data = other.get_data()
        # print(self_data["ID"], other_data["ID"])
        # sat == ture 则合并成功，merged_vehs为合并的车辆
        sat,merged_vehs_and_strikeroratfault = strict_rule(self_data, other_data)
        # print(merged_vehs)
        return sat,merged_vehs_and_strikeroratfault

def strict_rule(A, B) :
    rule_1 = extract_numbers(A["speedlimit"]) <= extract_numbers(B["speedlimit"])
    rule_2 = A["intersection"] == "no" or B["intersection"] == "yes"
    rule_3 = A["laneCount"] <= B["laneCount"]
    rule_4 = False
    A_carinformation_all_vehs = A["carInformation"].keys()
    B_carinformation_all_vehs = B["carInformation"].keys()
    for x in A_carinformation_all_vehs:
        for y in B_carinformation_all_vehs:
            A_behavior = []
            B_behavior = []
            for i in A["carInformation"][x]["behaviors"]:
                A_behavior.append(i[0])
            for i in B["carInformation"][y]["behaviors"]:
                B_behavior.append(i[0])
            # print(A_behavior, B_behavior)    
            striker = A["striker"]==x and B["striker"]==y
            atfault = A["At-Fault-Vehicle"]==x and B["At-Fault-Vehicle"]==y
            striker_or_atfault = (striker or atfault)
            if striker==True and atfault==False:
                striker_or_atfault_temp = "striker"
            elif striker==False and atfault==True:
                striker_or_atfault_temp = "atfault"
            elif striker==True and atfault==True:
                striker_or_atfault_temp = "strikerandatfault"
            else:
                striker_or_atfault_temp = "nostrikerandatfault"
            if is_continuous_subsequence(A_behavior, B_behavior) and A["carInformation"][x]["direction"] == B["carInformation"][y]["direction"] and A["carInformation"][x]["laneNumber"] == B["carInformation"][y]["laneNumber"] and striker_or_atfault:
                # x is A's merged vehicle, y is B's merged vehicle
                rule_4 = True
                merged_vehs_and_strikeroratfault = [x,y, striker_or_atfault_temp]  #合并的车辆,doushizhuzebeihebing
                break
        if rule_4:
            break
    
    if rule_1 and rule_2 and rule_3 and rule_4:
        # print("suc")
        return True, merged_vehs_and_strikeroratfault
    else:
        return False, []
    

# 创建有向图生成函数
def create_graph(source_folder):
    instances = []
    directory_path = source_folder
    json_files = glob.glob(os.path.join(directory_path, '*.json'))
    for file_path in json_files:
        instances.append(report_node(file_path))

    G = nx.DiGraph()  # 创建一个有向图
    # 添加节点，节点可以是实例的标识
    for i, instance in enumerate(instances):
        G.add_node(i, data=instance.get_data())
    n = 0
    # 根据规则添加有向边
    for i, instance_a in enumerate(instances):
        for j, instance_b in enumerate(instances):
            if i != j:
                res = instance_a.satify_rule(instance_b)
                if res[0]:
                    # description为合并的车辆,for example: [1,2] is A's first vehicle and B's second vehicle merged;
                    G.add_edge(i, j,description=res[1])  # 如果a可以到达b，则添加从a到b的边,
                    n += 1
    print(f"have {n} directed edges")
    return G

def generate_combined_reports_strict_rule(G, output_dir, length=1):
    # length=0 : represents only one report generated to one simulation test case
    # length=1 : represents two reports conbined to one report and generated to one simulation test case
    # length=2 : represents three reports conbined to one report and generated to one simulation test case 
    if length == 0:
        # get jsons from information_ex_results_pro to generate one simulation test case
        pass
    elif length == 1:
        # visualize_graph(G)
        # print("有向图中的边：")
        cnt = 0
        for edge in G.edges():
            combined_report = {}
            print(f"节点 {edge[0]} -> 节点 {edge[1]}")
            print(G.get_edge_data(edge[0], edge[1])["description"])
            description = G.get_edge_data(edge[0], edge[1])["description"]
            A = G.nodes[edge[0]]['data']
            B = G.nodes[edge[1]]['data']
            combined_report["weather"] = A["weather"]+B["weather"]
            combined_report["lighting"] = A["lighting"]+B["lighting"]
            combined_report["speedlimit"] = B["speedlimit"] 
            combined_report["intersection"] = "yes" if A["intersection"] == "yes" or B["intersection"] == "yes" else "no"
            combined_report["T"] = "yes" if A["T"] == "yes" or B["T"] == "yes" else "no"
            combined_report["carCount"] = A["carCount"]+B["carCount"]-1
            combined_report["laneCount"] = B["laneCount"]
            carinfo = {}
            index = 2
            carinfo["V1"]=B["carInformation"][description[1]]  #striker's carinfomation or at-fault-vehicle's carinfomation  and merged vehicle
            for key in A["carInformation"].keys():
                if key != description[0]:
                    KEY = "V"+str(index)
                    carinfo[KEY] = A["carInformation"][key]
                    index += 1
            for key in B["carInformation"].keys():
                if key != description[1]:
                    KEY = "V"+str(index)
                    carinfo[KEY] = B["carInformation"][key]
            combined_report["carInformation"] =carinfo
            combined_report["striker"] = "V1" if description[2] == "striker" or description[2] == "strikerandatfault" else ""
            combined_report["At-Fault-Vehicle"] = "V1" if description[2] == "atfault" or description[2] == "strikerandatfault" else ""
            combined_report["ID"] = [A["ID"],B["ID"]]
            combined_report["description"] = [A["description"],B["description"]]
            # print(combined_report)
            with open(fr"{output_dir}/2_{cnt}.json", 'w') as json_file:
                json.dump(combined_report, json_file, indent=4)
            cnt += 1
        pass
    elif length == 2:
        cnt=0
        path_res = find_paths_length_2_with_same_description(G)
        for node in G.nodes:
            if len(path_res[node]) > 0:
                for i in range(len(path_res[node])):
                    # print(G.nodes[path_res[node][0][1]]['data'])
                    A = G.nodes[path_res[node][i][0]]['data']
                    B = G.nodes[path_res[node][i][1]]['data']
                    C = G.nodes[path_res[node][i][2]]['data']
                    # description = [[v1,v2,striker], [v2,v3,atfault]]
                    description = [G.get_edge_data(path_res[node][i][0], path_res[node][i][1])["description"],G.get_edge_data(path_res[node][i][1], path_res[node][i][2])["description"]]
                    combined_report = {}
                    combined_report["weather"] = A["weather"]+B["weather"]+C["weather"]
                    combined_report["lighting"] = A["lighting"]+B["lighting"]+C["lighting"]
                    combined_report["speedlimit"] = C["speedlimit"] 
                    combined_report["intersection"] = "yes" if A["intersection"] == "yes" or B["intersection"] == "yes" or C["intersection"] == "yes" else "no"
                    combined_report["T"] = "yes" if A["T"] == "yes" or B["T"] == "yes" or C["T"] == "yes" else "no"
                    combined_report["carCount"] = A["carCount"]+B["carCount"]+C["carCount"]-2
                    combined_report["laneCount"] = C["laneCount"]
                    carinfo = {}
                    index = 2
                    carinfo["V1"]=C["carInformation"][description[1][1]]  #striker's carinfomation or at-fault-vehicle's carinfomation  and merged vehicle
                    for key in A["carInformation"].keys():
                        if key != description[0][0]:
                            KEY = "V"+str(index)
                            carinfo[KEY] = A["carInformation"][key]
                            index += 1
                    for key in B["carInformation"].keys():
                        if key != description[0][1]:
                            KEY = "V"+str(index)
                            carinfo[KEY] = B["carInformation"][key]
                            index += 1
                    for key in C["carInformation"].keys():
                        if key != description[1][1]:
                            KEY = "V"+str(index)
                            carinfo[KEY] = C["carInformation"][key]
                    combined_report["carInformation"] =carinfo
                    combined_report["striker"] = "V1" if description[0][2] == "striker" or description[0][2] == "strikerandatfault" else ""
                    combined_report["At-Fault-Vehicle"] = "V1" if description[0][2] == "atfault" or description[0][2] == "strikerandatfault" else ""
                    combined_report["ID"] = [A["ID"],B["ID"],C["ID"]]
                    combined_report["description"] = [A["description"],B["description"],C["description"]]
                    # print(combined_report)
                    with open(f'{output_dir}/3_{cnt}.json', 'w') as json_file:
                        json.dump(combined_report, json_file, indent=4)
                    cnt += 1
    elif length > 2:
        cnt=0
        path_res = find_paths_length_n_with_same_description(G, length)
        # print(path_res)
        for node in G.nodes:
            if len(path_res[node]) > 0:
                # print(path_res[node])
                for path in  path_res[node]:
                    nodecount_in_path = len(path)
                    # print(nodecount_in_path)
                    reportdata_in_path = []
                    for i in range(nodecount_in_path):
                        reportdata_in_path.append(G.nodes[path[i]]['data'])
                    # print(reportdata_in_path)
                    description_in_path = []
                    for i in range(nodecount_in_path-1):
                        description_in_path.append(G.get_edge_data(path[i], path[i+1])["description"])
                    # print(description_in_path)
                    combined_report = {}
                    combined_report["weather"] = []
                    combined_report["lighting"] = []
                    for i in range(len(reportdata_in_path)):
                        combined_report["weather"] = combined_report.get("weather") + reportdata_in_path[i]["weather"]
                        combined_report["lighting"] = combined_report.get("lighting") + reportdata_in_path[i]["lighting"]
                    combined_report["speedlimit"] = reportdata_in_path[-1]["speedlimit"] 
                    combined_report["intersection"] = "no"
                    combined_report["T"] = "no"
                    for i in range(len(reportdata_in_path)):
                        if reportdata_in_path[i]["intersection"] == "yes":
                            combined_report["intersection"] = "yes"
                            break
                    for i in range(len(reportdata_in_path)):
                        if reportdata_in_path[i]["T"] == "yes":
                            combined_report["T"] = "yes"
                            break
                    combined_report["carCount"] = sum([reportdata_in_path[i]["carCount"] for i in range(len(reportdata_in_path))])-nodecount_in_path+1
                    combined_report["laneCount"] = reportdata_in_path[-1]["laneCount"]
                    carinfo = {}
                    index = 2
                    carinfo["V1"]=reportdata_in_path[-1]["carInformation"][description_in_path[-1][1]]  #striker's carinfomation or at-fault-vehicle's carinfomation  and merged vehicle
                    for i in range(len(reportdata_in_path)):
                        if i != len(reportdata_in_path)-1:
                            for key in reportdata_in_path[i]["carInformation"].keys():
                                if key != description_in_path[i][0]:
                                    KEY = "V"+str(index)
                                    carinfo[KEY] = reportdata_in_path[i]["carInformation"][key]
                                    index += 1
                        else:
                            for key in reportdata_in_path[i]["carInformation"].keys():
                                if key != description_in_path[-1][1]:
                                    KEY = "V"+str(index)
                                    carinfo[KEY] = reportdata_in_path[i]["carInformation"][key]
                    combined_report["carInformation"] =carinfo
                    combined_report["striker"] = "V1" if description_in_path[0][2] == "striker" or description_in_path[0][2] == "strikerandatfault" else ""
                    combined_report["At-Fault-Vehicle"] = "V1" if description_in_path[0][2] == "atfault" or description_in_path[0][2] == "strikerandatfault" else ""
                    combined_report["ID"] = [reportdata_in_path[i]["ID"] for i in range(len(reportdata_in_path))]
                    combined_report["description"] = [reportdata_in_path[i]["description"] for i in range(len(reportdata_in_path))]
                    # print(combined_report)
                    with open(f'{output_dir}/{length+1}_{cnt}.json', 'w') as json_file:
                        json.dump(combined_report, json_file, indent=4)
                    cnt += 1
                

def find_paths_length_2_with_same_description(graph):
    paths = {}
    for node in graph.nodes:
        paths[node] = []
        # 遍历节点的第一级邻居
        for neighbor in graph.successors(node):
            # 遍历邻居的下一级邻居，形成长度为2的路径
            for next_node in graph.successors(neighbor):
                # 确保生成的路径不成闭环
                if next_node == node:
                    continue
                # 获取两条边的 description 属性
                edge_data_1 = graph.get_edge_data(node, neighbor)
                edge_data_2 = graph.get_edge_data(neighbor, next_node)
                # description 列表中包含了 3 个值，分别是： hited vehicle in first; hited vehicle in second; striker or at-fault-vehicle or both
                strikeratfaultflag_1 = edge_data_1.get('description')[2]
                strikeratfaultflag_2 = edge_data_2.get('description')[2]
                
                # 检查两条边的 description 是否一致
                if strikeratfaultflag_1 == strikeratfaultflag_2:
                    paths[node].append([node, neighbor, next_node])
    
    return paths

def find_paths_length_n_with_same_description(graph, n):
    paths = {}

    def dfs(current_node, current_path, strikeroratfaultflag):
        # 如果路径长度达到 n+1，则记录路径
        if len(current_path) == n + 1:
            paths[current_path[0]].append(current_path[:])
            return

        # 遍历当前节点的后继节点
        for next_node in graph.successors(current_node):
            # 确保生成的路径不成闭环
            if next_node in current_path:
                continue

            # 获取边的数据
            edge_data = graph.get_edge_data(current_node, next_node)
            if edge_data is None:
                continue

            # 检查 description 是否与路径中的一致
            if edge_data.get('description')[2] == strikeroratfaultflag:
                # 递归搜索下一个节点
                current_path.append(next_node)
                dfs(next_node, current_path, strikeroratfaultflag)
                # 回溯
                current_path.pop()

    # 初始化每个节点作为起始节点
    for node in graph.nodes:
        paths[node] = []
        # 遍历节点的第一级邻居
        for neighbor in graph.successors(node):
            edge_data = graph.get_edge_data(node, neighbor)
            if edge_data is None:
                continue

            # 使用邻居节点的 description 值进行递归搜索
            strikeroratfaultflag = edge_data.get('description')[2]
            dfs(neighbor, [node, neighbor], strikeroratfaultflag)

    return paths

def get_fuar_reports(fuar_report_output_dir, source_dir):
    G = create_graph(source_dir)
    generate_combined_reports_strict_rule(G,fuar_report_output_dir)
    generate_combined_reports_strict_rule(G,fuar_report_output_dir,length=2)
    generate_combined_reports_strict_rule(G,fuar_report_output_dir,length=3)
    generate_combined_reports_strict_rule(G,fuar_report_output_dir,length=4)
    generate_combined_reports_strict_rule(G,fuar_report_output_dir,length=5)
    generate_combined_reports_strict_rule(G,fuar_report_output_dir,length=6)
    generate_combined_reports_strict_rule(G,fuar_report_output_dir,length=7)
    generate_combined_reports_strict_rule(G,fuar_report_output_dir,length=8)
    generate_combined_reports_strict_rule(G,fuar_report_output_dir,length=9)

if __name__ == '__main__':
    source_dir = "/home/lsm/SFTSG_NME/src/approach/RQ/RQ2/raw_data"
    fuar_report_output_dir = "/home/lsm/SFTSG_NME/src/approach/RQ/RQ2/fuar_fused_data/fuar_report"
    get_fuar_reports(fuar_report_output_dir, source_dir)
