import os
import glob
import json
import networkx as nx
from generate_conbinable_graph import *
from utils_lsm import *

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
            with open(f'/home/lsm/SFTSG_NME/src/approach/combined_reports_strict_rule/two_merged/2_{cnt}.json', 'w') as json_file:
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
                    with open(f'/home/lsm/SFTSG_NME/src/approach/combined_reports_strict_rule/n_merged/3_{cnt}.json', 'w') as json_file:
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
                    with open(f'/home/lsm/SFTSG_NME/src/approach/combined_reports_strict_rule/n_merged/{length+1}_{cnt}.json', 'w') as json_file:
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

G = create_graph()
# visualize_graph(G)
generate_combined_reports_strict_rule(G, None,6)