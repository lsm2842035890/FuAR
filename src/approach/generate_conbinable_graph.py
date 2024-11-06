import os
import glob
import json
import networkx as nx
import matplotlib.pyplot as plt
from utils_lsm import *
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
    
def loosen_rule(A, B):
    pass

# 创建有向图生成函数
def create_graph():
    instances = []
    directory_path = '/home/lsm/SFTSG_NME/src/approach/information_ex_results_pro'
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

if __name__ == '__main__':
    G = create_graph()
    visualize_graph(G)