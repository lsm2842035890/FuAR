import os
import json
import glob
import random
import time
from utils_lsm import *
from solvewaypoints import *
import lgsvl
from lgsvl.dreamview import CoordType
from lgsvl import Transform, Vector
from datetime import datetime
from environs import Env
  
def simulate_report(report_path,flag_apollo):
    with open(report_path, 'r') as f:
        report = json.load(f)
    # print(report["carInformation"])
    report = get_merged_report_reasonable(report)
    map,map_lon_lat =search_in_map(report)
    all_vehs_symbol = trajectory_analysis(report)
    all_vehs_waypoints = trajectory_calculators(all_vehs_symbol, map)
    draw_map_and_trajs(all_vehs_waypoints)
    start_svl_simulation_nooryes_apollo(all_vehs_waypoints,all_vehs_symbol,map,map_lon_lat,report,flag_apollo)


def get_merged_report_reasonable(report):
    direction_frenquency = [0,0,0,0]
    for key in report["carInformation"].keys():
        if report["carInformation"][key]["direction"] == "north":
            direction_frenquency[0] += 1
        elif report["carInformation"][key]["direction"] == "south":
            direction_frenquency[1] += 1
        elif report["carInformation"][key]["direction"] == "east":
            direction_frenquency[2] += 1
        elif report["carInformation"][key]["direction"] == "west":
            direction_frenquency[3] += 1

    for key in report["carInformation"].keys():
        # remove abnormal values
        remove_index = []
        for index in range (0,len(report["carInformation"][key]["behaviors"])):
            if report["carInformation"][key]["behaviors"][index][0] not in ["follow lane", "turn left", "turn right", "turn around", "stop", "change lane", "go across", "retrograde", "drive off", "drive into"]:
                remove_index.append(index)
            if report["carInformation"][key]["behaviors"][index][0] in ["turn left", "turn right", "turn around", "change lane"] and len(report["carInformation"][key]["behaviors"][index])<2:
                report["carInformation"][key]["behaviors"][index].append(random.choices([1,2],weights=[0.5,0.5],k=1)[0])
        # print(remove_index)
        for index in remove_index:
            report["carInformation"][key]["behaviors"].pop(index)

        temp_behaviors = report["carInformation"][key]["behaviors"]
        zhizhen = 0
        while(zhizhen >= 0) :
            # print("zhizhen",zhizhen)
            # print(temp_behaviors)
            if zhizhen < len(temp_behaviors)-1 and temp_behaviors[zhizhen][0] == "follow lane" :
                if temp_behaviors[zhizhen+1][0] in ["follow lane","drive into","drive off","change lane"]:
                    temp_behaviors.pop(zhizhen+1)
                else:
                    zhizhen += 1
            elif zhizhen < len(temp_behaviors)-1 and temp_behaviors[zhizhen][0] == "turn left":
                if temp_behaviors[zhizhen+1][0] in ["turn left","turn right","go across","drive into"]:
                    temp_behaviors.pop(zhizhen+1)
                else:
                    zhizhen += 1
            elif zhizhen < len(temp_behaviors)-1 and temp_behaviors[zhizhen][0] == "turn right":
                if temp_behaviors[zhizhen+1][0] in ["turn right","turn left","go across","drive into"]:
                    temp_behaviors.pop(zhizhen+1)
                else:
                    zhizhen += 1
            elif zhizhen < len(temp_behaviors)-1 and temp_behaviors[zhizhen][0] == "change lane":
                if temp_behaviors[zhizhen+1][0] in ["follow lane","change lane","drive into","drive off"]:
                    temp_behaviors.pop(zhizhen+1)
                elif temp_behaviors[zhizhen+1][0] == "go across":
                    for i in range(0,zhizhen):
                        if temp_behaviors[i][0] in ["turn left","turn right","go across"]:
                            temp_behaviors.pop(zhizhen+1)
                            break
                else:
                    zhizhen += 1
            elif zhizhen < len(temp_behaviors)-1 and temp_behaviors[zhizhen][0] == "turn around":
                if temp_behaviors[zhizhen+1][0] in ["drive into"]:
                    temp_behaviors.pop(zhizhen+1)
                else:
                    zhizhen += 1
            elif zhizhen < len(temp_behaviors)-1 and temp_behaviors[zhizhen][0] == "go across":
                if temp_behaviors[zhizhen+1][0] in ["turn left","turn right","go across","drive into"]:
                    temp_behaviors.pop(zhizhen+1)
                else:
                    zhizhen += 1
            elif zhizhen < len(temp_behaviors)-1 and temp_behaviors[zhizhen][0] == "drive into":
                if temp_behaviors[zhizhen+1][0] == "drive into":
                    temp_behaviors.pop(zhizhen+1)
                else:
                    zhizhen += 1
            else:
                # print("lsm")
                zhizhen = -1


        if len(report["carInformation"][key]["behaviors"]) == 0:
            report["carInformation"][key]["behaviors"].append(["follow lane"])
            # print("ssa")
        if report["carInformation"][key]["direction"] == "unknown":
            report["carInformation"][key]["direction"] = random.choices(["north", "south", "east", "west"], weights=normalize(direction_frenquency), k=1)[0]
            # print(report["carInformation"][key]["direction"])
        if report["carInformation"][key]["laneNumber"] == 0 or report["carInformation"][key]["laneNumber"] > report["laneCount"]:
            report["carInformation"][key]["laneNumber"] = 1
        if report["intersection"] == "no" and report["T"] == "no":
            if report["carInformation"][key]["behaviors"][-1][0] in [["go across"], ["turn left"], ["turn right"]]:
                report["carInformation"][key]["behaviors"].pop()
                report["carInformation"][key]["behaviors"].append(["follow lane"])
                # print("lsm")
    # print(report["carInformation"])
    return report

def search_in_map(report):
    map = [[],      
        [[],[[0,0,0],[0,0,0]],[[0,0,0],[0,0,0]],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[],[]],
        [[],[[0,0,0],[0,0,0]],[[0,0,0],[0,0,0]],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[],[]],
        [[],[[0,0,0],[0,0,0]],[[0,0,0],[0,0,0]],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[],[]],
        [[],[[0,0,0],[0,0,0]],[[0,0,0],[0,0,0]],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[],[]],
        [[],[[0,0,0],[0,0,0]],[[0,0,0],[0,0,0]],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[],[]],
        [[],[[0,0,0],[0,0,0]],[[0,0,0],[0,0,0]],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[],[]],
        [[],[[0,0,0],[0,0,0]],[[0,0,0],[0,0,0]],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[],[]],
        [[],[[0,0,0],[0,0,0]],[[0,0,0],[0,0,0]],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[],[]]]
    map = [[],
        [[], [[69.0000152587891, 10, -59.9999923706055], [37.700008392334, 10, -30]], [[66.3000183105469, 10, -64.0000076293945], [35.5000076293945, 10, -33.9999961853027]], [90.0000152587891, 10, -75.9999923706055], [67.0000152587891, 10, -54], [-0.7219403581398334, 0.0, 0.6919552870590188], [-1.72745376403327e-06, 313.78515625, 5.08888754974322e-14], [], []],
        [[], [[38.3000030517578, 10, -18], [75.3000106811523, 10, 22.0000076293945]], [[34.8000030517578, 10, -16], [71.8000030517578, 10, 24.0000114440918]], [38.0000076293945, 10, -22.9999961853027], [64.0000076293945, 10, 3.00000762939453], [0.6790419872766388, 0.0, 0.7340994343516369], [0, 43.6904029846191, 0], [], []],
        [[], [[65.5, 10, 28.7000122070313], [28.5000038146973, 10, -11.4999961853027]], [[68.5, 10, 26.2000122070313], [31.5000057220459, 10, -13.4999961853027]], [64, 10, 30.0000114440918], [28.0000038146973, 10, -3.99999618530273], [-0.6772150065758247, 0.0, -0.7357851825556871], [0, 223.804779052734, 0], [], []],
        [[], [[14.0000028610229, 10, -7.99999809265137], [-22.0000057220459, 10, 26.0000019073486]], [[12.0000028610229, 10, -9.99999809265137], [-24.0000057220459, 10, 21.9999961853027]], [22.0000038146973, 10, -7.99999809265137], [-6.00000286102295, 10, 17], [-0.7270132342681785, 0.0, 0.6866234464383826], [0, 313.699340820313, 0], [], []],
        [[], [[-28.7000064849854, 10, 16.1999969482422], [7.30000305175781, 10, -17.8000011444092]], [[-26.2000064849854, 10, 18.6999969482422], [9.80000305175781, 10, -15.3000011444092]], [-52.0000114440918, 10, 34.9999923706055], [-0.999998211860657, 10, -16.0000019073486], [0.7270132625758454, 0.0, -0.6866234164655504], [0, 133.722274780273, 0], [], []],
        [[], [[6.30000495910645, 10, -33.3000030517578], [-24.6999969482422, 10, -66.3000183105469]], [[8.80000495910645, 10, -35.8000030517578], [-22.1999950408936, 10, -68.8000106811523]], [7.00000476837158, 10, -27.0000038146973], [-24.9999961853027, 10, -60.0000114440918], [-0.684675315866033, 0.0, -0.728848209055732], [-6.68073710130557e-07, 223.667877197266, 0], [], []],
        [[], [[-14.9999942779541, 10, -73.0000076293945], [16.0000057220459, 10, -40]], [[-18.9999942779541, 10, -71.0000076293945], [12.0000057220459, 10, -38]], [-29.9999961853027, 10, -90.0000152587891], [12.0000095367432, 10, -52.0000076293945], [0.6846753775758818, 0.0, 0.7288481510858923], [0, 43.6501121520996, 0], [], []],
        [[], [[29.5000076293945, 10, -38.2499961853027], [60.5000152587891, 10, -68.3000106811523]], [[32.5000076293945, 10, -36.7499961853027], [63.5000152587891, 10, -66.8000106811523]], [25.0000076293945, 10, -39.9999961853027], [68.0000152587891, 10, -77.9999923706055], [0.7180230709535472, 0.0, -0.6960193025904076], [0, 133.784973144531, 0], [], []]
    ]
    # null,lane1,lane2,di,do,direction_vec,rotation,lane1_width,lane2_width
    map_lon_lat = [[],      
        [[],[[592660,4134410],[592690,4134441.3]],[[592656,4134412.7],[592686,4134443.5]],[592644,4134389],[592666,4134412],[],[],[],[]],
        [[],[[592702,4134440.7],[592742,4134403.7]],[[592704,4134444.2],[592744,4134407.2]],[592697,4134441],[592723,4134415],[],[],[],[]],
        [[],[[592748.7,4134413.5],[592708.5,4134450.5]],[[592746.2,4134410.5],[592706.5,4134447.5]],[592750,4134415],[592716,4134451],[],[],[],[]],
        [[],[[592712,4134465],[592746,4134501]],[[592710,4134467],[592742,4134503]],[592712,4134457],[592737,4134485],[],[],[],[]],
        [[],[[592736.2,4134507.7],[592702.2,4134471.7]],[[592738.7,4134505.2],[592704.7,4134469.2]],[592755,4134531],[592704,4134480],[],[],[],[]],
        [[],[[592686.7,4134472.7],[592653.7,4134503.7]],[[592684.2,4134470.2],[592651.2,4134501.2]],[592693,4134472],[592660,4134504],[],[],[],[]],
        [[],[[592647,4134494],[592680,4134463]],[[592649,4134498],[592682,4134467]],[592630,4134509],[592668,4134467],[],[],[],[]],
        [[],[[592681.75,4134449.5],[592651.7,4134418.5]],[[592683.25,4134446.5],[592653.2,4134415.5]],[592680,4134454],[592642,4134411],[],[],[],[]]]
    return map,map_lon_lat
    env = Env()
    SIMULATOR_HOST = env.str("LGSVL__SIMULATOR_HOST", "127.0.0.1")
    SIMULATOR_PORT = env.int("LGSVL__SIMULATOR_PORT", 8181)
    vehicle_conf = env.str("LGSVL__VEHICLE_0", lgsvl.wise.DefaultAssets.ego_lincoln2017mkz_apollo6_modular)
    scene_name = env.str("LGSVL__MAP", "12da60a7-2fc9-474d-a62a-5cc08cb97fe8")
    sim = lgsvl.Simulator(SIMULATOR_HOST, SIMULATOR_PORT)
    if sim.current_scene == scene_name:
        sim.reset()
    else:
        sim.load(scene_name)
    for i in range(1,9):
        map[i][1][0][0] = sim.map_from_gps(easting=map_lon_lat[i][1][0][0],northing=map_lon_lat[i][1][0][1],altitude=10).position.x
        map[i][1][0][1] = sim.map_from_gps(easting=map_lon_lat[i][1][0][0],northing=map_lon_lat[i][1][0][1],altitude=10).position.y
        map[i][1][0][2] = sim.map_from_gps(easting=map_lon_lat[i][1][0][0],northing=map_lon_lat[i][1][0][1],altitude=10).position.z
        map[i][1][1][0] = sim.map_from_gps(easting=map_lon_lat[i][1][1][0],northing=map_lon_lat[i][1][1][1],altitude=10).position.x
        map[i][1][1][1] = sim.map_from_gps(easting=map_lon_lat[i][1][1][0],northing=map_lon_lat[i][1][1][1],altitude=10).position.y
        map[i][1][1][2] = sim.map_from_gps(easting=map_lon_lat[i][1][1][0],northing=map_lon_lat[i][1][1][1],altitude=10).position.z
        map[i][2][0][0] = sim.map_from_gps(easting=map_lon_lat[i][2][0][0],northing=map_lon_lat[i][2][0][1],altitude=10).position.x
        map[i][2][0][1] = sim.map_from_gps(easting=map_lon_lat[i][2][0][0],northing=map_lon_lat[i][2][0][1],altitude=10).position.y
        map[i][2][0][2] = sim.map_from_gps(easting=map_lon_lat[i][2][0][0],northing=map_lon_lat[i][2][0][1],altitude=10).position.z
        map[i][2][1][0] = sim.map_from_gps(easting=map_lon_lat[i][2][1][0],northing=map_lon_lat[i][2][1][1],altitude=10).position.x
        map[i][2][1][1] = sim.map_from_gps(easting=map_lon_lat[i][2][1][0],northing=map_lon_lat[i][2][1][1],altitude=10).position.y
        map[i][2][1][2] = sim.map_from_gps(easting=map_lon_lat[i][2][1][0],northing=map_lon_lat[i][2][1][1],altitude=10).position.z
        map[i][3][0] = sim.map_from_gps(easting=map_lon_lat[i][3][0],northing=map_lon_lat[i][3][1],altitude=10).position.x
        map[i][3][1] = sim.map_from_gps(easting=map_lon_lat[i][3][0],northing=map_lon_lat[i][3][1],altitude=10).position.y
        map[i][3][2] = sim.map_from_gps(easting=map_lon_lat[i][3][0],northing=map_lon_lat[i][3][1],altitude=10).position.z
        map[i][4][0] = sim.map_from_gps(easting=map_lon_lat[i][4][0],northing=map_lon_lat[i][4][1],altitude=10).position.x
        map[i][4][1] = sim.map_from_gps(easting=map_lon_lat[i][4][0],northing=map_lon_lat[i][4][1],altitude=10).position.y
        map[i][4][2] = sim.map_from_gps(easting=map_lon_lat[i][4][0],northing=map_lon_lat[i][4][1],altitude=10).position.z

        direction_vec = [0,0,0]
        direction_vec[0] = map[i][1][1][0]-map[i][1][0][0]
        direction_vec[1] = map[i][1][1][1]-map[i][1][0][1]
        direction_vec[2] = map[i][1][1][2]-map[i][1][0][2]
        direction_vec = unitnormalize(direction_vec)
        map[i][5][0] = direction_vec[0]
        map[i][5][1] = direction_vec[1]
        map[i][5][2] = direction_vec[2]
        
        direction_vec = [0,0,0]
        direction_vec[0] = (map[i][1][1][0]+map[i][1][0][0])/2
        direction_vec[1] = (map[i][1][1][1]+map[i][1][0][1])/2
        direction_vec[2] = (map[i][1][1][2]+map[i][1][0][2])/2
        # print(direction_vec)
        one = lgsvl.Vector(direction_vec[0],direction_vec[1],direction_vec[2])
        # print(one)
        # print(sim.map_point_on_lane(one).rotation)
        map[i][6][0] = sim.map_point_on_lane(one).rotation.x
        map[i][6][1] =sim.map_point_on_lane(one).rotation.y
        map[i][6][2] =sim.map_point_on_lane(one).rotation.z
        print(i)
        print(map[i])
    return map

def trajectory_analysis(report):
    all_vehs_symbol = {}
    maproad = [1,2,3,4,5,6,7,8]
    map_lane = [1,2]
    map_road_start = []
    map_road_end = []
    map_drive_into_site = []
    map_drive_off_site = []
    for veh_id in report["carInformation"].keys():
        # print(veh_id)
        driveinto_flag = 0
        driveoff_flag = 0
        retrograde_flag = 0
        road = 0
        lane = 0
        se = 's'
        behaviors = report["carInformation"][veh_id]["behaviors"]
        direction = report["carInformation"][veh_id]["direction"]
        lanenumber = report["carInformation"][veh_id]["laneNumber"]
        if lanenumber >2:
            lanenumber = random.choices(map_lane,[0.5,0.5],k=1)[0]
        all_vehs_symbol[veh_id] = []
        if direction == "north":
            road = 1
            lane = lanenumber
        elif direction == "south":
            road = 5
            lane = lanenumber
        elif direction == "west":
            road = 3
            lane = lanenumber
        elif direction == "east":
            road = 7
            lane = lanenumber
        else:
            print("unknown direction is not getting reasonable")
        all_vehs_symbol[veh_id].append([road,lane,se])
        if behaviors[0][0] == "drive into":
            driveinto_flag = 1
        if behaviors[-1][0] == "drive off":
            driveoff_flag = 1
        if ["retrograde"] in behaviors :
            retrograde_flag = 1
        # print(retrograde_flag)
        # print(all_vehs_symbol[veh_id])
        while len(behaviors) > 0:
            # print(behaviors)
            temp_road = 0
            if  retrograde_flag == 1:
                retrograde_flag = 0
                all_vehs_symbol[veh_id] = [["retrograde"]] + all_vehs_symbol[veh_id]
                all_vehs_symbol[veh_id] = [[all_vehs_symbol[veh_id][-1][0],all_vehs_symbol[veh_id][-1][1],'e']] + all_vehs_symbol[veh_id]
                behaviors.clear()
            elif behaviors[0][0] == "drive into" and driveinto_flag == 1:
                driveinto_flag = 0
                all_vehs_symbol[veh_id] = [["drive into"]] + all_vehs_symbol[veh_id]
                all_vehs_symbol[veh_id] = [[all_vehs_symbol[veh_id][-1][0],all_vehs_symbol[veh_id][-1][1],'di']] + all_vehs_symbol[veh_id]
                behaviors.pop(0)
            elif behaviors[0][0] == "follow lane":
                if all_vehs_symbol[veh_id][-1][2] == 's':
                    all_vehs_symbol[veh_id].append(["follow lane"])
                    all_vehs_symbol[veh_id].append([all_vehs_symbol[veh_id][-2][0],all_vehs_symbol[veh_id][-2][1],'e'])
                    behaviors.pop(0) 
                else:
                    behaviors.pop(0)
            elif behaviors[0][0] == "turn left":
                if all_vehs_symbol[veh_id][-1][2] == 'e':
                    if behaviors[0][1] > 0 and behaviors[0][1] < 3:
                        all_vehs_symbol[veh_id].append(["turn left"])
                        if all_vehs_symbol[veh_id][-2][0]+8-3 >8:
                            temp_road = (all_vehs_symbol[veh_id][-2][0]+8-3)%8
                            all_vehs_symbol[veh_id].append([temp_road,behaviors[0][1],'s'])
                        else:
                            all_vehs_symbol[veh_id].append([all_vehs_symbol[veh_id][-2][0]+8-3,behaviors[0][1],'s'])
                        behaviors.pop(0)
                    else:
                        all_vehs_symbol[veh_id].append(["turn left"])
                        if all_vehs_symbol[veh_id][-2][0]+8-3 >8:
                            temp_road = (all_vehs_symbol[veh_id][-2][0]+8-3)%8
                            all_vehs_symbol[veh_id].append([temp_road,random.choices(map_lane,[0.5,0.5],k=1)[0],'s'])
                        else:
                            all_vehs_symbol[veh_id].append([all_vehs_symbol[veh_id][-2][0]+8-3,random.choices(map_lane,[0.5,0.5],k=1)[0],'s'])
                        behaviors.pop(0)
                elif all_vehs_symbol[veh_id][-1][2] == 's':
                    if behaviors[0][1] > 0 and behaviors[0][1] < 3:
                        all_vehs_symbol[veh_id].append(["follow lane"])
                        all_vehs_symbol[veh_id].append([all_vehs_symbol[veh_id][-2][0],all_vehs_symbol[veh_id][-2][1],'e'])
                        all_vehs_symbol[veh_id].append(["turn left"])
                        if all_vehs_symbol[veh_id][-2][0]+8-3 >8:
                            temp_road = (all_vehs_symbol[veh_id][-2][0]+8-3)%8
                            all_vehs_symbol[veh_id].append([temp_road,behaviors[0][1],'s'])
                        else:
                            all_vehs_symbol[veh_id].append([all_vehs_symbol[veh_id][-2][0]+8-3,behaviors[0][1],'s'])
                        behaviors.pop(0)
                    else:
                        all_vehs_symbol[veh_id].append(["follow lane"])
                        all_vehs_symbol[veh_id].append([all_vehs_symbol[veh_id][-2][0],all_vehs_symbol[veh_id][-2][1],'e'])
                        all_vehs_symbol[veh_id].append(["turn left"])
                        if all_vehs_symbol[veh_id][-2][0]+8-3 >8:
                            temp_road = (all_vehs_symbol[veh_id][-2][0]+8-3)%8
                            all_vehs_symbol[veh_id].append([temp_road,random.choices(map_lane,[0.5,0.5],k=1)[0],'s'])
                        else:
                            all_vehs_symbol[veh_id].append([all_vehs_symbol[veh_id][-2][0]+8-3,random.choices(map_lane,[0.5,0.5],k=1)[0],'s'])
                        behaviors.pop(0)
            elif behaviors[0][0] == "turn right":
                if all_vehs_symbol[veh_id][-1][2] == 'e':
                    if behaviors[0][1] > 0 and behaviors[0][1] < 3:
                        all_vehs_symbol[veh_id].append(["turn right"])
                        all_vehs_symbol[veh_id].append([all_vehs_symbol[veh_id][-2][0]+1,behaviors[0][1],'s'])
                        behaviors.pop(0)
                    else:
                        all_vehs_symbol[veh_id].append(["turn right"])
                        all_vehs_symbol[veh_id].append([all_vehs_symbol[veh_id][-2][0]+1,random.choices(map_lane,[0.5,0.5],k=1)[0],'s'])
                        behaviors.pop(0)
                elif all_vehs_symbol[veh_id][-1][2] == 's':
                    if behaviors[0][1] > 0 and behaviors[0][1] < 3:
                        all_vehs_symbol[veh_id].append(["follow lane"])
                        all_vehs_symbol[veh_id].append([all_vehs_symbol[veh_id][-2][0],all_vehs_symbol[veh_id][-2][1],'e'])
                        all_vehs_symbol[veh_id].append(["turn right"])
                        all_vehs_symbol[veh_id].append([all_vehs_symbol[veh_id][-2][0]+1,behaviors[0][1],'s'])
                        behaviors.pop(0)
                    else:
                        all_vehs_symbol[veh_id].append(["follow lane"])
                        all_vehs_symbol[veh_id].append([all_vehs_symbol[veh_id][-2][0],all_vehs_symbol[veh_id][-2][1],'e'])
                        all_vehs_symbol[veh_id].append(["turn right"])
                        all_vehs_symbol[veh_id].append([all_vehs_symbol[veh_id][-2][0]+1,random.choices(map_lane,[0.5,0.5],k=1)[0],'s'])
                        behaviors.pop(0)
            elif behaviors[0][0] == "turn around":
                if all_vehs_symbol[veh_id][-1][2] == 'e':
                    if behaviors[0][1] > 0 and behaviors[0][1] < 3 and all_vehs_symbol[veh_id][-1][0] % 2 == 1 :
                        all_vehs_symbol[veh_id].append(["turn around"])
                        if all_vehs_symbol[veh_id][-2][0] - 1 == 0:
                            all_vehs_symbol[veh_id].append([8,behaviors[0][1],'s'])
                            behaviors.pop(0)
                        else:
                            all_vehs_symbol[veh_id].append([all_vehs_symbol[veh_id][-2][0]-1,behaviors[0][1],'s'])
                            behaviors.pop(0)
                    elif behaviors[0][1] > 0 and behaviors[0][1] < 3 and all_vehs_symbol[veh_id][-1][0] % 2 ==0:
                        all_vehs_symbol[veh_id].append(["turn around"])
                        if all_vehs_symbol[veh_id][-2][0] + 1 == 9:
                            all_vehs_symbol[veh_id].append([1,behaviors[0][1],'s'])
                            behaviors.pop(0)
                        else:
                            all_vehs_symbol[veh_id].append([all_vehs_symbol[veh_id][-2][0]+1,behaviors[0][1],'s'])
                            behaviors.pop(0)
                    elif (behaviors[0][1] > 3 or  behaviors[0][1]==0)  and all_vehs_symbol[veh_id][-1][0] % 2 == 1 :
                        all_vehs_symbol[veh_id].append(["turn around"])
                        if all_vehs_symbol[veh_id][-2][0] - 1 == 0:
                            all_vehs_symbol[veh_id].append([8,random.choices(map_lane,[0.5,0.5],k=1)[0],'s'])
                            behaviors.pop(0)
                        else:
                            all_vehs_symbol[veh_id].append([all_vehs_symbol[veh_id][-2][0]-1,random.choices(map_lane,[0.5,0.5],k=1)[0],'s'])
                            behaviors.pop(0)
                    elif (behaviors[0][1] > 3 or  behaviors[0][1]==0) and all_vehs_symbol[veh_id][-1][0] % 2 ==0:
                        all_vehs_symbol[veh_id].append(["turn around"])
                        if all_vehs_symbol[veh_id][-2][0] + 1 == 9:
                            all_vehs_symbol[veh_id].append([1,random.choices(map_lane,[0.5,0.5],k=1)[0],'s'])
                            behaviors.pop(0)
                        else:
                            all_vehs_symbol[veh_id].append([all_vehs_symbol[veh_id][-2][0]+1,random.choices(map_lane,[0.5,0.5],k=1)[0],'s'])
                            behaviors.pop(0)
                elif all_vehs_symbol[veh_id][-1][2] == 's':
                    if behaviors[0][1] > 0 and behaviors[0][1] < 3 and all_vehs_symbol[veh_id][-1][0] % 2 == 1 :
                        all_vehs_symbol[veh_id].append(["follow lane"])
                        all_vehs_symbol[veh_id].append([all_vehs_symbol[veh_id][-2][0],all_vehs_symbol[veh_id][-2][1],'e'])
                        all_vehs_symbol[veh_id].append(["turn around"])
                        if all_vehs_symbol[veh_id][-2][0] - 1 == 0:
                            all_vehs_symbol[veh_id].append([8,behaviors[0][1],'s'])
                            behaviors.pop(0)
                        else:
                            all_vehs_symbol[veh_id].append([all_vehs_symbol[veh_id][-2][0]-1,behaviors[0][1],'s'])
                            behaviors.pop(0)
                    elif behaviors[0][1] > 0 and behaviors[0][1] < 3 and all_vehs_symbol[veh_id][-1][0] % 2 ==0:
                        all_vehs_symbol[veh_id].append(["follow lane"])
                        all_vehs_symbol[veh_id].append([all_vehs_symbol[veh_id][-2][0],all_vehs_symbol[veh_id][-2][1],'e'])
                        all_vehs_symbol[veh_id].append(["turn around"])
                        if all_vehs_symbol[veh_id][-2][0] + 1 == 9:
                            all_vehs_symbol[veh_id].append([1,behaviors[0][1],'s'])
                            behaviors.pop(0)
                        else:
                            all_vehs_symbol[veh_id].append([all_vehs_symbol[veh_id][-2][0]+1,behaviors[0][1],'s'])
                            behaviors.pop(0)
                    elif (behaviors[0][1] > 3 or  behaviors[0][1]==0) and all_vehs_symbol[veh_id][-1][0] % 2 == 1 :
                        all_vehs_symbol[veh_id].append(["follow lane"])
                        all_vehs_symbol[veh_id].append([all_vehs_symbol[veh_id][-2][0],all_vehs_symbol[veh_id][-2][1],'e'])
                        all_vehs_symbol[veh_id].append(["turn around"])
                        if all_vehs_symbol[veh_id][-2][0] - 1 == 0:
                            all_vehs_symbol[veh_id].append([8,random.choices(map_lane,[0.5,0.5],k=1)[0],'s'])
                            behaviors.pop(0)
                        else:
                            all_vehs_symbol[veh_id].append([all_vehs_symbol[veh_id][-2][0]-1,random.choices(map_lane,[0.5,0.5],k=1)[0],'s'])
                            behaviors.pop(0)
                    elif (behaviors[0][1] > 3 or  behaviors[0][1]==0) and all_vehs_symbol[veh_id][-1][0] % 2 ==0:
                        all_vehs_symbol[veh_id].append(["follow lane"])
                        all_vehs_symbol[veh_id].append([all_vehs_symbol[veh_id][-2][0],all_vehs_symbol[veh_id][-2][1],'e'])
                        all_vehs_symbol[veh_id].append(["turn around"])
                        if all_vehs_symbol[veh_id][-2][0] + 1 == 9:
                            all_vehs_symbol[veh_id].append([1,random.choices(map_lane,[0.5,0.5],k=1)[0],'s'])
                            behaviors.pop(0)
                        else:
                            all_vehs_symbol[veh_id].append([all_vehs_symbol[veh_id][-2][0]+1,random.choices(map_lane,[0.5,0.5],k=1)[0],'s'])
                            behaviors.pop(0)
            elif behaviors[0][0] == "change lane":
                if all_vehs_symbol[veh_id][-1][2] == 's':
                    if behaviors[0][1] > 0 and behaviors[0][1] < 3:
                        all_vehs_symbol[veh_id].append(["change lane"])
                        all_vehs_symbol[veh_id].append([all_vehs_symbol[veh_id][-2][0],behaviors[0][1],'e'])
                        behaviors.pop(0)
                    else:
                        all_vehs_symbol[veh_id].append(["change lane"])
                        all_vehs_symbol[veh_id].append([all_vehs_symbol[veh_id][-2][0],random.choices(map_lane,[0.5,0.5],k=1)[0],'e'])
                        behaviors.pop(0)
                elif all_vehs_symbol[veh_id][-1][2] == 'e':
                    behaviors.pop(0)
            elif behaviors[0][0] == "stop":
                behaviors.pop(0)
            elif behaviors[0][0] == "go across":
                if all_vehs_symbol[veh_id][-1][2] == 'e':
                    all_vehs_symbol[veh_id].append(["go across"])
                    if all_vehs_symbol[veh_id][-2][0]+3 >8:
                        temp_road = (all_vehs_symbol[veh_id][-2][0]+3)%8
                        all_vehs_symbol[veh_id].append([temp_road,random.choices(map_lane,[0.5,0.5],k=1)[0],'s'])
                    else:
                        all_vehs_symbol[veh_id].append([all_vehs_symbol[veh_id][-2][0]+3,random.choices(map_lane,[0.5,0.5],k=1)[0],'s'])
                    behaviors.pop(0)
                elif all_vehs_symbol[veh_id][-1][2] == 's':
                    all_vehs_symbol[veh_id].append(["follow lane"])
                    all_vehs_symbol[veh_id].append([all_vehs_symbol[veh_id][-2][0],all_vehs_symbol[veh_id][-2][1],'e'])
                    all_vehs_symbol[veh_id].append(["go across"])
                    if all_vehs_symbol[veh_id][-2][0]+3 >8:
                        temp_road = (all_vehs_symbol[veh_id][-2][0]+3)%8
                        all_vehs_symbol[veh_id].append([temp_road,random.choices(map_lane,[0.5,0.5],k=1)[0],'s'])
                    else:
                        all_vehs_symbol[veh_id].append([all_vehs_symbol[veh_id][-2][0]+3,random.choices(map_lane,[0.5,0.5],k=1)[0],'s'])
                    behaviors.pop(0)
            elif behaviors[0][0] == "drive off" and driveoff_flag == 1:
                if all_vehs_symbol[veh_id][-1][2] == 's':
                    all_vehs_symbol[veh_id].append(["drive off"])
                    all_vehs_symbol[veh_id].append([all_vehs_symbol[veh_id][-2][0],all_vehs_symbol[veh_id][-2][1],'do'])
                    behaviors.clear()
            else:
                behaviors.pop(0)
            # print(all_vehs_symbol[veh_id])
    print(all_vehs_symbol)
    # print("lsm")
    return all_vehs_symbol

def trajectory_calculators(all_vehs_symbol,map):
    # /home/lsm/SFTSG_NME/src/approach/combined_reports_strict_rule/n_merged/3_194.json
    # {'V1': [[5, 1, 'e'], ['retrograde'], [5, 1, 's']], 'V2': [[1, 1, 's'], ['follow lane'], [1, 1, 'e']], 'V3': [[1, 1, 's'], ['follow lane'], [1, 1, 'e']], 'V4': [[1, 1, 's'], ['follow lane'], [1, 1, 'e'], ['go across'], [4, 2, 's']]}
    # /home/lsm/SFTSG_NME/src/approach/combined_reports_strict_rule/n_merged/4_101.json
    # {'V1': [[7, 1, 's'], ['follow lane'], [7, 1, 'e'], ['go across'], [2, 1, 's']], 'V2': [[1, 1, 's'], ['follow lane'], [1, 1, 'e'], ['turn left'], [6, 2, 's']], 'V3': [[1, 1, 'di'], ['drive into'], [1, 1, 's']], 'V4': [[1, 1, 's'], ['follow lane'], [1, 1, 'e'], ['go across'], [4, 1, 's']], 'V5': [[1, 1, 's'], ['follow lane'], [1, 1, 'e']]}
    # /home/lsm/SFTSG_NME/src/approach/combined_reports_strict_rule/n_merged/4_78.json
    # {'V1': [[1, 1, 's'], ['follow lane'], [1, 1, 'e'], ['go across'], [4, 2, 's']], 'V2': [[1, 1, 's'], ['follow lane'], [1, 1, 'e']], 'V3': [[1, 1, 'di'], ['drive into'], [1, 1, 's']], 'V4': [[1, 1, 'di'], ['drive into'], [1, 1, 's'], ['follow lane'], [1, 1, 'e'], ['turn around'], [8, 2, 's']], 'V5': [[5, 1, 's'], ['follow lane'], [5, 1, 'e'], ['turn left'], [2, 1, 's']]}
    # /home/lsm/SFTSG_NME/src/approach/combined_reports_strict_rule/two_merged/2_107.json
    # {'V1': [[5, 1, 's'], ['follow lane'], [5, 1, 'e'], ['go across'], [8, 2, 's']], 'V2': [[7, 1, 's'], ['follow lane'], [7, 1, 'e']], 'V3': [[7, 1, 's'], ['follow lane'], [7, 1, 'e'], ['turn right'], [8, 1, 's']]}
    xiaoyu3 = []
    dayu3 = []
    all_vehs_waypoints = {}
    maxlength = 0
    for key in all_vehs_symbol.keys():
        if len(all_vehs_symbol[key]) < 3:
            xiaoyu3.append(key)
            if maxlength < len(all_vehs_symbol[key]):
                maxlength = len(all_vehs_symbol[key])
        else:
            dayu3.append(key)
            if maxlength < len(all_vehs_symbol[key]):
                maxlength = len(all_vehs_symbol[key])
    behaviorscount = {}
    # print('xiaoyu3',xiaoyu3)
    # print('dayu3',dayu3)
    for veh_id in dayu3:
        behaviorscount[veh_id] = 0
        for symbol in all_vehs_symbol[veh_id]:
            if symbol[0] in ["follow lane","turn left","turn right","turn around","change lane","retrograde","go across","drive off","drive into","stop"]:
                behaviorscount[veh_id] += 1
    # maxlength = max(behaviorscount.values())
    dayu3_behaviors_flag = {}
    for veh_id in dayu3:
        dayu3_behaviors_flag[veh_id] = [1 for _ in range(behaviorscount[veh_id])]
        all_vehs_waypoints[veh_id] = []
    for veh_id in xiaoyu3:
        all_vehs_waypoints[veh_id]=[["start"]]
        all_vehs_waypoints[veh_id].append(get_position_from_symbol(all_vehs_symbol[veh_id][0],map))
    # print('dayu3_behaviors_flag',dayu3_behaviors_flag)
    # print('maxlength',maxlength)
    if len(dayu3)==1:
        print('only one vehicle')
        zhi = 1
        while zhi < maxlength:
            if len(all_vehs_waypoints[dayu3[0]]) == 0:
                all_vehs_waypoints[dayu3[0]].append(["start"])
            elif len(all_vehs_waypoints[dayu3[0]]) > 0:
                pass
            print('zhi',zhi)
            ii = dayu3[0]
            i_front = all_vehs_symbol[dayu3[0]][zhi-1]
            i_back = all_vehs_symbol[dayu3[0]][zhi+1]
            i_symbol = all_vehs_symbol[dayu3[0]][zhi]
            jj = ''
            j_front = []
            j_back = []
            j_symbol = []
            flag_index = int(zhi/2-0.5)
            iiorjj = ii
            all_vehs_waypoints =calculate_one_veh(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map,iiorjj)
            dayu3_behaviors_flag[iiorjj][flag_index] = 0
            zhi += 2
            
    zhi = 1
    while zhi <= maxlength:
        # print('zhi',zhi)
        for i in range(0,len(dayu3)):
            for j in range(i+1,len(dayu3)):
                    
                # for zhenfan in range(0,2):
                #     print('ii_id',dayu3[i],'jj_id',dayu3[j],'zhenfan',zhenfan)
                #     if zhenfan == 0:
                #         ii = dayu3[i]
                #         i_front = all_vehs_symbol[dayu3[i]][zhi-1]
                #         i_back = all_vehs_symbol[dayu3[i]][zhi+1]
                #         i_symbol = all_vehs_symbol[dayu3[i]][zhi]
                #         jj = dayu3[j]
                #         j_front = all_vehs_symbol[dayu3[j]][zhi-1]
                #         j_back = all_vehs_symbol[dayu3[j]][zhi+1]
                #         j_symbol = all_vehs_symbol[dayu3[j]][zhi]
                #     else:
                #         ii = dayu3[j]
                #         i_front = all_vehs_symbol[dayu3[j]][zhi-1]
                #         i_back = all_vehs_symbol[dayu3[j]][zhi+1]
                #         i_symbol = all_vehs_symbol[dayu3[j]][zhi]
                #         jj = dayu3[i]
                #         j_front = all_vehs_symbol[dayu3[i]][zhi-1]
                #         j_back = all_vehs_symbol[dayu3[i]][zhi+1]
                #         j_symbol = all_vehs_symbol[dayu3[i]][zhi]
                    
                    ii = dayu3[i]
                    # i_front = all_vehs_symbol[dayu3[i]][zhi-1]
                    # i_back = all_vehs_symbol[dayu3[i]][zhi+1]
                    # i_symbol = all_vehs_symbol[dayu3[i]][zhi]
                    jj = dayu3[j]
                    # j_front = all_vehs_symbol[dayu3[j]][zhi-1]
                    # j_back = all_vehs_symbol[dayu3[j]][zhi+1]
                    # j_symbol = all_vehs_symbol[dayu3[j]][zhi]
                    flag_index = int(zhi/2-0.5)
                    # print('flag_index',flag_index)
                    if flag_index<len(dayu3_behaviors_flag[ii]) and flag_index<len(dayu3_behaviors_flag[jj]):
                        #####
                        for zhenfan in range(0,2):
                            if zhenfan == 0:
                                ii = dayu3[i]
                                i_front = all_vehs_symbol[dayu3[i]][zhi-1]
                                i_back = all_vehs_symbol[dayu3[i]][zhi+1]
                                i_symbol = all_vehs_symbol[dayu3[i]][zhi]
                                jj = dayu3[j]
                                j_front = all_vehs_symbol[dayu3[j]][zhi-1]
                                j_back = all_vehs_symbol[dayu3[j]][zhi+1]
                                j_symbol = all_vehs_symbol[dayu3[j]][zhi]
                            else:
                                ii = dayu3[j]
                                i_front = all_vehs_symbol[dayu3[j]][zhi-1]
                                i_back = all_vehs_symbol[dayu3[j]][zhi+1]
                                i_symbol = all_vehs_symbol[dayu3[j]][zhi]
                                jj = dayu3[i]
                                j_front = all_vehs_symbol[dayu3[i]][zhi-1]
                                j_back = all_vehs_symbol[dayu3[i]][zhi+1]
                                j_symbol = all_vehs_symbol[dayu3[i]][zhi]
                        #####
                            if dayu3_behaviors_flag[ii][flag_index] == 1 and dayu3_behaviors_flag[jj][flag_index] == 1:
                                if i_symbol==["follow lane"] and j_symbol==["follow lane"]:
                                    if i_front[0] == j_front[0] and i_front[1] == j_front[1]:
                                        all_vehs_waypoints = calculate_follow_lane_and_follow_lane(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map)
                                        dayu3_behaviors_flag[ii][flag_index] = 0
                                        dayu3_behaviors_flag[jj][flag_index] = 0
                                    else:
                                        iiorjj = ii
                                        all_vehs_waypoints =calculate_one_veh(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map,iiorjj)
                                        dayu3_behaviors_flag[iiorjj][flag_index] = 0
                                        iiorjj = jj
                                        all_vehs_waypoints =calculate_one_veh(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map,iiorjj)
                                        dayu3_behaviors_flag[iiorjj][flag_index] = 0
                                # elif i_symbol==["follow lane"] and j_symbol==["turn left"]:
                                #     if i_back[0]==j_front[0] and i_back[1]==j_front[1]:
                                #         all_vehs_waypoints =calculate_follow_lane_and_turn_left (all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map)
                                #         dayu3_behaviors_flag[ii][flag_index] = 0
                                #         dayu3_behaviors_flag[jj][flag_index] = 0
                                # elif i_symbol==["follow lane"] and j_symbol==["turn right"]:
                                #     if i_back[0]==j_front[0] and i_back[1]==j_front[1]:
                                #         all_vehs_waypoints =calculate_follow_lane_and_turn_right(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map)
                                #         dayu3_behaviors_flag[ii][flag_index] = 0
                                #         dayu3_behaviors_flag[jj][flag_index] = 0
                                # elif i_symbol==["follow lane"] and j_symbol==["turn around"]:
                                #     if i_back[0]==j_front[0] and i_back[1]==j_front[1]:
                                #         all_vehs_waypoints =calculate_follow_lane_and_turn_around(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map)
                                #         dayu3_behaviors_flag[ii][flag_index] = 0
                                #         dayu3_behaviors_flag[jj][flag_index] = 0
                                elif i_symbol==["follow lane"] and j_symbol==["change lane"]:
                                    if i_front[0] == j_front[0]:
                                        all_vehs_waypoints =calculate_follow_lane_and_change_lane(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map)
                                        dayu3_behaviors_flag[ii][flag_index] = 0
                                        dayu3_behaviors_flag[jj][flag_index] = 0
                                    else:
                                        iiorjj = ii
                                        all_vehs_waypoints =calculate_one_veh(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map,iiorjj)
                                        dayu3_behaviors_flag[iiorjj][flag_index] = 0
                                        iiorjj = jj
                                        all_vehs_waypoints =calculate_one_veh(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map,iiorjj)
                                        dayu3_behaviors_flag[iiorjj][flag_index] = 0
                                elif i_symbol==["follow lane"] and j_symbol==["retrograde"]:
                                    if i_front[0] == j_front[0] and i_front[1] == j_front[1]:
                                        all_vehs_waypoints =calculate_follow_lane_and_retrograde(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map)
                                        dayu3_behaviors_flag[ii][flag_index] = 0
                                        dayu3_behaviors_flag[jj][flag_index] = 0
                                    else:
                                        iiorjj = ii
                                        all_vehs_waypoints =calculate_one_veh(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map,iiorjj)
                                        dayu3_behaviors_flag[iiorjj][flag_index] = 0
                                        iiorjj = jj
                                        all_vehs_waypoints =calculate_one_veh(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map,iiorjj)
                                        dayu3_behaviors_flag[iiorjj][flag_index] = 0
                                elif i_symbol==["turn left"] and j_symbol==["turn left"]:
                                    if i_front[0] == j_front[0]:
                                        if i_front[1] == j_front[1]:
                                            all_vehs_waypoints =calculate_turn_left_and_turn_left(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map)
                                            dayu3_behaviors_flag[ii][flag_index] = 0
                                            dayu3_behaviors_flag[jj][flag_index] = 0
                                        elif i_front[1] != j_front[1] and i_back[1] == j_back[1]:
                                            all_vehs_waypoints =calculate_turn_left_and_turn_left(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map)
                                            dayu3_behaviors_flag[ii][flag_index] = 0
                                            dayu3_behaviors_flag[jj][flag_index] = 0
                                        elif i_front[1] != j_front[1] and i_back[1] != i_front[1] and j_back[1] != j_front[1]:
                                            all_vehs_waypoints =calculate_turn_left_and_turn_left(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map)
                                            dayu3_behaviors_flag[ii][flag_index] = 0
                                            dayu3_behaviors_flag[jj][flag_index] = 0
                                        elif abs(i_front[0] - j_front[0]) != 4:
                                            all_vehs_waypoints =calculate_turn_left_and_turn_left(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map)
                                            dayu3_behaviors_flag[ii][flag_index] = 0
                                            dayu3_behaviors_flag[jj][flag_index] = 0
                                        else:
                                            iiorjj = ii
                                            all_vehs_waypoints =calculate_one_veh(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map,iiorjj)
                                            dayu3_behaviors_flag[iiorjj][flag_index] = 0
                                            iiorjj = jj
                                            all_vehs_waypoints =calculate_one_veh(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map,iiorjj)
                                            dayu3_behaviors_flag[iiorjj][flag_index] = 0
                                    else:
                                        iiorjj = ii
                                        all_vehs_waypoints =calculate_one_veh(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map,iiorjj)
                                        dayu3_behaviors_flag[iiorjj][flag_index] = 0
                                        iiorjj = jj
                                        all_vehs_waypoints =calculate_one_veh(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map,iiorjj)
                                        dayu3_behaviors_flag[iiorjj][flag_index] = 0
                                elif i_symbol==["turn left"] and j_symbol==["turn right"]:
                                    if i_front[0] == j_front[0] and i_front[1] == j_front[1]:
                                        all_vehs_waypoints =calculate_turn_left_and_turn_right(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map)
                                        dayu3_behaviors_flag[ii][flag_index] = 0
                                        dayu3_behaviors_flag[jj][flag_index] = 0
                                    elif i_front[0] == j_front[0] and i_front[1] ==1 and j_front[1] ==2:
                                        all_vehs_waypoints =calculate_turn_left_and_turn_right(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map)
                                        dayu3_behaviors_flag[ii][flag_index] = 0
                                        dayu3_behaviors_flag[jj][flag_index] = 0
                                    elif abs(i_front[0] - j_front[0]) == 4 and i_back[1] == j_back[1]:
                                        all_vehs_waypoints =calculate_turn_left_and_turn_right(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map)
                                        dayu3_behaviors_flag[ii][flag_index] = 0
                                        dayu3_behaviors_flag[jj][flag_index] = 0
                                    elif abs(i_front[0] - j_front[0]) == 4 and i_back[1] ==1 and j_back[1] ==2:
                                        all_vehs_waypoints =calculate_turn_left_and_turn_right(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map)
                                        dayu3_behaviors_flag[ii][flag_index] = 0
                                        dayu3_behaviors_flag[jj][flag_index] = 0
                                    else:
                                        iiorjj = ii
                                        all_vehs_waypoints =calculate_one_veh(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map,iiorjj)
                                        dayu3_behaviors_flag[iiorjj][flag_index] = 0
                                        iiorjj = jj
                                        all_vehs_waypoints =calculate_one_veh(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map,iiorjj)
                                        dayu3_behaviors_flag[iiorjj][flag_index] = 0
                                elif i_symbol==["turn left"] and j_symbol==["turn around"]:
                                    if [i_front[0],j_front[0]] in [[1,7],[3,1],[5,3],[7,5]] and i_back[1]==j_back[1]:
                                        all_vehs_waypoints =calculate_turn_left_and_turn_around(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map)
                                        dayu3_behaviors_flag[ii][flag_index] = 0
                                        dayu3_behaviors_flag[jj][flag_index] = 0
                                    elif [i_front[0],j_front[0]] in [[1,7],[3,1],[5,3],[7,5]] and i_back[1]==2 and j_back[1]==1:
                                        all_vehs_waypoints =calculate_turn_left_and_turn_around(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map)
                                        dayu3_behaviors_flag[ii][flag_index] = 0
                                        dayu3_behaviors_flag[jj][flag_index] = 0
                                    elif i_front[0] == j_front[0] and i_front[1] == j_front[1]:
                                        all_vehs_waypoints =calculate_turn_left_and_turn_around(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map)
                                        dayu3_behaviors_flag[ii][flag_index] = 0
                                        dayu3_behaviors_flag[jj][flag_index] = 0
                                    elif i_front[0] == j_front[0] and i_front[1] ==2 and j_front[1] ==1:
                                        all_vehs_waypoints =calculate_turn_left_and_turn_around(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map)
                                        dayu3_behaviors_flag[ii][flag_index] = 0
                                        dayu3_behaviors_flag[jj][flag_index] = 0
                                    else:
                                        iiorjj = ii
                                        all_vehs_waypoints =calculate_one_veh(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map,iiorjj)
                                        dayu3_behaviors_flag[iiorjj][flag_index] = 0
                                        iiorjj = jj
                                        all_vehs_waypoints =calculate_one_veh(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map,iiorjj)
                                        dayu3_behaviors_flag[iiorjj][flag_index] = 0
                                elif i_symbol==["turn left"] and j_symbol==["go across"]:
                                    if i_front[0]!=j_front[0]:
                                        all_vehs_waypoints =calculate_turn_left_and_go_across(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map)
                                        dayu3_behaviors_flag[ii][flag_index] = 0
                                        dayu3_behaviors_flag[jj][flag_index] = 0
                                    elif i_front[0] == j_front[0] and i_front[1] == j_front[1]:
                                        all_vehs_waypoints =calculate_turn_left_and_go_across(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map)
                                        dayu3_behaviors_flag[ii][flag_index] = 0
                                        dayu3_behaviors_flag[jj][flag_index] = 0
                                    elif i_front[0] == j_front[0] and i_front[1] ==1 and j_front[1] ==2:
                                        all_vehs_waypoints =calculate_turn_left_and_go_across(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map)
                                        dayu3_behaviors_flag[ii][flag_index] = 0
                                        dayu3_behaviors_flag[jj][flag_index] = 0
                                    else:
                                        iiorjj = ii
                                        all_vehs_waypoints =calculate_one_veh(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map,iiorjj)
                                        dayu3_behaviors_flag[iiorjj][flag_index] = 0
                                        iiorjj = jj
                                        all_vehs_waypoints =calculate_one_veh(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map,iiorjj)
                                        dayu3_behaviors_flag[iiorjj][flag_index] = 0
                                elif i_symbol==["turn right"] and j_symbol==["turn right"]:
                                    if i_front[0] == j_front[0]:
                                        if i_front[1] == j_front[1]:
                                            all_vehs_waypoints =calculate_turn_right_and_turn_right(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map)
                                            dayu3_behaviors_flag[ii][flag_index] = 0
                                            dayu3_behaviors_flag[jj][flag_index] = 0
                                        elif i_front[1] != j_front[1] and i_back[1] != i_front[1] and j_back[1] != j_front[1]:
                                            all_vehs_waypoints =calculate_turn_right_and_turn_right(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map)
                                            dayu3_behaviors_flag[ii][flag_index] = 0
                                            dayu3_behaviors_flag[jj][flag_index] = 0
                                        elif i_front[1] != j_front[1] and i_back[1] == j_back[1]:
                                            all_vehs_waypoints =calculate_turn_right_and_turn_right(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map)
                                            dayu3_behaviors_flag[ii][flag_index] = 0
                                            dayu3_behaviors_flag[jj][flag_index] = 0
                                        else:
                                            iiorjj = ii
                                            all_vehs_waypoints =calculate_one_veh(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map,iiorjj)
                                            dayu3_behaviors_flag[iiorjj][flag_index] = 0
                                            iiorjj = jj
                                            all_vehs_waypoints =calculate_one_veh(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map,iiorjj)
                                            dayu3_behaviors_flag[iiorjj][flag_index] = 0
                                    else:
                                        iiorjj = ii
                                        all_vehs_waypoints =calculate_one_veh(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map,iiorjj)
                                        dayu3_behaviors_flag[iiorjj][flag_index] = 0
                                        iiorjj = jj
                                        all_vehs_waypoints =calculate_one_veh(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map,iiorjj)
                                        dayu3_behaviors_flag[iiorjj][flag_index] = 0
                                elif i_symbol==["turn right"] and j_symbol==["turn around"]:
                                    if i_front[0] == j_front[0] and i_front[1] > j_front[1]:
                                        all_vehs_waypoints =calculate_turn_right_and_turn_around(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map)
                                        dayu3_behaviors_flag[ii][flag_index] = 0
                                        dayu3_behaviors_flag[jj][flag_index] = 0
                                    elif [i_front[0],j_front[0]] in [[1,3],[3,5],[5,7],[7,1]] and i_back[1]==j_back[1]:
                                        all_vehs_waypoints =calculate_turn_right_and_turn_around(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map)
                                        dayu3_behaviors_flag[ii][flag_index] = 0
                                        dayu3_behaviors_flag[jj][flag_index] = 0
                                    elif [i_front[0],j_front[0]] in [[1,3],[3,5],[5,7],[7,1]] and i_back[1]==2 and j_back[1]==1:
                                        all_vehs_waypoints =calculate_turn_right_and_turn_around(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map)
                                        dayu3_behaviors_flag[ii][flag_index] = 0
                                        dayu3_behaviors_flag[jj][flag_index] = 0
                                    else:
                                        iiorjj = ii
                                        all_vehs_waypoints =calculate_one_veh(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map,iiorjj)
                                        dayu3_behaviors_flag[iiorjj][flag_index] = 0
                                        iiorjj = jj
                                        all_vehs_waypoints =calculate_one_veh(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map,iiorjj)
                                        dayu3_behaviors_flag[iiorjj][flag_index] = 0
                                elif i_symbol==["turn right"] and j_symbol==["go across"]:
                                    if [i_front[0],j_front[0]] in [[1,7],[3,1],[5,3],[7,5]]:
                                        all_vehs_waypoints =calculate_turn_right_and_go_across(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map)
                                        dayu3_behaviors_flag[ii][flag_index] = 0
                                        dayu3_behaviors_flag[jj][flag_index] = 0
                                    elif i_front[0] == j_front[0] and i_front[1] == j_front[1]:
                                        all_vehs_waypoints =calculate_turn_right_and_go_across(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map)
                                        dayu3_behaviors_flag[ii][flag_index] = 0
                                        dayu3_behaviors_flag[jj][flag_index] = 0
                                    else:
                                        iiorjj = ii
                                        all_vehs_waypoints =calculate_one_veh(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map,iiorjj)
                                        dayu3_behaviors_flag[iiorjj][flag_index] = 0
                                        iiorjj = jj
                                        all_vehs_waypoints =calculate_one_veh(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map,iiorjj)
                                        dayu3_behaviors_flag[iiorjj][flag_index] = 0
                                elif i_symbol==["turn around"] and j_symbol==["turn around"]:
                                    if i_front[0] ==j_front[0] and i_front[1] == j_front[1]:
                                        all_vehs_waypoints =calculate_turn_around_and_turn_around(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map)
                                        dayu3_behaviors_flag[ii][flag_index] = 0
                                        dayu3_behaviors_flag[jj][flag_index] = 0
                                    elif i_front[0] ==j_front[0] and i_front[1] != i_back[1] and j_front[1] != j_back[1]:
                                        all_vehs_waypoints =calculate_turn_around_and_turn_around(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map)
                                        dayu3_behaviors_flag[ii][flag_index] = 0
                                        dayu3_behaviors_flag[jj][flag_index] = 0
                                    else:
                                        iiorjj = ii
                                        all_vehs_waypoints =calculate_one_veh(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map,iiorjj)
                                        dayu3_behaviors_flag[iiorjj][flag_index] = 0
                                        iiorjj = jj
                                        all_vehs_waypoints =calculate_one_veh(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map,iiorjj)
                                        dayu3_behaviors_flag[iiorjj][flag_index] = 0
                                elif i_symbol==["turn around"] and j_symbol==["go across"]:
                                    if [i_front[0],j_front[0]] in [[1,5],[3,7],[5,1],[7,3]]:
                                        all_vehs_waypoints =calculate_turn_around_and_go_across(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map)
                                        dayu3_behaviors_flag[ii][flag_index] = 0
                                        dayu3_behaviors_flag[jj][flag_index] = 0
                                    elif i_front[0] == j_front[0] and i_front[1] == 1 and j_front[1]==2:
                                        all_vehs_waypoints =calculate_turn_around_and_go_across(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map)
                                        dayu3_behaviors_flag[ii][flag_index] = 0
                                        dayu3_behaviors_flag[jj][flag_index] = 0
                                    else:
                                        iiorjj = ii
                                        all_vehs_waypoints =calculate_one_veh(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map,iiorjj)
                                        dayu3_behaviors_flag[iiorjj][flag_index] = 0
                                        iiorjj = jj
                                        all_vehs_waypoints =calculate_one_veh(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map,iiorjj)
                                        dayu3_behaviors_flag[iiorjj][flag_index] = 0
                                elif i_symbol==["go across"] and j_symbol==["go across"]:
                                    if [i_front[0],j_front[0]] in [[1,3],[1,7],[3,1],[3,5],[5,3],[5,7],[7,1],[7.5]]:
                                        all_vehs_waypoints =calculate_go_across_and_go_across(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map)
                                        dayu3_behaviors_flag[ii][flag_index] = 0
                                        dayu3_behaviors_flag[jj][flag_index] = 0
                                    else:
                                        iiorjj = ii
                                        all_vehs_waypoints =calculate_one_veh(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map,iiorjj)
                                        dayu3_behaviors_flag[iiorjj][flag_index] = 0
                                        iiorjj = jj
                                        all_vehs_waypoints =calculate_one_veh(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map,iiorjj)
                                        dayu3_behaviors_flag[iiorjj][flag_index] = 0
                                elif i_symbol==["change lane"] and j_symbol==["change lane"]:
                                    if i_front[0] == j_front[0] and i_front[1] == j_front[1]:
                                        all_vehs_waypoints =calculate_change_lane_and_change_lane(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map)
                                        dayu3_behaviors_flag[ii][flag_index] = 0
                                        dayu3_behaviors_flag[jj][flag_index] = 0
                                    elif i_front[0] == j_front[0] and i_front[1] != j_front[1]:
                                        all_vehs_waypoints =calculate_change_lane_and_change_lane(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map)
                                        dayu3_behaviors_flag[ii][flag_index] = 0
                                        dayu3_behaviors_flag[jj][flag_index] = 0
                                    else:
                                        iiorjj = ii
                                        all_vehs_waypoints =calculate_one_veh(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map,iiorjj)
                                        dayu3_behaviors_flag[iiorjj][flag_index] = 0
                                        iiorjj = jj
                                        all_vehs_waypoints =calculate_one_veh(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map,iiorjj)
                                        dayu3_behaviors_flag[iiorjj][flag_index] = 0
                                else:
                                    # calculate waypoints respectively
                                    iorj = ii
                                    all_vehs_waypoints =calculate_one_veh(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map,iorj)
                                    dayu3_behaviors_flag[ii][flag_index] = 0
                                    iorj = jj
                                    all_vehs_waypoints =calculate_one_veh(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map,iorj)
                                    dayu3_behaviors_flag[jj][flag_index] = 0
                                    # print(all_vehs_waypoints)
                                    # print(dayu3_behaviors_flag)
                            elif dayu3_behaviors_flag[ii][flag_index] == 1 and dayu3_behaviors_flag[jj][flag_index] == 0:
                                iorj = ii
                                all_vehs_waypoints =calculate_one_veh(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map,iorj)
                                dayu3_behaviors_flag[ii][flag_index] = 0
                            elif dayu3_behaviors_flag[ii][flag_index] == 0 and dayu3_behaviors_flag[jj][flag_index] == 1:
                                iorj = jj
                                all_vehs_waypoints =calculate_one_veh(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map,iorj)
                                dayu3_behaviors_flag[jj][flag_index] = 0
                                # print(all_vehs_waypoints)
                    elif flag_index >=len(dayu3_behaviors_flag[ii]) and flag_index <len(dayu3_behaviors_flag[jj]):
                        if dayu3_behaviors_flag[jj][flag_index] == 1:
                            ii = dayu3[i]
                            i_front = []
                            i_back = []
                            i_symbol = []
                            jj = dayu3[j]
                            j_front = all_vehs_symbol[dayu3[j]][zhi-1]
                            j_back = all_vehs_symbol[dayu3[j]][zhi+1]
                            j_symbol = all_vehs_symbol[dayu3[j]][zhi]
                            iiorjj = jj
                            all_vehs_waypoints =calculate_one_veh(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map,iiorjj)
                            dayu3_behaviors_flag[jj][flag_index] = 0
                    elif flag_index < len(dayu3_behaviors_flag[ii]) and flag_index >= len(dayu3_behaviors_flag[jj]):
                        if dayu3_behaviors_flag[ii][flag_index] == 1:
                            ii = dayu3[i]
                            i_front = all_vehs_symbol[dayu3[i]][zhi-1]
                            i_back = all_vehs_symbol[dayu3[i]][zhi+1]
                            i_symbol = all_vehs_symbol[dayu3[i]][zhi]
                            jj = dayu3[j]
                            j_front = []
                            j_back = []
                            j_symbol = []
                            iiorjj = ii
                            all_vehs_waypoints =calculate_one_veh(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map,iiorjj)
                            dayu3_behaviors_flag[ii][flag_index] = 0
                    else:
                        pass           
        zhi+=2     
        # print(zhi)        
    # print(all_vehs_waypoints)
    return all_vehs_waypoints

def reasonable():
    temp_behaviors = [["follow lane"],["follow lane"],["drive into"],["drive off"],["turn left"],["turn right"],["go across"],["drive into"],["change lane"],["follow lane"],["turn around"],["go across"],["drive into"],["stop"]]
    zhizhen = 0
    while(zhizhen >= 0) :
        # print("zhizhen",zhizhen)
        # print(temp_behaviors)
        if zhizhen < len(temp_behaviors)-1 and temp_behaviors[zhizhen][0] == "follow lane" :
            if temp_behaviors[zhizhen+1][0] in ["follow lane","drive into","drive off"]:
                temp_behaviors.pop(zhizhen+1)
            else:
                zhizhen += 1
        elif zhizhen < len(temp_behaviors)-1 and temp_behaviors[zhizhen][0] == "turn left":
            if temp_behaviors[zhizhen+1][0] in ["turn left","turn right","go across","drive into"]:
                temp_behaviors.pop(zhizhen+1)
            else:
                zhizhen += 1
        elif zhizhen < len(temp_behaviors)-1 and temp_behaviors[zhizhen][0] == "turn right":
            if temp_behaviors[zhizhen+1][0] in ["turn right","turn left","go across","drive into"]:
                temp_behaviors.pop(zhizhen+1)
            else:
                zhizhen += 1
        elif zhizhen < len(temp_behaviors)-1 and temp_behaviors[zhizhen][0] == "change lane":
            if temp_behaviors[zhizhen+1][0] in ["follow lane","change lane","drive into","drive off"]:
                temp_behaviors.pop(zhizhen+1)
            elif temp_behaviors[zhizhen+1][0] == "go across":
                for i in range(0,zhizhen):
                    if temp_behaviors[i][0] in ["turn left","turn right","go across"]:
                        temp_behaviors.pop(zhizhen+1)
                        break
            else:
                zhizhen += 1
        elif zhizhen < len(temp_behaviors)-1 and temp_behaviors[zhizhen][0] == "turn around":
            if temp_behaviors[zhizhen+1][0] in ["drive into"]:
                temp_behaviors.pop(zhizhen+1)
            else:
                zhizhen += 1
        elif zhizhen < len(temp_behaviors)-1 and temp_behaviors[zhizhen][0] == "go across":
            if temp_behaviors[zhizhen+1][0] in ["turn left","turn right","go across","drive into"]:
                temp_behaviors.pop(zhizhen+1)
            else:
                zhizhen += 1
        elif zhizhen < len(temp_behaviors)-1 and temp_behaviors[zhizhen][0] == "drive into":
            if temp_behaviors[zhizhen+1][0] == "drive into":
                temp_behaviors.pop(zhizhen+1)
            else:
                zhizhen += 1
        else:
            # print("lsm")
            zhizhen = -1

def start_svl_simulation_nooryes_apollo(all_vehs_waypoints,all_vehs_symbol,map,map_lon_lat,report,flag_apollo=False):
    
    for veh_id in all_vehs_waypoints.keys():
        removed_index = []
        if len(all_vehs_waypoints[veh_id]) <=2:
            # all_vehs_waypoints[veh_id].pop(0)
            removed_index.append(0)
        elif len(all_vehs_waypoints[veh_id]) > 2:
            for index in range(0,len(all_vehs_waypoints[veh_id])):
                # print("idnex",index)
                if all_vehs_waypoints[veh_id][index] == ["start"]:
                    # all_vehs_waypoints[veh_id].pop(index)
                    removed_index.append(index)
                elif all_vehs_waypoints[veh_id][index] in [["follow lane"],["turn left"],["turn right"],["turn around"],["change lane"],["go across"],["drive into"],["drive off"],["retrograde"],["stop"]]:
                    # all_vehs_waypoints[veh_id].pop(index)
                    removed_index.append(index)
                    if index+1<len(all_vehs_waypoints[veh_id]):
                        if all_vehs_waypoints[veh_id][index-1] == all_vehs_waypoints[veh_id][index+1]:
                            # all_vehs_waypoints[veh_id].pop(index+1)
                            removed_index.append(index+1)
                        else:
                            pass
                else:
                    pass
        # print("removed_index",removed_index)
        for i in range(0,len(removed_index)):
            all_vehs_waypoints[veh_id].pop(removed_index[len(removed_index)-1-i])
    print(all_vehs_waypoints)

    
    if flag_apollo == False:
        ####
        env = Env()
        SIMULATOR_HOST = env.str("LGSVL__SIMULATOR_HOST", "127.0.0.1")
        SIMULATOR_PORT = env.int("LGSVL__SIMULATOR_PORT", 8181)

        vehicle_conf = env.str("LGSVL__VEHICLE_0", lgsvl.wise.DefaultAssets.ego_lincoln2017mkz_apollo6_modular)
        scene_name = env.str("LGSVL__MAP", "12da60a7-2fc9-474d-a62a-5cc08cb97fe8")
        sim = lgsvl.Simulator(SIMULATOR_HOST, SIMULATOR_PORT)
        if sim.current_scene == scene_name:
            sim.reset()
        else:
            sim.load(scene_name)
        sim.set_date_time(datetime(2022, 6, 22, 11, 0, 0, 0), True)
        #####
        npc_list = {}
        npc_waypoints = {}
        for veh_id in all_vehs_waypoints.keys():
            npc_waypoints[veh_id] = []
            if len(all_vehs_waypoints[veh_id]) ==1:
                temp_pos = lgsvl.Vector(all_vehs_waypoints[veh_id][0][0],all_vehs_waypoints[veh_id][0][1],all_vehs_waypoints[veh_id][0][2])
                state = lgsvl.AgentState()
                state.transform = Transform(position=temp_pos,rotation=sim.map_point_on_lane(temp_pos).rotation)
                npc_list[veh_id] = sim.add_agent("SUV", lgsvl.AgentType.NPC,state)
            elif len(all_vehs_waypoints[veh_id]) > 2:
                if all_vehs_symbol[veh_id][1] == ["retrograde"]:
                    road = all_vehs_symbol[veh_id][0][0]
                    if road+1 ==9:
                        road = 1
                        temp_rotation = lgsvl.Vector(map[road][6][0],map[road][6][1],map[road][6][2])
                    else:
                        temp_rotation = lgsvl.Vector(map[road+1][6][0],map[road+1][6][1],map[road+1][6][2])
                    temp_pos = lgsvl.Vector(all_vehs_waypoints[veh_id][0][0],all_vehs_waypoints[veh_id][0][1],all_vehs_waypoints[veh_id][0][2])
                    state = lgsvl.AgentState()
                    state.transform = Transform(position=temp_pos,rotation=temp_rotation)
                    npc_list[veh_id] = sim.add_agent("SUV", lgsvl.AgentType.NPC,state)
                    for i in range(0,len(all_vehs_waypoints[veh_id])):
                        if i == 0:
                            npc_waypoints[veh_id].append(lgsvl.DriveWaypoint(position=lgsvl.Vector(all_vehs_waypoints[veh_id][i][0],all_vehs_waypoints[veh_id][i][1],all_vehs_waypoints[veh_id][i][2]),speed=10,angle=temp_rotation))
                        elif i>0:
                            former_pos = [all_vehs_waypoints[veh_id][i-1][0],all_vehs_waypoints[veh_id][i-1][1],all_vehs_waypoints[veh_id][i-1][2]]
                            curr_pos = [all_vehs_waypoints[veh_id][i][0],all_vehs_waypoints[veh_id][i][1],all_vehs_waypoints[veh_id][i][2]]
                            dis = math.sqrt((former_pos[0]-curr_pos[0])**2+(former_pos[1]-curr_pos[1])**2+(former_pos[2]-curr_pos[2])**2)
                            speed = dis/1
                            temp_pos = lgsvl.Vector(curr_pos[0],curr_pos[1],curr_pos[2])
                            temp_rotation = [0,math.degrees(math.atan2(curr_pos[0]-former_pos[0],curr_pos[2]-former_pos[2])),0,0]
                            temp_rotation = lgsvl.Vector(temp_rotation[0],temp_rotation[1],temp_rotation[2])
                            npc_waypoints[veh_id].append(lgsvl.DriveWaypoint(position=temp_pos,speed=speed,angle=temp_rotation))
                else:
                    road = all_vehs_symbol[veh_id][0][0]
                    temp_rotation = lgsvl.Vector(map[road][6][0],map[road][6][1],map[road][6][2])
                    temp_pos = lgsvl.Vector(all_vehs_waypoints[veh_id][0][0],all_vehs_waypoints[veh_id][0][1],all_vehs_waypoints[veh_id][0][2])
                    state = lgsvl.AgentState()
                    state.transform = Transform(position=temp_pos,rotation=temp_rotation)
                    npc_list[veh_id] = sim.add_agent("SUV", lgsvl.AgentType.NPC,state)
                    for i in range (0,len(all_vehs_waypoints[veh_id])):
                        if i == 0:
                            npc_waypoints[veh_id].append(lgsvl.DriveWaypoint(position=lgsvl.Vector(all_vehs_waypoints[veh_id][i][0],all_vehs_waypoints[veh_id][i][1],all_vehs_waypoints[veh_id][i][2]),speed=10,angle=temp_rotation))
                        elif i>0:
                            former_pos = [all_vehs_waypoints[veh_id][i-1][0],all_vehs_waypoints[veh_id][i-1][1],all_vehs_waypoints[veh_id][i-1][2]]
                            curr_pos = [all_vehs_waypoints[veh_id][i][0],all_vehs_waypoints[veh_id][i][1],all_vehs_waypoints[veh_id][i][2]]
                            dis = math.sqrt((former_pos[0]-curr_pos[0])**2+(former_pos[1]-curr_pos[1])**2+(former_pos[2]-curr_pos[2])**2)
                            speed = dis/1
                            temp_pos = lgsvl.Vector(curr_pos[0],curr_pos[1],curr_pos[2])
                            temp_rotation = [0,math.degrees(math.atan2(curr_pos[0]-former_pos[0],curr_pos[2]-former_pos[2])),0,0]
                            temp_rotation = lgsvl.Vector(temp_rotation[0],temp_rotation[1],temp_rotation[2])
                            npc_waypoints[veh_id].append(lgsvl.DriveWaypoint(position=temp_pos,speed=speed,angle=temp_rotation))

        for veh_id in npc_waypoints.keys():
            for drivewp in npc_waypoints[veh_id]:
                print(drivewp.position,drivewp.speed,drivewp.angle)

        for veh_id in npc_list.keys():
            npc_list[veh_id].follow(npc_waypoints[veh_id])

        cameraPosition = Vector(53, 100, -44)
        cameraRotation = Vector(90, 0, 0)
        camera = Transform(cameraPosition, cameraRotation)
        sim.set_sim_camera(camera)
        sim.run()
    else:
        choosen_vehicle = ""
        striker = report['striker']
        at_fault_vehicle = report["At-Fault-Vehicle"]
        if striker != "":
            choosen_vehicle = striker
        elif at_fault_vehicle != "":
            choosen_vehicle = at_fault_vehicle
        else:
            indexx = random.randint(0,len(all_vehs_waypoints.keys())-1)
            choosen_vehicle = list(all_vehs_waypoints.keys())[indexx]
        while len(all_vehs_symbol[choosen_vehicle])==1:
            print("choosen_vehicle stays still,so change anthor vehicle")
            indexx = random.randint(0,len(all_vehs_waypoints.keys())-1)
            choosen_vehicle = list(all_vehs_waypoints.keys())[indexx]
        print("choosen_vehicle",choosen_vehicle)
        env = Env()
        SIMULATOR_HOST = env.str("LGSVL__SIMULATOR_HOST", "127.0.0.1")
        SIMULATOR_PORT = env.int("LGSVL__SIMULATOR_PORT", 8181)
        BRIDGE_HOST = env.str("LGSVL__AUTOPILOT_0_HOST", "127.0.0.1")
        BRIDGE_PORT = env.int("LGSVL__AUTOPILOT_0_PORT", 9090)
        LGSVL__AUTOPILOT_HD_MAP = env.str("LGSVL__AUTOPILOT_HD_MAP", "san_francisco")
        LGSVL__AUTOPILOT_0_VEHICLE_CONFIG = env.str("LGSVL__AUTOPILOT_0_VEHICLE_CONFIG", 'Lincoln2017MKZ_LGSVL')
        vehicle_conf = env.str("LGSVL__VEHICLE_0", lgsvl.wise.DefaultAssets.ego_lincoln2017mkz_apollo6_modular)
        scene_name = env.str("LGSVL__MAP", "12da60a7-2fc9-474d-a62a-5cc08cb97fe8")
        sim = lgsvl.Simulator(SIMULATOR_HOST, SIMULATOR_PORT)
        if sim.current_scene == scene_name:
            sim.reset()
        else:
            sim.load(scene_name)
        sim.set_date_time(datetime(2022, 6, 22, 11, 0, 0, 0), True)
        npc_list = {}
        npc_waypoints = {}

        ##############
        for veh_id in all_vehs_waypoints.keys()-{choosen_vehicle}:
            print(veh_id)
            npc_waypoints[veh_id] = []
            if len(all_vehs_waypoints[veh_id]) ==1:
                temp_pos = lgsvl.Vector(all_vehs_waypoints[veh_id][0][0],all_vehs_waypoints[veh_id][0][1],all_vehs_waypoints[veh_id][0][2])
                state = lgsvl.AgentState()
                state.transform = Transform(position=temp_pos,rotation=sim.map_point_on_lane(temp_pos).rotation)
                npc_list[veh_id] = sim.add_agent("SUV", lgsvl.AgentType.NPC,state)
                npc_waypoints[veh_id].append(lgsvl.DriveWaypoint(position=temp_pos,speed=0,angle=state.transform.rotation))
            elif len(all_vehs_waypoints[veh_id]) > 2:
                if all_vehs_symbol[veh_id][1] == ["retrograde"]:
                    road = all_vehs_symbol[veh_id][0][0]
                    if road+1 ==9:
                        road = 1
                        temp_rotation = lgsvl.Vector(map[road][6][0],map[road][6][1],map[road][6][2])
                    else:
                        temp_rotation = lgsvl.Vector(map[road+1][6][0],map[road+1][6][1],map[road+1][6][2])
                    temp_pos = lgsvl.Vector(all_vehs_waypoints[veh_id][0][0],all_vehs_waypoints[veh_id][0][1],all_vehs_waypoints[veh_id][0][2])
                    state = lgsvl.AgentState()
                    state.transform = Transform(position=temp_pos,rotation=temp_rotation)
                    npc_list[veh_id] = sim.add_agent("SUV", lgsvl.AgentType.NPC,state)
                    for i in range(0,len(all_vehs_waypoints[veh_id])):
                        if i == 0:
                            npc_waypoints[veh_id].append(lgsvl.DriveWaypoint(position=lgsvl.Vector(all_vehs_waypoints[veh_id][i][0],all_vehs_waypoints[veh_id][i][1],all_vehs_waypoints[veh_id][i][2]),speed=2.5,angle=temp_rotation))
                        elif i>0:
                            former_pos = [all_vehs_waypoints[veh_id][i-1][0],all_vehs_waypoints[veh_id][i-1][1],all_vehs_waypoints[veh_id][i-1][2]]
                            curr_pos = [all_vehs_waypoints[veh_id][i][0],all_vehs_waypoints[veh_id][i][1],all_vehs_waypoints[veh_id][i][2]]
                            dis = math.sqrt((former_pos[0]-curr_pos[0])**2+(former_pos[1]-curr_pos[1])**2+(former_pos[2]-curr_pos[2])**2)
                            speed = dis/4
                            temp_pos = lgsvl.Vector(curr_pos[0],curr_pos[1],curr_pos[2])
                            temp_rotation = [0,math.degrees(math.atan2(curr_pos[0]-former_pos[0],curr_pos[2]-former_pos[2])),0,0]
                            temp_rotation = lgsvl.Vector(temp_rotation[0],temp_rotation[1],temp_rotation[2])
                            npc_waypoints[veh_id].append(lgsvl.DriveWaypoint(position=temp_pos,speed=speed,angle=temp_rotation))
                else:
                    road = all_vehs_symbol[veh_id][0][0]
                    temp_rotation = lgsvl.Vector(map[road][6][0],map[road][6][1],map[road][6][2])
                    temp_pos = lgsvl.Vector(all_vehs_waypoints[veh_id][0][0],all_vehs_waypoints[veh_id][0][1],all_vehs_waypoints[veh_id][0][2])
                    state = lgsvl.AgentState()
                    state.transform = Transform(position=temp_pos,rotation=temp_rotation)
                    npc_list[veh_id] = sim.add_agent("SUV", lgsvl.AgentType.NPC,state)
                    for i in range (0,len(all_vehs_waypoints[veh_id])):
                        if i == 0:
                            npc_waypoints[veh_id].append(lgsvl.DriveWaypoint(position=lgsvl.Vector(all_vehs_waypoints[veh_id][i][0],all_vehs_waypoints[veh_id][i][1],all_vehs_waypoints[veh_id][i][2]),speed=2.5,angle=temp_rotation))
                        elif i>0:
                            former_pos = [all_vehs_waypoints[veh_id][i-1][0],all_vehs_waypoints[veh_id][i-1][1],all_vehs_waypoints[veh_id][i-1][2]]
                            curr_pos = [all_vehs_waypoints[veh_id][i][0],all_vehs_waypoints[veh_id][i][1],all_vehs_waypoints[veh_id][i][2]]
                            dis = math.sqrt((former_pos[0]-curr_pos[0])**2+(former_pos[1]-curr_pos[1])**2+(former_pos[2]-curr_pos[2])**2)
                            speed = dis/4
                            temp_pos = lgsvl.Vector(curr_pos[0],curr_pos[1],curr_pos[2])
                            temp_rotation = [0,math.degrees(math.atan2(curr_pos[0]-former_pos[0],curr_pos[2]-former_pos[2])),0,0]
                            temp_rotation = lgsvl.Vector(temp_rotation[0],temp_rotation[1],temp_rotation[2])
                            npc_waypoints[veh_id].append(lgsvl.DriveWaypoint(position=temp_pos,speed=speed,angle=temp_rotation))
        for veh_id in npc_waypoints.keys():
            for drivewp in npc_waypoints[veh_id]:
                print(drivewp.position,drivewp.speed,drivewp.angle)

        for veh_id in npc_list.keys():
            # print(veh_id)
            npc_list[veh_id].follow(npc_waypoints[veh_id])

        ##############
        if all_vehs_symbol[choosen_vehicle][1] == ["retrograde"]:
            road = all_vehs_symbol[choosen_vehicle][0][0]
            if road+1 ==9:
                road = 1
                temp_rotation = lgsvl.Vector(map[road][6][0],map[road][6][1],map[road][6][2])
            else:
                temp_rotation = lgsvl.Vector(map[road+1][6][0],map[road+1][6][1],map[road+1][6][2])
            temp_pos = lgsvl.Vector(all_vehs_waypoints[choosen_vehicle][0][0],all_vehs_waypoints[choosen_vehicle][0][1],all_vehs_waypoints[choosen_vehicle][0][2])
            ego_state = lgsvl.AgentState()
            ego_state.transform = Transform(position=temp_pos,rotation=temp_rotation)
            ego = sim.add_agent(vehicle_conf, lgsvl.AgentType.EGO, ego_state)
            ego.connect_bridge(BRIDGE_HOST, BRIDGE_PORT)

            dv = lgsvl.dreamview.Connection(sim, ego, BRIDGE_HOST)
            dv.set_hd_map(LGSVL__AUTOPILOT_HD_MAP)
            dv.set_vehicle(LGSVL__AUTOPILOT_0_VEHICLE_CONFIG)

            default_modules = [
                'Localization',
                'Transform',
                'Routing',
                'Prediction',
                'Planning',
                'Control',
                'Recorder',
                'Perception'
                
            ]

            dv.disable_apollo()
            # time.sleep(5)
            if all_vehs_symbol[choosen_vehicle][-1][2] == "e":
                final_pos_lon_lat = map_lon_lat[all_vehs_symbol[choosen_vehicle][-1][0]][all_vehs_symbol[choosen_vehicle][-1][1]][1]
            else:
                final_pos_lon_lat = map_lon_lat[all_vehs_symbol[choosen_vehicle][-1][0]][all_vehs_symbol[choosen_vehicle][-1][1]][0]
            dv.setup_apollo(final_pos_lon_lat[0], final_pos_lon_lat[1], default_modules)
            dv.set_destination(final_pos_lon_lat[0],final_pos_lon_lat[1],coord_type=CoordType.Northing)
            cameraPosition = Vector(53, 100, -44)
            cameraRotation = Vector(90, 0, 0)
            camera = Transform(cameraPosition, cameraRotation)
            sim.set_sim_camera(camera)
            sim.run(1500)
        else:
            road = all_vehs_symbol[choosen_vehicle][0][0]
            temp_rotation = lgsvl.Vector(map[road][6][0],map[road][6][1],map[road][6][2])
            temp_pos = lgsvl.Vector(all_vehs_waypoints[choosen_vehicle][0][0],all_vehs_waypoints[choosen_vehicle][0][1],all_vehs_waypoints[choosen_vehicle][0][2])
            ego_state = lgsvl.AgentState()
            ego_state.transform = Transform(position=temp_pos,rotation=temp_rotation)
            ego = sim.add_agent(vehicle_conf, lgsvl.AgentType.EGO, ego_state)
            ego.connect_bridge(BRIDGE_HOST, BRIDGE_PORT)

            dv = lgsvl.dreamview.Connection(sim, ego, BRIDGE_HOST)
            dv.set_hd_map(LGSVL__AUTOPILOT_HD_MAP)
            dv.set_vehicle(LGSVL__AUTOPILOT_0_VEHICLE_CONFIG)

            default_modules = [
                'Localization',
                'Transform',
                'Routing',
                'Prediction',
                'Planning',
                'Control',
                'Recorder',
                'Perception'
                
            ]

            dv.disable_apollo()
            # time.sleep(5)
            if all_vehs_symbol[choosen_vehicle][-1][2] == "e":
                final_pos_lon_lat = map_lon_lat[all_vehs_symbol[choosen_vehicle][-1][0]][all_vehs_symbol[choosen_vehicle][-1][1]][1]
            else:
                final_pos_lon_lat = map_lon_lat[all_vehs_symbol[choosen_vehicle][-1][0]][all_vehs_symbol[choosen_vehicle][-1][1]][0]
            dv.setup_apollo(final_pos_lon_lat[0], final_pos_lon_lat[1], default_modules)
            dv.set_destination(final_pos_lon_lat[0],final_pos_lon_lat[1],coord_type=CoordType.Northing)
            cameraPosition = Vector(53, 100, -44)
            cameraRotation = Vector(90, 0, 0)
            camera = Transform(cameraPosition, cameraRotation)
            sim.set_sim_camera(camera)
            sim.run(1500)
        pass
    
if __name__ == '__main__':
    directory_path = "/home/lsm/SFTSG_NME/src/approach/combined_reports_strict_rule/two_merged"
    json_files = glob.glob(os.path.join(directory_path, '*.json'))
    for file_path in json_files:
        # print(file_path)
        # bug::211.json 's vehs'fist coordination is overlapped
        # file_path = "/home/lsm/SFTSG_NME/src/approach/combined_reports_strict_rule/two_merged/2_211.json"  
        # file_path = "/home/lsm/SFTSG_NME/src/approach/combined_reports_strict_rule/two_merged/2_40.json"
        file_path = "/home/lsm/SFTSG_NME/src/approach/combined_reports_strict_rule/n_merged/3_12.json"
        simulate_report(file_path,True)
        # input("Press any key to continue...")
        break

# /home/lsm/SFTSG_NME/src/approach/combined_reports_strict_rule/two_merged/2_223.json
# {'V1': [[5, 1, 's'], ['follow lane'], [5, 1, 'e'], ['go across'], [8, 1, 's']], 'V2': [[7, 1, 's'], ['follow lane'], [7, 1, 'e'], ['go across'], [2, 1, 's']], 'V3': [[7, 1, 's'], ['follow lane'], [7, 1, 'e'], ['turn right'], [8, 1, 's']]}
# {'V1': [['start'], [7.30000305175781, 10, -17.8000011444092], [12.8, 10, -22.9], [18.4, 10, -28.0], [23.9, 10, -33.1], [29.5000076293945, 10, -38.2499961853027], ['go across']], 'V2': [['start'], [-14.9999942779541, 10, -73.0000076293945], [-7.2, 10, -64.7], [0.5, 10, -56.5], [8.2, 10, -48.2], [16.0000057220459, 10, -40], ['follow lane'], [16.0000057220459, 10, -40], [21.5, 10, -34.5], [27.1, 10, -29.0], [32.7, 10, -23.5], [38.3000030517578, 10, -18], ['go across']], 'V3': [['start'], [-12.945968145226455, 10.0, -70.81346317613682], [-5.1, 10.0, -62.5], [2.5, 10.0, -54.3], [10.3, 10.0, -46.0], [18.054031854773545, 10.0, -37.813455546742325], ['follow lane'], [16.0000057220459, 10, -40], [19.3, 10, -39.5], [22.7, 10, -39.1], [26.1, 10, -38.6], [29.5000076293945, 10, -38.2499961853027], ['turn right']]}

