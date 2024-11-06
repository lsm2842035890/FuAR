import random
import z3
from z3 import Or
from utils_lsm import *
# [[], [[69.0000152587891, 10, -59.9999923706055], [37.700008392334, 10, -30]], [[66.3000183105469, 10, -64.0000076293945], [35.5000076293945, 10, -33.9999961853027]], [90.0000152587891, 10, -75.9999923706055], [67.0000152587891, 10, -54], [-0.7219403581398334, 0.0, 0.6919552870590188], [-1.72745376403327e-06, 313.78515625, 5.08888754974322e-14], [], []]
# [[], [[38.3000030517578, 10, -18], [75.3000106811523, 10, 22.0000076293945]], [[34.8000030517578, 10, -16], [71.8000030517578, 10, 24.0000114440918]], [38.0000076293945, 10, -22.9999961853027], [64.0000076293945, 10, 3.00000762939453], [0.6790419872766388, 0.0, 0.7340994343516369], [0, 43.6904029846191, 0], [], []]
# [[], [[65.5, 10, 28.7000122070313], [28.5000038146973, 10, -11.4999961853027]], [[68.5, 10, 26.2000122070313], [31.5000057220459, 10, -13.4999961853027]], [64, 10, 30.0000114440918], [28.0000038146973, 10, -3.99999618530273], [-0.6772150065758247, 0.0, -0.7357851825556871], [0, 223.804779052734, 0], [], []]
# [[], [[14.0000028610229, 10, -7.99999809265137], [-22.0000057220459, 10, 26.0000019073486]], [[12.0000028610229, 10, -9.99999809265137], [-24.0000057220459, 10, 21.9999961853027]], [22.0000038146973, 10, -7.99999809265137], [-6.00000286102295, 10, 17], [-0.7270132342681785, 0.0, 0.6866234464383826], [0, 313.699340820313, 0], [], []]
# [[], [[-28.7000064849854, 10, 16.1999969482422], [7.30000305175781, 10, -17.8000011444092]], [[-26.2000064849854, 10, 18.6999969482422], [9.80000305175781, 10, -15.3000011444092]], [-52.0000114440918, 10, 34.9999923706055], [-0.999998211860657, 10, -16.0000019073486], [0.7270132625758454, 0.0, -0.6866234164655504], [0, 133.722274780273, 0], [], []]
# [[], [[6.30000495910645, 10, -33.3000030517578], [-24.6999969482422, 10, -66.3000183105469]], [[8.80000495910645, 10, -35.8000030517578], [-22.1999950408936, 10, -68.8000106811523]], [7.00000476837158, 10, -27.0000038146973], [-24.9999961853027, 10, -60.0000114440918], [-0.684675315866033, 0.0, -0.728848209055732], [-6.68073710130557e-07, 223.667877197266, 0], [], []]
# [[], [[-14.9999942779541, 10, -73.0000076293945], [16.0000057220459, 10, -40]], [[-18.9999942779541, 10, -71.0000076293945], [12.0000057220459, 10, -38]], [-29.9999961853027, 10, -90.0000152587891], [12.0000095367432, 10, -52.0000076293945], [0.6846753775758818, 0.0, 0.7288481510858923], [0, 43.6501121520996, 0], [], []]
# [[], [[29.5000076293945, 10, -38.2499961853027], [60.5000152587891, 10, -68.3000106811523]], [[32.5000076293945, 10, -36.7499961853027], [63.5000152587891, 10, -66.8000106811523]], [25.0000076293945, 10, -39.9999961853027], [68.0000152587891, 10, -77.9999923706055], [0.7180230709535472, 0.0, -0.6960193025904076], [0, 133.784973144531, 0], [], []]

def calculate_follow_lane_and_follow_lane(all_vehs_waypoints,ii_id,jj_id,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map):
    if len(all_vehs_waypoints[ii_id]) == 0 and len(all_vehs_waypoints[jj_id]) == 0:
        all_vehs_waypoints[ii_id].append(["start"])
        all_vehs_waypoints[jj_id].append(["start"])
    elif len(all_vehs_waypoints[ii_id]) == 0 and len(all_vehs_waypoints[jj_id]) > 0:
        all_vehs_waypoints[ii_id].append(["start"])
    elif len(all_vehs_waypoints[ii_id]) > 0 and len(all_vehs_waypoints[jj_id]) == 0:
        all_vehs_waypoints[jj_id].append(["start"])
    elif len(all_vehs_waypoints[ii_id]) > 0 and len(all_vehs_waypoints[jj_id]) > 0:
        pass
    ii_initial_position = map[i_front[0]][i_front[1]][0]
    jj_initial_position = [ii_initial_position[0]+3*unitnormalize(map[i_front[0]][5])[0],
                            ii_initial_position[1]+3*unitnormalize(map[i_front[0]][5])[1],
                            ii_initial_position[2]+3*unitnormalize(map[i_front[0]][5])[2]]
    ii_final_position = map[i_back[0]][i_back[1]][1]
    jj_final_position = [ii_final_position[0]+3*unitnormalize(map[i_back[0]][5])[0],
                            ii_final_position[1]+3*unitnormalize(map[i_back[0]][5])[1],
                            ii_final_position[2]+3*unitnormalize(map[i_back[0]][5])[2]]
    all_vehs_waypoints[ii_id].append(ii_initial_position)
    all_vehs_waypoints[jj_id].append(jj_initial_position)
    ii_coords = [(z3.Real(f"{ii_id}_x{i}"), z3.Real(f"{ii_id}_y{i}"), z3.Real(f"{ii_id}_z{i}")) for i in range(1, 4)]
    jj_coords = [(z3.Real(f"{jj_id}_x{i}"), z3.Real(f"{jj_id}_y{i}"), z3.Real(f"{jj_id}_z{i}")) for i in range(1, 4)]
    # print(ii_coords)
    exp = [z3.And(ii_coords[1-1][0]-ii_initial_position[0]==ii_coords[2-1][0]-ii_coords[1-1][0],ii_coords[2-1][0]-ii_coords[1-1][0]==ii_coords[3-1][0]-ii_coords[2-1][0],ii_coords[3-1][0]-ii_coords[2-1][0]==ii_final_position[0]-ii_coords[3-1][0])]+\
            [z3.And(ii_coords[1-1][2]-ii_initial_position[2]==ii_coords[2-1][2]-ii_coords[1-1][2],ii_coords[2-1][2]-ii_coords[1-1][2]==ii_coords[3-1][2]-ii_coords[2-1][2],ii_coords[3-1][2]-ii_coords[2-1][2]==ii_final_position[2]-ii_coords[3-1][2])]+\
            [z3.And(jj_coords[1-1][0]-jj_initial_position[0]==jj_coords[2-1][0]-jj_coords[1-1][0],jj_coords[2-1][0]-jj_coords[1-1][0]==jj_coords[3-1][0]-jj_coords[2-1][0],jj_coords[3-1][0]-jj_coords[2-1][0]==jj_final_position[0]-jj_coords[3-1][0])]+\
            [z3.And(jj_coords[1-1][2]-jj_initial_position[2]==jj_coords[2-1][2]-jj_coords[1-1][2],jj_coords[2-1][2]-jj_coords[1-1][2]==jj_coords[3-1][2]-jj_coords[2-1][2],jj_coords[3-1][2]-jj_coords[2-1][2]==jj_final_position[2]-jj_coords[3-1][2])]
        #   [z3.And(ii_coords[1-1][0]!=jj_coords[1-1][0],ii_coords[3-1][0]!=jj_coords[3-1][0],ii_coords[1-1][2]!=jj_coords[1-1][2],ii_coords[3-1][2]!=jj_coords[3-1][2])]+\
        #   [z3.And(ii_coords[2-1][0]==jj_coords[2-1][0],ii_coords[2-1][2]==jj_coords[2-1][2])]
    solver = z3.Solver()
    solver.add(exp)
    z3.set_option(rational_to_decimal=True)
    z3.set_option(precision=1)
    i = 0
    while solver.check() == z3.sat and i < 10000:
        i += 1
        model = solver.model()
        for i in range(1, 4):
            all_vehs_waypoints[ii_id].append([toNum(model.evaluate(ii_coords[i-1][0])),ii_initial_position[1],toNum(model.evaluate(ii_coords[i-1][2]))])
            # print(toNum(model.evaluate(ii_coords[i-1][0])),toNum(model.evaluate(ii_coords[i-1][2])))
        for i in range(1, 4):
            all_vehs_waypoints[jj_id].append([toNum(model.evaluate(jj_coords[i-1][0])),jj_initial_position[1],toNum(model.evaluate(jj_coords[i-1][2]))])
            # print(toNum(model.evaluate(jj_coords[i-1][0])),toNum(model.evaluate(jj_coords[i-1][2])))
        break
    all_vehs_waypoints[ii_id].append(ii_final_position)
    all_vehs_waypoints[jj_id].append(jj_final_position)
    all_vehs_waypoints[ii_id].append(["follow lane"])
    all_vehs_waypoints[jj_id].append(["follow lane"])
    return all_vehs_waypoints

def calculate_follow_lane_and_turn_left(all_vehs_waypoints,ii_id,jj_id,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map):
    # consider whether to generate this event
    # if len(all_vehs_waypoints[ii_id]) == 0 and len(all_vehs_waypoints[jj_id]) == 0:
    #     all_vehs_waypoints[ii_id].append(["start"])
    #     all_vehs_waypoints[ii_id].append(["start"])
    # elif len(all_vehs_waypoints[ii_id]) == 0 and len(all_vehs_waypoints[jj_id]) > 0:
    #     all_vehs_waypoints[ii_id].append(["start"])
    # elif len(all_vehs_waypoints[ii_id]) > 0 and len(all_vehs_waypoints[jj_id]) == 0:
    #     all_vehs_waypoints[ii_id].append(["start"])
    # elif len(all_vehs_waypoints[ii_id]) > 0 and len(all_vehs_waypoints[jj_id]) > 0:
    #     pass
    # ii_initial_position = map[i_front[0]][i_front[1]][0]
    # jj_initial_position = map[j_front[0]][j_front[1]][1]
    # ii_final_position = map[i_back[0]][i_back[1]][1]
    # jj_final_position = map[j_back[0]][j_back[1]][0]
    # all_vehs_waypoints[ii_id].append(ii_initial_position)
    # all_vehs_waypoints[jj_id].append(jj_initial_position)
    pass


def calculate_follow_lane_and_turn_right(all_vehs_waypoints,ii_id,jj_id,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map):
    pass

def calculate_follow_lane_and_turn_around(all_vehs_waypoints,ii_id,jj_id,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map):
    pass

def calculate_follow_lane_and_change_lane(all_vehs_waypoints,ii_id,jj_id,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map):
    if len(all_vehs_waypoints[ii_id]) == 0 and len(all_vehs_waypoints[jj_id]) == 0:
        all_vehs_waypoints[ii_id].append(["start"])
        all_vehs_waypoints[jj_id].append(["start"])
    elif len(all_vehs_waypoints[ii_id]) == 0 and len(all_vehs_waypoints[jj_id]) > 0:
        all_vehs_waypoints[ii_id].append(["start"])
    elif len(all_vehs_waypoints[ii_id]) > 0 and len(all_vehs_waypoints[jj_id]) == 0:
        all_vehs_waypoints[jj_id].append(["start"])
    elif len(all_vehs_waypoints[ii_id]) > 0 and len(all_vehs_waypoints[jj_id]) > 0:
        pass
    ii_initial_position = map[i_front[0]][i_front[1]][0]
    jj_initial_position = map[j_front[0]][j_front[1]][0]
    ii_final_position = map[i_back[0]][i_back[1]][1]
    jj_final_position = map[j_back[0]][j_back[1]][1]
    jj_middle_position = [(jj_final_position[0]+jj_initial_position[0])/2, (jj_final_position[1]+jj_initial_position[1])/2, (jj_final_position[2]+jj_initial_position[2])/2]
    all_vehs_waypoints[ii_id].append(ii_initial_position)
    all_vehs_waypoints[jj_id].append(jj_initial_position)
    ii_coords = [(z3.Real(f"{ii_id}_x{i}"), z3.Real(f"{ii_id}_y{i}"), z3.Real(f"{ii_id}_z{i}")) for i in range(1, 4)]
    exp = [z3.And(ii_coords[1-1][0]-ii_initial_position[0]==ii_coords[2-1][0]-ii_coords[1-1][0],ii_coords[2-1][0]-ii_coords[1-1][0]==ii_coords[3-1][0]-ii_coords[2-1][0],ii_coords[3-1][0]-ii_coords[2-1][0]==ii_final_position[0]-ii_coords[3-1][0])]+\
          [z3.And(ii_coords[1-1][2]-ii_initial_position[2]==ii_coords[2-1][2]-ii_coords[1-1][2],ii_coords[2-1][2]-ii_coords[1-1][2]==ii_coords[3-1][2]-ii_coords[2-1][2],ii_coords[3-1][2]-ii_coords[2-1][2]==ii_final_position[2]-ii_coords[3-1][2])]
    solver = z3.Solver()
    solver.add(exp)
    z3.set_option(rational_to_decimal=True)
    z3.set_option(precision=1)
    i = 0
    while solver.check() == z3.sat and i < 10000:
        i += 1
        model = solver.model()
        for i in range(1, 4):
            all_vehs_waypoints[ii_id].append([toNum(model.evaluate(ii_coords[i-1][0])),ii_initial_position[1],toNum(model.evaluate(ii_coords[i-1][2]))])
        break
    all_vehs_waypoints[ii_id].append(ii_final_position)
    all_vehs_waypoints[ii_id].append(["follow lane"])

    jj_coords = [(z3.Real(f"{jj_id}_x{i}"), z3.Real(f"{jj_id}_y{i}"), z3.Real(f"{jj_id}_z{i}")) for i in range(1, 4)]
    exp = [z3.And(jj_coords[1-1][0]-jj_initial_position[0]==jj_coords[2-1][0]-jj_coords[1-1][0],jj_coords[2-1][0]-jj_coords[1-1][0]==jj_coords[3-1][0]-jj_coords[2-1][0],jj_coords[3-1][0]-jj_coords[2-1][0]==jj_middle_position[0]-jj_coords[3-1][0])]+\
          [z3.And(jj_coords[1-1][2]-jj_initial_position[2]==jj_coords[2-1][2]-jj_coords[1-1][2],jj_coords[2-1][2]-jj_coords[1-1][2]==jj_coords[3-1][2]-jj_coords[2-1][2],jj_coords[3-1][2]-jj_coords[2-1][2]==jj_middle_position[2]-jj_coords[3-1][2])]
    solver = z3.Solver()
    solver.add(exp)
    z3.set_option(rational_to_decimal=True)
    z3.set_option(precision=1)
    i = 0
    while solver.check() == z3.sat and i < 10000:
        print(i)
        i += 1
        model = solver.model()
        for i in range(1, 4):
            all_vehs_waypoints[jj_id].append([toNum(model.evaluate(jj_coords[i-1][0])),jj_initial_position[1],toNum(model.evaluate(jj_coords[i-1][2]))])
        break
    all_vehs_waypoints[jj_id].append(jj_middle_position)
    
    jj_coords = [(z3.Real(f"{jj_id}_x{i}"), z3.Real(f"{jj_id}_y{i}"), z3.Real(f"{jj_id}_z{i}")) for i in range(1, 4)]
    exp = [z3.And(jj_coords[1-1][0]-jj_middle_position[0]==jj_coords[2-1][0]-jj_coords[1-1][0],jj_coords[2-1][0]-jj_coords[1-1][0]==jj_coords[3-1][0]-jj_coords[2-1][0],jj_coords[3-1][0]-jj_coords[2-1][0]==jj_final_position[0]-jj_coords[3-1][0])]+\
          [z3.And(jj_coords[1-1][2]-jj_middle_position[2]==jj_coords[2-1][2]-jj_coords[1-1][2],jj_coords[2-1][2]-jj_coords[1-1][2]==jj_coords[3-1][2]-jj_coords[2-1][2],jj_coords[3-1][2]-jj_coords[2-1][2]==jj_final_position[2]-jj_coords[3-1][2])]
    solver = z3.Solver()
    solver.add(exp)
    z3.set_option(rational_to_decimal=True)
    z3.set_option(precision=1)
    i = 0
    while solver.check() == z3.sat and i < 10000:
        i += 1
        model = solver.model()
        for i in range(1, 4):
            all_vehs_waypoints[jj_id].append([toNum(model.evaluate(jj_coords[i-1][0])),jj_initial_position[1],toNum(model.evaluate(jj_coords[i-1][2]))])
        break
    all_vehs_waypoints[jj_id].append(jj_final_position)
    all_vehs_waypoints[jj_id].append(["change lane"])
    return all_vehs_waypoints


def calculate_follow_lane_and_retrograde(all_vehs_waypoints,ii_id,jj_id,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map):
    if len(all_vehs_waypoints[ii_id]) == 0 and len(all_vehs_waypoints[jj_id]) == 0:
        all_vehs_waypoints[ii_id].append(["start"])
        all_vehs_waypoints[jj_id].append(["start"])
    elif len(all_vehs_waypoints[ii_id]) == 0 and len(all_vehs_waypoints[jj_id]) > 0:
        all_vehs_waypoints[ii_id].append(["start"])
    elif len(all_vehs_waypoints[ii_id]) > 0 and len(all_vehs_waypoints[jj_id]) == 0:
        all_vehs_waypoints[jj_id].append(["start"])
    elif len(all_vehs_waypoints[ii_id]) > 0 and len(all_vehs_waypoints[jj_id]) > 0:
        pass
    ii_initial_position = map[i_front[0]][i_front[1]][0]
    jj_initial_position = map[j_front[0]][j_front[1]][1]
    ii_final_position = map[i_back[0]][i_back[1]][1]
    jj_final_position = map[j_back[0]][j_back[1]][0]
    all_vehs_waypoints[ii_id].append(ii_initial_position)
    all_vehs_waypoints[jj_id].append(jj_initial_position)

    ii_coords = [(z3.Real(f"{ii_id}_x{i}"), z3.Real(f"{ii_id}_y{i}"), z3.Real(f"{ii_id}_z{i}")) for i in range(1, 4)]
    exp = [z3.And(ii_coords[1-1][0]-ii_initial_position[0]==ii_coords[2-1][0]-ii_coords[1-1][0],ii_coords[2-1][0]-ii_coords[1-1][0]==ii_coords[3-1][0]-ii_coords[2-1][0],ii_coords[3-1][0]-ii_coords[2-1][0]==ii_final_position[0]-ii_coords[3-1][0])]+\
          [z3.And(ii_coords[1-1][2]-ii_initial_position[2]==ii_coords[2-1][2]-ii_coords[1-1][2],ii_coords[2-1][2]-ii_coords[1-1][2]==ii_coords[3-1][2]-ii_coords[2-1][2],ii_coords[3-1][2]-ii_coords[2-1][2]==ii_final_position[2]-ii_coords[3-1][2])]
    solver = z3.Solver()
    solver.add(exp)
    z3.set_option(rational_to_decimal=True)
    z3.set_option(precision=1)
    i = 0
    while solver.check() == z3.sat and i < 10000:
        i += 1
        model = solver.model()
        for i in range(1, 4):
            all_vehs_waypoints[ii_id].append([toNum(model.evaluate(ii_coords[i-1][0])),ii_initial_position[1],toNum(model.evaluate(ii_coords[i-1][2]))])
        break
    all_vehs_waypoints[ii_id].append(ii_final_position)
    all_vehs_waypoints[ii_id].append(["follow lane"])

    jj_coords = [(z3.Real(f"{jj_id}_x{i}"), z3.Real(f"{jj_id}_y{i}"), z3.Real(f"{jj_id}_z{i}")) for i in range(1, 4)]
    exp = [z3.And(jj_coords[1-1][0]-jj_initial_position[0]==jj_coords[2-1][0]-jj_coords[1-1][0],jj_coords[2-1][0]-jj_coords[1-1][0]==jj_coords[3-1][0]-jj_coords[2-1][0],jj_coords[3-1][0]-jj_coords[2-1][0]==jj_final_position[0]-jj_coords[3-1][0])]+\
          [z3.And(jj_coords[1-1][2]-jj_initial_position[2]==jj_coords[2-1][2]-jj_coords[1-1][2],jj_coords[2-1][2]-jj_coords[1-1][2]==jj_coords[3-1][2]-jj_coords[2-1][2],jj_coords[3-1][2]-jj_coords[2-1][2]==jj_final_position[2]-jj_coords[3-1][2])]
    solver = z3.Solver()
    solver.add(exp)
    z3.set_option(rational_to_decimal=True)
    z3.set_option(precision=1)
    i = 0
    while solver.check() == z3.sat and i < 10000:
        i += 1
        model = solver.model()
        for i in range(1, 4):
            all_vehs_waypoints[jj_id].append([toNum(model.evaluate(jj_coords[i-1][0])),jj_initial_position[1],toNum(model.evaluate(jj_coords[i-1][2]))])
        break
    all_vehs_waypoints[jj_id].append(jj_final_position)
    all_vehs_waypoints[jj_id].append(["retrograde"])
    return all_vehs_waypoints

def calculate_turn_left_and_turn_left(all_vehs_waypoints,ii_id,jj_id,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map):
    if len(all_vehs_waypoints[ii_id]) == 0 and len(all_vehs_waypoints[jj_id]) == 0:
        all_vehs_waypoints[ii_id].append(["start"])
        all_vehs_waypoints[jj_id].append(["start"])
    elif len(all_vehs_waypoints[ii_id]) == 0 and len(all_vehs_waypoints[jj_id]) > 0:
        all_vehs_waypoints[ii_id].append(["start"])
    elif len(all_vehs_waypoints[ii_id]) > 0 and len(all_vehs_waypoints[jj_id]) == 0:
        all_vehs_waypoints[jj_id].append(["start"])
    elif len(all_vehs_waypoints[ii_id]) > 0 and len(all_vehs_waypoints[jj_id]) > 0:
        pass
    ii_initial_position = map[i_front[0]][i_front[1]][1]
    jj_initial_position = map[j_front[0]][j_front[1]][1]
    ii_final_position = map[i_back[0]][i_back[1]][0]
    jj_final_position = map[j_back[0]][j_back[1]][0]
    all_vehs_waypoints[ii_id].append(ii_initial_position)
    all_vehs_waypoints[jj_id].append(jj_initial_position)
    ii_start_direction_vec = map[i_front[0]][5]
    ii_end_direction_vec = map[i_back[0]][5]
    jj_start_direction_vec = map[j_front[0]][5]
    jj_end_direction_vec = map[j_back[0]][5]

    ii_coords = [(z3.Real(f"{ii_id}_x{i}"), z3.Real(f"{ii_id}_y{i}"), z3.Real(f"{ii_id}_z{i}")) for i in range(1, 4)]
    exp = [z3.And(ii_coords[1-1][0]-ii_initial_position[0]==ii_coords[2-1][0]-ii_coords[1-1][0],ii_coords[2-1][0]-ii_coords[1-1][0]==ii_coords[3-1][0]-ii_coords[2-1][0],ii_coords[3-1][0]-ii_coords[2-1][0]==ii_final_position[0]-ii_coords[3-1][0])]+\
          [z3.And(ii_coords[1-1][2]-ii_initial_position[2]==ii_coords[2-1][2]-ii_coords[1-1][2],ii_coords[2-1][2]-ii_coords[1-1][2]==ii_coords[3-1][2]-ii_coords[2-1][2],ii_coords[3-1][2]-ii_coords[2-1][2]==ii_final_position[2]-ii_coords[3-1][2])]+\
          [z3.And(dianji([ii_coords[1-1][0]-ii_initial_position[0],ii_coords[1-1][2]-ii_initial_position[2]],[ii_start_direction_vec[0],ii_start_direction_vec[2]])>0,dianji([ii_final_position[0]-ii_coords[1-1][0],ii_final_position[2]-ii_coords[1-1][2]],[ii_end_direction_vec[0],ii_end_direction_vec[2]])>0)]+\
          [z3.And(dianji([ii_coords[2-1][0]-ii_initial_position[0],ii_coords[2-1][2]-ii_initial_position[2]],[ii_start_direction_vec[0],ii_start_direction_vec[2]])>0,dianji([ii_final_position[0]-ii_coords[2-1][0],ii_final_position[2]-ii_coords[2-1][2]],[ii_end_direction_vec[0],ii_end_direction_vec[2]])>0)]+\
          [z3.And(dianji([ii_coords[3-1][0]-ii_initial_position[0],ii_coords[3-1][2]-ii_initial_position[2]],[ii_start_direction_vec[0],ii_start_direction_vec[2]])>0,dianji([ii_final_position[0]-ii_coords[3-1][0],ii_final_position[2]-ii_coords[3-1][2]],[ii_end_direction_vec[0],ii_end_direction_vec[2]])>0)]
    solver = z3.Solver()
    solver.add(exp)
    z3.set_option(rational_to_decimal=True)
    z3.set_option(precision=1)
    i = 0
    while solver.check() == z3.sat and i < 10000:
        # print(i)
        i += 1
        model = solver.model()
        for i in range(1, 4):
            all_vehs_waypoints[ii_id].append([toNum(model.evaluate(ii_coords[i-1][0])),ii_initial_position[1],toNum(model.evaluate(ii_coords[i-1][2]))])
        break
    all_vehs_waypoints[ii_id].append(ii_final_position)
    all_vehs_waypoints[ii_id].append(["turn left"])

    jj_coords = [(z3.Real(f"{jj_id}_x{i}"), z3.Real(f"{jj_id}_y{i}"), z3.Real(f"{jj_id}_z{i}")) for i in range(1, 4)]
    exp = [z3.And(jj_coords[1-1][0]-jj_initial_position[0]==jj_coords[2-1][0]-jj_coords[1-1][0],jj_coords[2-1][0]-jj_coords[1-1][0]==jj_coords[3-1][0]-jj_coords[2-1][0],jj_coords[3-1][0]-jj_coords[2-1][0]==jj_final_position[0]-jj_coords[3-1][0])]+\
          [z3.And(jj_coords[1-1][2]-jj_initial_position[2]==jj_coords[2-1][2]-jj_coords[1-1][2],jj_coords[2-1][2]-jj_coords[1-1][2]==jj_coords[3-1][2]-jj_coords[2-1][2],jj_coords[3-1][2]-jj_coords[2-1][2]==jj_final_position[2]-jj_coords[3-1][2])]+\
          [z3.And(dianji([jj_coords[1-1][0]-jj_initial_position[0],jj_coords[1-1][2]-jj_initial_position[2]],[jj_start_direction_vec[0],jj_start_direction_vec[2]])>0,dianji([jj_final_position[0]-jj_coords[1-1][0],jj_final_position[2]-jj_coords[1-1][2]],[jj_end_direction_vec[0],jj_end_direction_vec[2]])>0)]+\
          [z3.And(dianji([jj_coords[2-1][0]-jj_initial_position[0],jj_coords[2-1][2]-jj_initial_position[2]],[jj_start_direction_vec[0],jj_start_direction_vec[2]])>0,dianji([jj_final_position[0]-jj_coords[2-1][0],jj_final_position[2]-jj_coords[2-1][2]],[jj_end_direction_vec[0],jj_end_direction_vec[2]])>0)]+\
          [z3.And(dianji([jj_coords[3-1][0]-jj_initial_position[0],jj_coords[3-1][2]-jj_initial_position[2]],[jj_start_direction_vec[0],jj_start_direction_vec[2]])>0,dianji([jj_final_position[0]-jj_coords[3-1][0],jj_final_position[2]-jj_coords[3-1][2]],[jj_end_direction_vec[0],jj_end_direction_vec[2]])>0)]
    solver = z3.Solver()
    solver.add(exp)
    z3.set_option(rational_to_decimal=True)
    z3.set_option(precision=1)
    i = 0
    while solver.check() == z3.sat and i < 10000:
        # print(i)
        i += 1
        model = solver.model()
        for i in range(1, 4):
            all_vehs_waypoints[jj_id].append([toNum(model.evaluate(jj_coords[i-1][0])),jj_initial_position[1],toNum(model.evaluate(jj_coords[i-1][2]))])
        break
    all_vehs_waypoints[jj_id].append(jj_final_position)
    all_vehs_waypoints[jj_id].append(["turn left"])
    return all_vehs_waypoints

def calculate_turn_left_and_turn_right(all_vehs_waypoints,ii_id,jj_id,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map):
    if len(all_vehs_waypoints[ii_id]) == 0 and len(all_vehs_waypoints[jj_id]) == 0:
        all_vehs_waypoints[ii_id].append(["start"])
        all_vehs_waypoints[jj_id].append(["start"])
    elif len(all_vehs_waypoints[ii_id]) == 0 and len(all_vehs_waypoints[jj_id]) > 0:
        all_vehs_waypoints[ii_id].append(["start"])
    elif len(all_vehs_waypoints[ii_id]) > 0 and len(all_vehs_waypoints[jj_id]) == 0:
        all_vehs_waypoints[jj_id].append(["start"])
    elif len(all_vehs_waypoints[ii_id]) > 0 and len(all_vehs_waypoints[jj_id]) > 0:
        pass
    ii_initial_position = map[i_front[0]][i_front[1]][1]
    jj_initial_position = map[j_front[0]][j_front[1]][1]
    ii_final_position = map[i_back[0]][i_back[1]][0]
    jj_final_position = map[j_back[0]][j_back[1]][0]
    all_vehs_waypoints[ii_id].append(ii_initial_position)
    all_vehs_waypoints[jj_id].append(jj_initial_position)
    ii_start_direction_vec = map[i_front[0]][5]
    ii_end_direction_vec = map[i_back[0]][5]
    jj_start_direction_vec = map[j_front[0]][5]
    jj_end_direction_vec = map[j_back[0]][5]

    ii_coords = [(z3.Real(f"{ii_id}_x{i}"), z3.Real(f"{ii_id}_y{i}"), z3.Real(f"{ii_id}_z{i}")) for i in range(1, 4)]
    exp = [z3.And(ii_coords[1-1][0]-ii_initial_position[0]==ii_coords[2-1][0]-ii_coords[1-1][0],ii_coords[2-1][0]-ii_coords[1-1][0]==ii_coords[3-1][0]-ii_coords[2-1][0],ii_coords[3-1][0]-ii_coords[2-1][0]==ii_final_position[0]-ii_coords[3-1][0])]+\
          [z3.And(ii_coords[1-1][2]-ii_initial_position[2]==ii_coords[2-1][2]-ii_coords[1-1][2],ii_coords[2-1][2]-ii_coords[1-1][2]==ii_coords[3-1][2]-ii_coords[2-1][2],ii_coords[3-1][2]-ii_coords[2-1][2]==ii_final_position[2]-ii_coords[3-1][2])]+\
          [z3.And(dianji([ii_coords[1-1][0]-ii_initial_position[0],ii_coords[1-1][2]-ii_initial_position[2]],[ii_start_direction_vec[0],ii_start_direction_vec[2]])>0,dianji([ii_final_position[0]-ii_coords[1-1][0],ii_final_position[2]-ii_coords[1-1][2]],[ii_end_direction_vec[0],ii_end_direction_vec[2]])>0)]+\
          [z3.And(dianji([ii_coords[2-1][0]-ii_initial_position[0],ii_coords[2-1][2]-ii_initial_position[2]],[ii_start_direction_vec[0],ii_start_direction_vec[2]])>0,dianji([ii_final_position[0]-ii_coords[2-1][0],ii_final_position[2]-ii_coords[2-1][2]],[ii_end_direction_vec[0],ii_end_direction_vec[2]])>0)]+\
          [z3.And(dianji([ii_coords[3-1][0]-ii_initial_position[0],ii_coords[3-1][2]-ii_initial_position[2]],[ii_start_direction_vec[0],ii_start_direction_vec[2]])>0,dianji([ii_final_position[0]-ii_coords[3-1][0],ii_final_position[2]-ii_coords[3-1][2]],[ii_end_direction_vec[0],ii_end_direction_vec[2]])>0)]
    solver = z3.Solver()
    solver.add(exp)
    z3.set_option(rational_to_decimal=True)
    z3.set_option(precision=1)
    i = 0
    while solver.check() == z3.sat and i < 10000:
        # print(i)
        i += 1
        model = solver.model()
        for i in range(1, 4):
            all_vehs_waypoints[ii_id].append([toNum(model.evaluate(ii_coords[i-1][0])),ii_initial_position[1],toNum(model.evaluate(ii_coords[i-1][2]))])
        break
    all_vehs_waypoints[ii_id].append(ii_final_position)
    all_vehs_waypoints[ii_id].append(["turn left"])

    jj_coords = [(z3.Real(f"{jj_id}_x{i}"), z3.Real(f"{jj_id}_y{i}"), z3.Real(f"{jj_id}_z{i}")) for i in range(1, 4)]
    exp = [z3.And(jj_coords[1-1][0]-jj_initial_position[0]==jj_coords[2-1][0]-jj_coords[1-1][0],jj_coords[2-1][0]-jj_coords[1-1][0]==jj_coords[3-1][0]-jj_coords[2-1][0],jj_coords[3-1][0]-jj_coords[2-1][0]==jj_final_position[0]-jj_coords[3-1][0])]+\
          [z3.And(jj_coords[1-1][2]-jj_initial_position[2]==jj_coords[2-1][2]-jj_coords[1-1][2],jj_coords[2-1][2]-jj_coords[1-1][2]==jj_coords[3-1][2]-jj_coords[2-1][2],jj_coords[3-1][2]-jj_coords[2-1][2]==jj_final_position[2]-jj_coords[3-1][2])]+\
          [z3.And(dianji([jj_coords[1-1][0]-jj_initial_position[0],jj_coords[1-1][2]-jj_initial_position[2]],[jj_start_direction_vec[0],jj_start_direction_vec[2]])>0,dianji([jj_final_position[0]-jj_coords[1-1][0],jj_final_position[2]-jj_coords[1-1][2]],[jj_end_direction_vec[0],jj_end_direction_vec[2]])>0)]+\
          [z3.And(dianji([jj_coords[2-1][0]-jj_initial_position[0],jj_coords[2-1][2]-jj_initial_position[2]],[jj_start_direction_vec[0],jj_start_direction_vec[2]])>0,dianji([jj_final_position[0]-jj_coords[2-1][0],jj_final_position[2]-jj_coords[2-1][2]],[jj_end_direction_vec[0],jj_end_direction_vec[2]])>0)]+\
          [z3.And(dianji([jj_coords[3-1][0]-jj_initial_position[0],jj_coords[3-1][2]-jj_initial_position[2]],[jj_start_direction_vec[0],jj_start_direction_vec[2]])>0,dianji([jj_final_position[0]-jj_coords[3-1][0],jj_final_position[2]-jj_coords[3-1][2]],[jj_end_direction_vec[0],jj_end_direction_vec[2]])>0)]
    solver = z3.Solver()
    solver.add(exp)
    z3.set_option(rational_to_decimal=True)
    z3.set_option(precision=1)
    i = 0
    while solver.check() == z3.sat and i < 10000:
        # print(i)
        i += 1
        model = solver.model()
        for i in range(1, 4):
            all_vehs_waypoints[jj_id].append([toNum(model.evaluate(jj_coords[i-1][0])),jj_initial_position[1],toNum(model.evaluate(jj_coords[i-1][2]))])
        break
    all_vehs_waypoints[jj_id].append(jj_final_position)
    all_vehs_waypoints[jj_id].append(["turn right"])
    return all_vehs_waypoints
    

def calculate_turn_left_and_turn_around(all_vehs_waypoints,ii_id,jj_id,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map):
    if len(all_vehs_waypoints[ii_id]) == 0 and len(all_vehs_waypoints[jj_id]) == 0:
        all_vehs_waypoints[ii_id].append(["start"])
        all_vehs_waypoints[jj_id].append(["start"])
    elif len(all_vehs_waypoints[ii_id]) == 0 and len(all_vehs_waypoints[jj_id]) > 0:
        all_vehs_waypoints[ii_id].append(["start"])
    elif len(all_vehs_waypoints[ii_id]) > 0 and len(all_vehs_waypoints[jj_id]) == 0:
        all_vehs_waypoints[jj_id].append(["start"])
    elif len(all_vehs_waypoints[ii_id]) > 0 and len(all_vehs_waypoints[jj_id]) > 0:
        pass
    ii_initial_position = map[i_front[0]][i_front[1]][1]
    jj_initial_position = map[j_front[0]][j_front[1]][1]
    ii_final_position = map[i_back[0]][i_back[1]][0]
    jj_final_position = map[j_back[0]][j_back[1]][0]
    all_vehs_waypoints[ii_id].append(ii_initial_position)
    all_vehs_waypoints[jj_id].append(jj_initial_position)
    ii_start_direction_vec = map[i_front[0]][5]
    ii_end_direction_vec = map[i_back[0]][5]
    jj_start_direction_vec = map[j_front[0]][5]
    jj_end_direction_vec = map[j_back[0]][5]
    jj_middle_position = [(jj_initial_position[0]+jj_final_position[0])/2*8*jj_start_direction_vec[0],(jj_initial_position[1]+jj_final_position[1])/2*8*jj_start_direction_vec[1],(jj_initial_position[2]+jj_final_position[2])/2*8*jj_start_direction_vec[2]]
    jj_middle_direction_vec = [jj_final_position[0]-jj_initial_position[0],jj_final_position[1]-jj_initial_position[1],jj_final_position[2]-jj_initial_position[2]]

    ii_coords = [(z3.Real(f"{ii_id}_x{i}"), z3.Real(f"{ii_id}_y{i}"), z3.Real(f"{ii_id}_z{i}")) for i in range(1, 4)]
    exp = [z3.And(ii_coords[1-1][0]-ii_initial_position[0]==ii_coords[2-1][0]-ii_coords[1-1][0],ii_coords[2-1][0]-ii_coords[1-1][0]==ii_coords[3-1][0]-ii_coords[2-1][0],ii_coords[3-1][0]-ii_coords[2-1][0]==ii_final_position[0]-ii_coords[3-1][0])]+\
          [z3.And(ii_coords[1-1][2]-ii_initial_position[2]==ii_coords[2-1][2]-ii_coords[1-1][2],ii_coords[2-1][2]-ii_coords[1-1][2]==ii_coords[3-1][2]-ii_coords[2-1][2],ii_coords[3-1][2]-ii_coords[2-1][2]==ii_final_position[2]-ii_coords[3-1][2])]+\
          [z3.And(dianji([ii_coords[1-1][0]-ii_initial_position[0],ii_coords[1-1][2]-ii_initial_position[2]],[ii_start_direction_vec[0],ii_start_direction_vec[2]])>0,dianji([ii_final_position[0]-ii_coords[1-1][0],ii_final_position[2]-ii_coords[1-1][2]],[ii_end_direction_vec[0],ii_end_direction_vec[2]])>0)]+\
          [z3.And(dianji([ii_coords[2-1][0]-ii_initial_position[0],ii_coords[2-1][2]-ii_initial_position[2]],[ii_start_direction_vec[0],ii_start_direction_vec[2]])>0,dianji([ii_final_position[0]-ii_coords[2-1][0],ii_final_position[2]-ii_coords[2-1][2]],[ii_end_direction_vec[0],ii_end_direction_vec[2]])>0)]+\
          [z3.And(dianji([ii_coords[3-1][0]-ii_initial_position[0],ii_coords[3-1][2]-ii_initial_position[2]],[ii_start_direction_vec[0],ii_start_direction_vec[2]])>0,dianji([ii_final_position[0]-ii_coords[3-1][0],ii_final_position[2]-ii_coords[3-1][2]],[ii_end_direction_vec[0],ii_end_direction_vec[2]])>0)]
    solver = z3.Solver()
    solver.add(exp)
    z3.set_option(rational_to_decimal=True)
    z3.set_option(precision=1)
    i = 0
    while solver.check() == z3.sat and i < 10000:
        # print(i)
        i += 1
        model = solver.model()
        for i in range(1, 4):
            all_vehs_waypoints[ii_id].append([toNum(model.evaluate(ii_coords[i-1][0])),ii_initial_position[1],toNum(model.evaluate(ii_coords[i-1][2]))])
        break
    all_vehs_waypoints[ii_id].append(ii_final_position)
    all_vehs_waypoints[ii_id].append(["turn left"])

    jj_coords = [(z3.Real(f"{jj_id}_x{i}"), z3.Real(f"{jj_id}_y{i}"), z3.Real(f"{jj_id}_z{i}")) for i in range(1, 4)]
    exp = [z3.And(jj_coords[1-1][0]-jj_initial_position[0]==jj_coords[2-1][0]-jj_coords[1-1][0],jj_coords[2-1][0]-jj_coords[1-1][0]==jj_coords[3-1][0]-jj_coords[2-1][0],jj_coords[3-1][0]-jj_coords[2-1][0]==jj_middle_position[0]-jj_coords[3-1][0])]+\
          [z3.And(jj_coords[1-1][2]-jj_initial_position[2]==jj_coords[2-1][2]-jj_coords[1-1][2],jj_coords[2-1][2]-jj_coords[1-1][2]==jj_coords[3-1][2]-jj_coords[2-1][2],jj_coords[3-1][2]-jj_coords[2-1][2]==jj_middle_position[2]-jj_coords[3-1][2])]+\
          [z3.And(dianji([jj_coords[1-1][0]-jj_initial_position[0],jj_coords[1-1][2]-jj_initial_position[2]],[jj_start_direction_vec[0],jj_start_direction_vec[2]])>0,dianji([jj_middle_position[0]-jj_coords[1-1][0],jj_middle_position[2]-jj_coords[1-1][2]],[jj_middle_direction_vec[0],jj_middle_direction_vec[2]])>0)]+\
          [z3.And(dianji([jj_coords[2-1][0]-jj_initial_position[0],jj_coords[2-1][2]-jj_initial_position[2]],[jj_start_direction_vec[0],jj_start_direction_vec[2]])>0,dianji([jj_middle_position[0]-jj_coords[2-1][0],jj_middle_position[2]-jj_coords[2-1][2]],[jj_middle_direction_vec[0],jj_middle_direction_vec[2]])>0)]+\
          [z3.And(dianji([jj_coords[3-1][0]-jj_initial_position[0],jj_coords[3-1][2]-jj_initial_position[2]],[jj_start_direction_vec[0],jj_start_direction_vec[2]])>0,dianji([jj_middle_position[0]-jj_coords[3-1][0],jj_middle_position[2]-jj_coords[3-1][2]],[jj_middle_direction_vec[0],jj_middle_direction_vec[2]])>0)]
    solver = z3.Solver()
    solver.add(exp)
    z3.set_option(rational_to_decimal=True)
    z3.set_option(precision=1)
    i = 0
    while solver.check() == z3.sat and i < 10000:
        # print(i)
        i += 1
        model = solver.model()
        for i in range(1, 4):
            all_vehs_waypoints[jj_id].append([toNum(model.evaluate(jj_coords[i-1][0])),jj_initial_position[1],toNum(model.evaluate(jj_coords[i-1][2]))])
        break
    all_vehs_waypoints[jj_id].append(jj_middle_position)

    jj_coords = [(z3.Real(f"{jj_id}_x{i}"), z3.Real(f"{jj_id}_y{i}"), z3.Real(f"{jj_id}_z{i}")) for i in range(1, 4)]
    exp = [z3.And(jj_coords[1-1][0]-jj_middle_position[0]==jj_coords[2-1][0]-jj_coords[1-1][0],jj_coords[2-1][0]-jj_coords[1-1][0]==jj_coords[3-1][0]-jj_coords[2-1][0],jj_coords[3-1][0]-jj_coords[2-1][0]==jj_final_position[0]-jj_coords[3-1][0])]+\
          [z3.And(jj_coords[1-1][2]-jj_middle_position[2]==jj_coords[2-1][2]-jj_coords[1-1][2],jj_coords[2-1][2]-jj_coords[1-1][2]==jj_coords[3-1][2]-jj_coords[2-1][2],jj_coords[3-1][2]-jj_coords[2-1][2]==jj_final_position[2]-jj_coords[3-1][2])]+\
          [z3.And(dianji([jj_coords[1-1][0]-jj_middle_position[0],jj_coords[1-1][2]-jj_middle_position[2]],[jj_middle_direction_vec[0],jj_middle_direction_vec[2]])>0,dianji([jj_final_position[0]-jj_coords[1-1][0],jj_final_position[2]-jj_coords[1-1][2]],[jj_end_direction_vec[0],jj_end_direction_vec[2]])>0)]+\
          [z3.And(dianji([jj_coords[2-1][0]-jj_middle_position[0],jj_coords[2-1][2]-jj_middle_position[2]],[jj_middle_direction_vec[0],jj_middle_direction_vec[2]])>0,dianji([jj_final_position[0]-jj_coords[2-1][0],jj_final_position[2]-jj_coords[2-1][2]],[jj_end_direction_vec[0],jj_end_direction_vec[2]])>0)]+\
          [z3.And(dianji([jj_coords[3-1][0]-jj_middle_position[0],jj_coords[3-1][2]-jj_middle_position[2]],[jj_middle_direction_vec[0],jj_middle_direction_vec[2]])>0,dianji([jj_final_position[0]-jj_coords[3-1][0],jj_final_position[2]-jj_coords[3-1][2]],[jj_end_direction_vec[0],jj_end_direction_vec[2]])>0)]
    solver = z3.Solver()
    solver.add(exp)
    z3.set_option(rational_to_decimal=True)
    z3.set_option(precision=1)
    i = 0
    while solver.check() == z3.sat and i < 10000:
        # print(i)
        i += 1
        model = solver.model()
        for i in range(1, 4):
            all_vehs_waypoints[jj_id].append([toNum(model.evaluate(jj_coords[i-1][0])),jj_initial_position[1],toNum(model.evaluate(jj_coords[i-1][2]))])
        break
    all_vehs_waypoints[jj_id].append(jj_final_position)
    all_vehs_waypoints[jj_id].append(["turn around"])
    return all_vehs_waypoints
    

def calculate_turn_left_and_go_across(all_vehs_waypoints,ii_id,jj_id,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map):
    if len(all_vehs_waypoints[ii_id]) == 0 and len(all_vehs_waypoints[jj_id]) == 0:
        all_vehs_waypoints[ii_id].append(["start"])
        all_vehs_waypoints[jj_id].append(["start"])
    elif len(all_vehs_waypoints[ii_id]) == 0 and len(all_vehs_waypoints[jj_id]) > 0:
        all_vehs_waypoints[ii_id].append(["start"])
    elif len(all_vehs_waypoints[ii_id]) > 0 and len(all_vehs_waypoints[jj_id]) == 0:
        all_vehs_waypoints[jj_id].append(["start"])
    elif len(all_vehs_waypoints[ii_id]) > 0 and len(all_vehs_waypoints[jj_id]) > 0:
        pass
    ii_initial_position = map[i_front[0]][i_front[1]][1]
    jj_initial_position = map[j_front[0]][j_front[1]][1]
    ii_final_position = map[i_back[0]][i_back[1]][0]
    jj_final_position = map[j_back[0]][j_back[1]][0]
    all_vehs_waypoints[ii_id].append(ii_initial_position)
    all_vehs_waypoints[jj_id].append(jj_initial_position)
    ii_start_direction_vec = map[i_front[0]][5]
    ii_end_direction_vec = map[i_back[0]][5]

    ii_coords = [(z3.Real(f"{ii_id}_x{i}"), z3.Real(f"{ii_id}_y{i}"), z3.Real(f"{ii_id}_z{i}")) for i in range(1, 4)]
    exp = [z3.And(ii_coords[1-1][0]-ii_initial_position[0]==ii_coords[2-1][0]-ii_coords[1-1][0],ii_coords[2-1][0]-ii_coords[1-1][0]==ii_coords[3-1][0]-ii_coords[2-1][0],ii_coords[3-1][0]-ii_coords[2-1][0]==ii_final_position[0]-ii_coords[3-1][0])]+\
          [z3.And(ii_coords[1-1][2]-ii_initial_position[2]==ii_coords[2-1][2]-ii_coords[1-1][2],ii_coords[2-1][2]-ii_coords[1-1][2]==ii_coords[3-1][2]-ii_coords[2-1][2],ii_coords[3-1][2]-ii_coords[2-1][2]==ii_final_position[2]-ii_coords[3-1][2])]+\
          [z3.And(dianji([ii_coords[1-1][0]-ii_initial_position[0],ii_coords[1-1][2]-ii_initial_position[2]],[ii_start_direction_vec[0],ii_start_direction_vec[2]])>0,dianji([ii_final_position[0]-ii_coords[1-1][0],ii_final_position[2]-ii_coords[1-1][2]],[ii_end_direction_vec[0],ii_end_direction_vec[2]])>0)]+\
          [z3.And(dianji([ii_coords[2-1][0]-ii_initial_position[0],ii_coords[2-1][2]-ii_initial_position[2]],[ii_start_direction_vec[0],ii_start_direction_vec[2]])>0,dianji([ii_final_position[0]-ii_coords[2-1][0],ii_final_position[2]-ii_coords[2-1][2]],[ii_end_direction_vec[0],ii_end_direction_vec[2]])>0)]+\
          [z3.And(dianji([ii_coords[3-1][0]-ii_initial_position[0],ii_coords[3-1][2]-ii_initial_position[2]],[ii_start_direction_vec[0],ii_start_direction_vec[2]])>0,dianji([ii_final_position[0]-ii_coords[3-1][0],ii_final_position[2]-ii_coords[3-1][2]],[ii_end_direction_vec[0],ii_end_direction_vec[2]])>0)]
    solver = z3.Solver()
    solver.add(exp)
    z3.set_option(rational_to_decimal=True)
    z3.set_option(precision=1)
    i = 0
    while solver.check() == z3.sat and i < 10000:
        # print(i)
        i += 1
        model = solver.model()
        for i in range(1, 4):
            all_vehs_waypoints[ii_id].append([toNum(model.evaluate(ii_coords[i-1][0])),ii_initial_position[1],toNum(model.evaluate(ii_coords[i-1][2]))])
        break
    all_vehs_waypoints[ii_id].append(ii_final_position)
    all_vehs_waypoints[ii_id].append(["turn left"])

    jj_coords = [(z3.Real(f"{jj_id}_x{i}"), z3.Real(f"{jj_id}_y{i}"), z3.Real(f"{jj_id}_z{i}")) for i in range(1, 4)]
    exp = [z3.And(jj_coords[1-1][0]-jj_initial_position[0]==jj_coords[2-1][0]-jj_coords[1-1][0],jj_coords[2-1][0]-jj_coords[1-1][0]==jj_coords[3-1][0]-jj_coords[2-1][0],jj_coords[3-1][0]-jj_coords[2-1][0]==jj_final_position[0]-jj_coords[3-1][0])]+\
          [z3.And(jj_coords[1-1][2]-jj_initial_position[2]==jj_coords[2-1][2]-jj_coords[1-1][2],jj_coords[2-1][2]-jj_coords[1-1][2]==jj_coords[3-1][2]-jj_coords[2-1][2],jj_coords[3-1][2]-jj_coords[2-1][2]==jj_final_position[2]-jj_coords[3-1][2])]
    solver = z3.Solver()
    solver.add(exp)
    z3.set_option(rational_to_decimal=True)
    z3.set_option(precision=1)
    i = 0
    while solver.check() == z3.sat and i < 10000:
        i += 1
        model = solver.model()
        for i in range(1, 4):
            all_vehs_waypoints[jj_id].append([toNum(model.evaluate(jj_coords[i-1][0])),jj_initial_position[1],toNum(model.evaluate(jj_coords[i-1][2]))])
        break
    all_vehs_waypoints[jj_id].append(jj_final_position)
    all_vehs_waypoints[jj_id].append(["go across"])
    return all_vehs_waypoints

def calculate_turn_right_and_turn_right(all_vehs_waypoints,ii_id,jj_id,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map):
    if len(all_vehs_waypoints[ii_id]) == 0 and len(all_vehs_waypoints[jj_id]) == 0:
        all_vehs_waypoints[ii_id].append(["start"])
        all_vehs_waypoints[jj_id].append(["start"])
    elif len(all_vehs_waypoints[ii_id]) == 0 and len(all_vehs_waypoints[jj_id]) > 0:
        all_vehs_waypoints[ii_id].append(["start"])
    elif len(all_vehs_waypoints[ii_id]) > 0 and len(all_vehs_waypoints[jj_id]) == 0:
        all_vehs_waypoints[jj_id].append(["start"])
    elif len(all_vehs_waypoints[ii_id]) > 0 and len(all_vehs_waypoints[jj_id]) > 0:
        pass
    ii_initial_position = map[i_front[0]][i_front[1]][1]
    jj_initial_position = map[j_front[0]][j_front[1]][1]
    ii_final_position = map[i_back[0]][i_back[1]][0]
    jj_final_position = map[j_back[0]][j_back[1]][0]
    all_vehs_waypoints[ii_id].append(ii_initial_position)
    all_vehs_waypoints[jj_id].append(jj_initial_position)
    ii_start_direction_vec = map[i_front[0]][5]
    ii_end_direction_vec = map[i_back[0]][5]
    jj_start_direction_vec = map[j_front[0]][5]
    jj_end_direction_vec = map[j_back[0]][5]

    ii_coords = [(z3.Real(f"{ii_id}_x{i}"), z3.Real(f"{ii_id}_y{i}"), z3.Real(f"{ii_id}_z{i}")) for i in range(1, 4)]
    exp = [z3.And(ii_coords[1-1][0]-ii_initial_position[0]==ii_coords[2-1][0]-ii_coords[1-1][0],ii_coords[2-1][0]-ii_coords[1-1][0]==ii_coords[3-1][0]-ii_coords[2-1][0],ii_coords[3-1][0]-ii_coords[2-1][0]==ii_final_position[0]-ii_coords[3-1][0])]+\
          [z3.And(ii_coords[1-1][2]-ii_initial_position[2]==ii_coords[2-1][2]-ii_coords[1-1][2],ii_coords[2-1][2]-ii_coords[1-1][2]==ii_coords[3-1][2]-ii_coords[2-1][2],ii_coords[3-1][2]-ii_coords[2-1][2]==ii_final_position[2]-ii_coords[3-1][2])]+\
          [z3.And(dianji([ii_coords[1-1][0]-ii_initial_position[0],ii_coords[1-1][2]-ii_initial_position[2]],[ii_start_direction_vec[0],ii_start_direction_vec[2]])>0,dianji([ii_final_position[0]-ii_coords[1-1][0],ii_final_position[2]-ii_coords[1-1][2]],[ii_end_direction_vec[0],ii_end_direction_vec[2]])>0)]+\
          [z3.And(dianji([ii_coords[2-1][0]-ii_initial_position[0],ii_coords[2-1][2]-ii_initial_position[2]],[ii_start_direction_vec[0],ii_start_direction_vec[2]])>0,dianji([ii_final_position[0]-ii_coords[2-1][0],ii_final_position[2]-ii_coords[2-1][2]],[ii_end_direction_vec[0],ii_end_direction_vec[2]])>0)]+\
          [z3.And(dianji([ii_coords[3-1][0]-ii_initial_position[0],ii_coords[3-1][2]-ii_initial_position[2]],[ii_start_direction_vec[0],ii_start_direction_vec[2]])>0,dianji([ii_final_position[0]-ii_coords[3-1][0],ii_final_position[2]-ii_coords[3-1][2]],[ii_end_direction_vec[0],ii_end_direction_vec[2]])>0)]
    solver = z3.Solver()
    solver.add(exp)
    z3.set_option(rational_to_decimal=True)
    z3.set_option(precision=1)
    i = 0
    while solver.check() == z3.sat and i < 10000:
        # print(i)
        i += 1
        model = solver.model()
        for i in range(1, 4):
            all_vehs_waypoints[ii_id].append([toNum(model.evaluate(ii_coords[i-1][0])),ii_initial_position[1],toNum(model.evaluate(ii_coords[i-1][2]))])
        break
    all_vehs_waypoints[ii_id].append(ii_final_position)
    all_vehs_waypoints[ii_id].append(["turn right"])

    jj_coords = [(z3.Real(f"{jj_id}_x{i}"), z3.Real(f"{jj_id}_y{i}"), z3.Real(f"{jj_id}_z{i}")) for i in range(1, 4)]
    exp = [z3.And(jj_coords[1-1][0]-jj_initial_position[0]==jj_coords[2-1][0]-jj_coords[1-1][0],jj_coords[2-1][0]-jj_coords[1-1][0]==jj_coords[3-1][0]-jj_coords[2-1][0],jj_coords[3-1][0]-jj_coords[2-1][0]==jj_final_position[0]-jj_coords[3-1][0])]+\
          [z3.And(jj_coords[1-1][2]-jj_initial_position[2]==jj_coords[2-1][2]-jj_coords[1-1][2],jj_coords[2-1][2]-jj_coords[1-1][2]==jj_coords[3-1][2]-jj_coords[2-1][2],jj_coords[3-1][2]-jj_coords[2-1][2]==jj_final_position[2]-jj_coords[3-1][2])]+\
          [z3.And(dianji([jj_coords[1-1][0]-jj_initial_position[0],jj_coords[1-1][2]-jj_initial_position[2]],[jj_start_direction_vec[0],jj_start_direction_vec[2]])>0,dianji([jj_final_position[0]-jj_coords[1-1][0],jj_final_position[2]-jj_coords[1-1][2]],[jj_end_direction_vec[0],jj_end_direction_vec[2]])>0)]+\
          [z3.And(dianji([jj_coords[2-1][0]-jj_initial_position[0],jj_coords[2-1][2]-jj_initial_position[2]],[jj_start_direction_vec[0],jj_start_direction_vec[2]])>0,dianji([jj_final_position[0]-jj_coords[2-1][0],jj_final_position[2]-jj_coords[2-1][2]],[jj_end_direction_vec[0],jj_end_direction_vec[2]])>0)]+\
          [z3.And(dianji([jj_coords[3-1][0]-jj_initial_position[0],jj_coords[3-1][2]-jj_initial_position[2]],[jj_start_direction_vec[0],jj_start_direction_vec[2]])>0,dianji([jj_final_position[0]-jj_coords[3-1][0],jj_final_position[2]-jj_coords[3-1][2]],[jj_end_direction_vec[0],jj_end_direction_vec[2]])>0)]
    solver = z3.Solver()
    solver.add(exp)
    z3.set_option(rational_to_decimal=True)
    z3.set_option(precision=1)
    i = 0
    while solver.check() == z3.sat and i < 10000:
        # print(i)
        i += 1
        model = solver.model()
        for i in range(1, 4):
            all_vehs_waypoints[jj_id].append([toNum(model.evaluate(jj_coords[i-1][0])),jj_initial_position[1],toNum(model.evaluate(jj_coords[i-1][2]))])
        break
    all_vehs_waypoints[jj_id].append(jj_final_position)
    all_vehs_waypoints[jj_id].append(["turn right"])
    return all_vehs_waypoints

def calculate_turn_right_and_turn_around(all_vehs_waypoints,ii_id,jj_id,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map):
    if len(all_vehs_waypoints[ii_id]) == 0 and len(all_vehs_waypoints[jj_id]) == 0:
        all_vehs_waypoints[ii_id].append(["start"])
        all_vehs_waypoints[jj_id].append(["start"])
    elif len(all_vehs_waypoints[ii_id]) == 0 and len(all_vehs_waypoints[jj_id]) > 0:
        all_vehs_waypoints[ii_id].append(["start"])
    elif len(all_vehs_waypoints[ii_id]) > 0 and len(all_vehs_waypoints[jj_id]) == 0:
        all_vehs_waypoints[jj_id].append(["start"])
    elif len(all_vehs_waypoints[ii_id]) > 0 and len(all_vehs_waypoints[jj_id]) > 0:
        pass
    ii_initial_position = map[i_front[0]][i_front[1]][1]
    jj_initial_position = map[j_front[0]][j_front[1]][1]
    ii_final_position = map[i_back[0]][i_back[1]][0]
    jj_final_position = map[j_back[0]][j_back[1]][0]
    all_vehs_waypoints[ii_id].append(ii_initial_position)
    all_vehs_waypoints[jj_id].append(jj_initial_position)
    ii_start_direction_vec = map[i_front[0]][5]
    ii_end_direction_vec = map[i_back[0]][5]
    jj_start_direction_vec = map[j_front[0]][5]
    jj_end_direction_vec = map[j_back[0]][5]
    jj_middle_position = [(jj_initial_position[0]+jj_final_position[0])/2*8*jj_start_direction_vec[0],(jj_initial_position[1]+jj_final_position[1])/2*8*jj_start_direction_vec[1],(jj_initial_position[2]+jj_final_position[2])/2*8*jj_start_direction_vec[2]]
    jj_middle_direction_vec = [jj_final_position[0]-jj_initial_position[0],jj_final_position[1]-jj_initial_position[1],jj_final_position[2]-jj_initial_position[2]]

    ii_coords = [(z3.Real(f"{ii_id}_x{i}"), z3.Real(f"{ii_id}_y{i}"), z3.Real(f"{ii_id}_z{i}")) for i in range(1, 4)]
    exp = [z3.And(ii_coords[1-1][0]-ii_initial_position[0]==ii_coords[2-1][0]-ii_coords[1-1][0],ii_coords[2-1][0]-ii_coords[1-1][0]==ii_coords[3-1][0]-ii_coords[2-1][0],ii_coords[3-1][0]-ii_coords[2-1][0]==ii_final_position[0]-ii_coords[3-1][0])]+\
          [z3.And(ii_coords[1-1][2]-ii_initial_position[2]==ii_coords[2-1][2]-ii_coords[1-1][2],ii_coords[2-1][2]-ii_coords[1-1][2]==ii_coords[3-1][2]-ii_coords[2-1][2],ii_coords[3-1][2]-ii_coords[2-1][2]==ii_final_position[2]-ii_coords[3-1][2])]+\
          [z3.And(dianji([ii_coords[1-1][0]-ii_initial_position[0],ii_coords[1-1][2]-ii_initial_position[2]],[ii_start_direction_vec[0],ii_start_direction_vec[2]])>0,dianji([ii_final_position[0]-ii_coords[1-1][0],ii_final_position[2]-ii_coords[1-1][2]],[ii_end_direction_vec[0],ii_end_direction_vec[2]])>0)]+\
          [z3.And(dianji([ii_coords[2-1][0]-ii_initial_position[0],ii_coords[2-1][2]-ii_initial_position[2]],[ii_start_direction_vec[0],ii_start_direction_vec[2]])>0,dianji([ii_final_position[0]-ii_coords[2-1][0],ii_final_position[2]-ii_coords[2-1][2]],[ii_end_direction_vec[0],ii_end_direction_vec[2]])>0)]+\
          [z3.And(dianji([ii_coords[3-1][0]-ii_initial_position[0],ii_coords[3-1][2]-ii_initial_position[2]],[ii_start_direction_vec[0],ii_start_direction_vec[2]])>0,dianji([ii_final_position[0]-ii_coords[3-1][0],ii_final_position[2]-ii_coords[3-1][2]],[ii_end_direction_vec[0],ii_end_direction_vec[2]])>0)]
    solver = z3.Solver()
    solver.add(exp)
    z3.set_option(rational_to_decimal=True)
    z3.set_option(precision=1)
    i = 0
    while solver.check() == z3.sat and i < 10000:
        # print(i)
        i += 1
        model = solver.model()
        for i in range(1, 4):
            all_vehs_waypoints[ii_id].append([toNum(model.evaluate(ii_coords[i-1][0])),ii_initial_position[1],toNum(model.evaluate(ii_coords[i-1][2]))])
        break
    all_vehs_waypoints[ii_id].append(ii_final_position)
    all_vehs_waypoints[ii_id].append(["turn right"])

    jj_coords = [(z3.Real(f"{jj_id}_x{i}"), z3.Real(f"{jj_id}_y{i}"), z3.Real(f"{jj_id}_z{i}")) for i in range(1, 4)]
    exp = [z3.And(jj_coords[1-1][0]-jj_initial_position[0]==jj_coords[2-1][0]-jj_coords[1-1][0],jj_coords[2-1][0]-jj_coords[1-1][0]==jj_coords[3-1][0]-jj_coords[2-1][0],jj_coords[3-1][0]-jj_coords[2-1][0]==jj_middle_position[0]-jj_coords[3-1][0])]+\
          [z3.And(jj_coords[1-1][2]-jj_initial_position[2]==jj_coords[2-1][2]-jj_coords[1-1][2],jj_coords[2-1][2]-jj_coords[1-1][2]==jj_coords[3-1][2]-jj_coords[2-1][2],jj_coords[3-1][2]-jj_coords[2-1][2]==jj_middle_position[2]-jj_coords[3-1][2])]+\
          [z3.And(dianji([jj_coords[1-1][0]-jj_initial_position[0],jj_coords[1-1][2]-jj_initial_position[2]],[jj_start_direction_vec[0],jj_start_direction_vec[2]])>0,dianji([jj_middle_position[0]-jj_coords[1-1][0],jj_middle_position[2]-jj_coords[1-1][2]],[jj_middle_direction_vec[0],jj_middle_direction_vec[2]])>0)]+\
          [z3.And(dianji([jj_coords[2-1][0]-jj_initial_position[0],jj_coords[2-1][2]-jj_initial_position[2]],[jj_start_direction_vec[0],jj_start_direction_vec[2]])>0,dianji([jj_middle_position[0]-jj_coords[2-1][0],jj_middle_position[2]-jj_coords[2-1][2]],[jj_middle_direction_vec[0],jj_middle_direction_vec[2]])>0)]+\
          [z3.And(dianji([jj_coords[3-1][0]-jj_initial_position[0],jj_coords[3-1][2]-jj_initial_position[2]],[jj_start_direction_vec[0],jj_start_direction_vec[2]])>0,dianji([jj_middle_position[0]-jj_coords[3-1][0],jj_middle_position[2]-jj_coords[3-1][2]],[jj_middle_direction_vec[0],jj_middle_direction_vec[2]])>0)]
    solver = z3.Solver()
    solver.add(exp)
    z3.set_option(rational_to_decimal=True)
    z3.set_option(precision=1)
    i = 0
    while solver.check() == z3.sat and i < 10000:
        # print(i)
        i += 1
        model = solver.model()
        for i in range(1, 4):
            all_vehs_waypoints[jj_id].append([toNum(model.evaluate(jj_coords[i-1][0])),jj_initial_position[1],toNum(model.evaluate(jj_coords[i-1][2]))])
        break
    all_vehs_waypoints[jj_id].append(jj_middle_position)

    jj_coords = [(z3.Real(f"{jj_id}_x{i}"), z3.Real(f"{jj_id}_y{i}"), z3.Real(f"{jj_id}_z{i}")) for i in range(1, 4)]
    exp = [z3.And(jj_coords[1-1][0]-jj_middle_position[0]==jj_coords[2-1][0]-jj_coords[1-1][0],jj_coords[2-1][0]-jj_coords[1-1][0]==jj_coords[3-1][0]-jj_coords[2-1][0],jj_coords[3-1][0]-jj_coords[2-1][0]==jj_final_position[0]-jj_coords[3-1][0])]+\
          [z3.And(jj_coords[1-1][2]-jj_middle_position[2]==jj_coords[2-1][2]-jj_coords[1-1][2],jj_coords[2-1][2]-jj_coords[1-1][2]==jj_coords[3-1][2]-jj_coords[2-1][2],jj_coords[3-1][2]-jj_coords[2-1][2]==jj_final_position[2]-jj_coords[3-1][2])]+\
          [z3.And(dianji([jj_coords[1-1][0]-jj_middle_position[0],jj_coords[1-1][2]-jj_middle_position[2]],[jj_middle_direction_vec[0],jj_middle_direction_vec[2]])>0,dianji([jj_final_position[0]-jj_coords[1-1][0],jj_final_position[2]-jj_coords[1-1][2]],[jj_end_direction_vec[0],jj_end_direction_vec[2]])>0)]+\
          [z3.And(dianji([jj_coords[2-1][0]-jj_middle_position[0],jj_coords[2-1][2]-jj_middle_position[2]],[jj_middle_direction_vec[0],jj_middle_direction_vec[2]])>0,dianji([jj_final_position[0]-jj_coords[2-1][0],jj_final_position[2]-jj_coords[2-1][2]],[jj_end_direction_vec[0],jj_end_direction_vec[2]])>0)]+\
          [z3.And(dianji([jj_coords[3-1][0]-jj_middle_position[0],jj_coords[3-1][2]-jj_middle_position[2]],[jj_middle_direction_vec[0],jj_middle_direction_vec[2]])>0,dianji([jj_final_position[0]-jj_coords[3-1][0],jj_final_position[2]-jj_coords[3-1][2]],[jj_end_direction_vec[0],jj_end_direction_vec[2]])>0)]
    solver = z3.Solver()
    solver.add(exp)
    z3.set_option(rational_to_decimal=True)
    z3.set_option(precision=1)
    i = 0
    while solver.check() == z3.sat and i < 10000:
        # print(i)
        i += 1
        model = solver.model()
        for i in range(1, 4):
            all_vehs_waypoints[jj_id].append([toNum(model.evaluate(jj_coords[i-1][0])),jj_initial_position[1],toNum(model.evaluate(jj_coords[i-1][2]))])
        break
    all_vehs_waypoints[jj_id].append(jj_final_position)
    all_vehs_waypoints[jj_id].append(["turn around"])
    return all_vehs_waypoints

def calculate_turn_right_and_go_across(all_vehs_waypoints,ii_id,jj_id,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map):
    if len(all_vehs_waypoints[ii_id]) == 0 and len(all_vehs_waypoints[jj_id]) == 0:
        all_vehs_waypoints[ii_id].append(["start"])
        all_vehs_waypoints[jj_id].append(["start"])
    elif len(all_vehs_waypoints[ii_id]) == 0 and len(all_vehs_waypoints[jj_id]) > 0:
        all_vehs_waypoints[ii_id].append(["start"])
    elif len(all_vehs_waypoints[ii_id]) > 0 and len(all_vehs_waypoints[jj_id]) == 0:
        all_vehs_waypoints[jj_id].append(["start"])
    elif len(all_vehs_waypoints[ii_id]) > 0 and len(all_vehs_waypoints[jj_id]) > 0:
        pass
    ii_initial_position = map[i_front[0]][i_front[1]][1]
    jj_initial_position = map[j_front[0]][j_front[1]][1]
    ii_final_position = map[i_back[0]][i_back[1]][0]
    jj_final_position = map[j_back[0]][j_back[1]][0]
    all_vehs_waypoints[ii_id].append(ii_initial_position)
    all_vehs_waypoints[jj_id].append(jj_initial_position)
    ii_start_direction_vec = map[i_front[0]][5]
    ii_end_direction_vec = map[i_back[0]][5]

    ii_coords = [(z3.Real(f"{ii_id}_x{i}"), z3.Real(f"{ii_id}_y{i}"), z3.Real(f"{ii_id}_z{i}")) for i in range(1, 4)]
    exp = [z3.And(ii_coords[1-1][0]-ii_initial_position[0]==ii_coords[2-1][0]-ii_coords[1-1][0],ii_coords[2-1][0]-ii_coords[1-1][0]==ii_coords[3-1][0]-ii_coords[2-1][0],ii_coords[3-1][0]-ii_coords[2-1][0]==ii_final_position[0]-ii_coords[3-1][0])]+\
          [z3.And(ii_coords[1-1][2]-ii_initial_position[2]==ii_coords[2-1][2]-ii_coords[1-1][2],ii_coords[2-1][2]-ii_coords[1-1][2]==ii_coords[3-1][2]-ii_coords[2-1][2],ii_coords[3-1][2]-ii_coords[2-1][2]==ii_final_position[2]-ii_coords[3-1][2])]+\
          [z3.And(dianji([ii_coords[1-1][0]-ii_initial_position[0],ii_coords[1-1][2]-ii_initial_position[2]],[ii_start_direction_vec[0],ii_start_direction_vec[2]])>0,dianji([ii_final_position[0]-ii_coords[1-1][0],ii_final_position[2]-ii_coords[1-1][2]],[ii_end_direction_vec[0],ii_end_direction_vec[2]])>0)]+\
          [z3.And(dianji([ii_coords[2-1][0]-ii_initial_position[0],ii_coords[2-1][2]-ii_initial_position[2]],[ii_start_direction_vec[0],ii_start_direction_vec[2]])>0,dianji([ii_final_position[0]-ii_coords[2-1][0],ii_final_position[2]-ii_coords[2-1][2]],[ii_end_direction_vec[0],ii_end_direction_vec[2]])>0)]+\
          [z3.And(dianji([ii_coords[3-1][0]-ii_initial_position[0],ii_coords[3-1][2]-ii_initial_position[2]],[ii_start_direction_vec[0],ii_start_direction_vec[2]])>0,dianji([ii_final_position[0]-ii_coords[3-1][0],ii_final_position[2]-ii_coords[3-1][2]],[ii_end_direction_vec[0],ii_end_direction_vec[2]])>0)]
    solver = z3.Solver()
    solver.add(exp)
    z3.set_option(rational_to_decimal=True)
    z3.set_option(precision=1)
    i = 0
    while solver.check() == z3.sat and i < 10000:
        # print(i)
        i += 1
        model = solver.model()
        for i in range(1, 4):
            all_vehs_waypoints[ii_id].append([toNum(model.evaluate(ii_coords[i-1][0])),ii_initial_position[1],toNum(model.evaluate(ii_coords[i-1][2]))])
        break
    all_vehs_waypoints[ii_id].append(ii_final_position)
    all_vehs_waypoints[ii_id].append(["turn right"])

    jj_coords = [(z3.Real(f"{jj_id}_x{i}"), z3.Real(f"{jj_id}_y{i}"), z3.Real(f"{jj_id}_z{i}")) for i in range(1, 4)]
    exp = [z3.And(jj_coords[1-1][0]-jj_initial_position[0]==jj_coords[2-1][0]-jj_coords[1-1][0],jj_coords[2-1][0]-jj_coords[1-1][0]==jj_coords[3-1][0]-jj_coords[2-1][0],jj_coords[3-1][0]-jj_coords[2-1][0]==jj_final_position[0]-jj_coords[3-1][0])]+\
          [z3.And(jj_coords[1-1][2]-jj_initial_position[2]==jj_coords[2-1][2]-jj_coords[1-1][2],jj_coords[2-1][2]-jj_coords[1-1][2]==jj_coords[3-1][2]-jj_coords[2-1][2],jj_coords[3-1][2]-jj_coords[2-1][2]==jj_final_position[2]-jj_coords[3-1][2])]
    solver = z3.Solver()
    solver.add(exp)
    z3.set_option(rational_to_decimal=True)
    z3.set_option(precision=1)
    i = 0
    while solver.check() == z3.sat and i < 10000:
        i += 1
        model = solver.model()
        for i in range(1, 4):
            all_vehs_waypoints[jj_id].append([toNum(model.evaluate(jj_coords[i-1][0])),jj_initial_position[1],toNum(model.evaluate(jj_coords[i-1][2]))])
        break
    all_vehs_waypoints[jj_id].append(jj_final_position)
    all_vehs_waypoints[jj_id].append(["go across"])
    return all_vehs_waypoints

def calculate_turn_around_and_turn_around(all_vehs_waypoints,ii_id,jj_id,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map):
    if len(all_vehs_waypoints[ii_id]) == 0 and len(all_vehs_waypoints[jj_id]) == 0:
        all_vehs_waypoints[ii_id].append(["start"])
        all_vehs_waypoints[jj_id].append(["start"])
    elif len(all_vehs_waypoints[ii_id]) == 0 and len(all_vehs_waypoints[jj_id]) > 0:
        all_vehs_waypoints[ii_id].append(["start"])
    elif len(all_vehs_waypoints[ii_id]) > 0 and len(all_vehs_waypoints[jj_id]) == 0:
        all_vehs_waypoints[jj_id].append(["start"])
    elif len(all_vehs_waypoints[ii_id]) > 0 and len(all_vehs_waypoints[jj_id]) > 0:
        pass
    ii_initial_position = map[i_front[0]][i_front[1]][1]
    jj_initial_position = map[j_front[0]][j_front[1]][1]
    ii_final_position = map[i_back[0]][i_back[1]][0]
    jj_final_position = map[j_back[0]][j_back[1]][0]
    all_vehs_waypoints[ii_id].append(ii_initial_position)
    all_vehs_waypoints[jj_id].append(jj_initial_position)
    ii_start_direction_vec = map[i_front[0]][5]
    ii_end_direction_vec = map[i_back[0]][5]
    ii_middle_position = [(ii_initial_position[0]+ii_final_position[0])/2*8*ii_start_direction_vec[0],(ii_initial_position[1]+ii_final_position[1])/2*8*ii_start_direction_vec[1],(ii_initial_position[2]+ii_final_position[2])/2*8*ii_start_direction_vec[2]]
    ii_middle_direction_vec = [ii_final_position[0]-ii_initial_position[0],ii_final_position[1]-ii_initial_position[1],ii_final_position[2]-ii_initial_position[2]]
    jj_start_direction_vec = map[j_front[0]][5]
    jj_end_direction_vec = map[j_back[0]][5]
    jj_middle_position = [(jj_initial_position[0]+jj_final_position[0])/2*8*jj_start_direction_vec[0],(jj_initial_position[1]+jj_final_position[1])/2*8*jj_start_direction_vec[1],(jj_initial_position[2]+jj_final_position[2])/2*8*jj_start_direction_vec[2]]
    jj_middle_direction_vec = [jj_final_position[0]-jj_initial_position[0],jj_final_position[1]-jj_initial_position[1],jj_final_position[2]-jj_initial_position[2]]

    ii_coords = [(z3.Real(f"{ii_id}_x{i}"), z3.Real(f"{ii_id}_y{i}"), z3.Real(f"{ii_id}_z{i}")) for i in range(1, 4)]
    exp = [z3.And(ii_coords[1-1][0]-ii_initial_position[0]==ii_coords[2-1][0]-ii_coords[1-1][0],ii_coords[2-1][0]-ii_coords[1-1][0]==ii_coords[3-1][0]-ii_coords[2-1][0],ii_coords[3-1][0]-ii_coords[2-1][0]==ii_middle_position[0]-ii_coords[3-1][0])]+\
          [z3.And(ii_coords[1-1][2]-ii_initial_position[2]==ii_coords[2-1][2]-ii_coords[1-1][2],ii_coords[2-1][2]-ii_coords[1-1][2]==ii_coords[3-1][2]-ii_coords[2-1][2],ii_coords[3-1][2]-ii_coords[2-1][2]==ii_middle_position[2]-ii_coords[3-1][2])]+\
          [z3.And(dianji([ii_coords[1-1][0]-ii_initial_position[0],ii_coords[1-1][2]-ii_initial_position[2]],[ii_start_direction_vec[0],ii_start_direction_vec[2]])>0,dianji([ii_middle_position[0]-ii_coords[1-1][0],ii_middle_position[2]-ii_coords[1-1][2]],[ii_middle_direction_vec[0],ii_middle_direction_vec[2]])>0)]+\
          [z3.And(dianji([ii_coords[2-1][0]-ii_initial_position[0],ii_coords[2-1][2]-ii_initial_position[2]],[ii_start_direction_vec[0],ii_start_direction_vec[2]])>0,dianji([ii_middle_position[0]-ii_coords[2-1][0],ii_middle_position[2]-ii_coords[2-1][2]],[ii_middle_direction_vec[0],ii_middle_direction_vec[2]])>0)]+\
          [z3.And(dianji([ii_coords[3-1][0]-ii_initial_position[0],ii_coords[3-1][2]-ii_initial_position[2]],[ii_start_direction_vec[0],ii_start_direction_vec[2]])>0,dianji([ii_middle_position[0]-ii_coords[3-1][0],ii_middle_position[2]-ii_coords[3-1][2]],[ii_middle_direction_vec[0],ii_middle_direction_vec[2]])>0)]
    solver = z3.Solver()
    solver.add(exp)
    z3.set_option(rational_to_decimal=True)
    z3.set_option(precision=1)
    i = 0
    while solver.check() == z3.sat and i < 10000:
        # print(i)
        i += 1
        model = solver.model()
        for i in range(1, 4):
            all_vehs_waypoints[ii_id].append([toNum(model.evaluate(ii_coords[i-1][0])),ii_initial_position[1],toNum(model.evaluate(ii_coords[i-1][2]))])
        break
    all_vehs_waypoints[ii_id].append(ii_middle_position)

    ii_coords = [(z3.Real(f"{ii_id}_x{i}"), z3.Real(f"{ii_id}_y{i}"), z3.Real(f"{ii_id}_z{i}")) for i in range(1, 4)]
    exp = [z3.And(ii_coords[1-1][0]-ii_middle_position[0]==ii_coords[2-1][0]-ii_coords[1-1][0],ii_coords[2-1][0]-ii_coords[1-1][0]==ii_coords[3-1][0]-ii_coords[2-1][0],ii_coords[3-1][0]-ii_coords[2-1][0]==ii_final_position[0]-ii_coords[3-1][0])]+\
          [z3.And(ii_coords[1-1][2]-ii_middle_position[2]==ii_coords[2-1][2]-ii_coords[1-1][2],ii_coords[2-1][2]-ii_coords[1-1][2]==ii_coords[3-1][2]-ii_coords[2-1][2],ii_coords[3-1][2]-ii_coords[2-1][2]==ii_final_position[2]-ii_coords[3-1][2])]+\
          [z3.And(dianji([ii_coords[1-1][0]-ii_middle_position[0],ii_coords[1-1][2]-ii_middle_position[2]],[ii_middle_direction_vec[0],ii_middle_direction_vec[2]])>0,dianji([ii_final_position[0]-ii_coords[1-1][0],ii_final_position[2]-ii_coords[1-1][2]],[ii_end_direction_vec[0],ii_end_direction_vec[2]])>0)]+\
          [z3.And(dianji([ii_coords[2-1][0]-ii_middle_position[0],ii_coords[2-1][2]-ii_middle_position[2]],[ii_middle_direction_vec[0],ii_middle_direction_vec[2]])>0,dianji([ii_final_position[0]-ii_coords[2-1][0],ii_final_position[2]-ii_coords[2-1][2]],[ii_end_direction_vec[0],ii_end_direction_vec[2]])>0)]+\
          [z3.And(dianji([ii_coords[3-1][0]-ii_middle_position[0],ii_coords[3-1][2]-ii_middle_position[2]],[ii_middle_direction_vec[0],ii_middle_direction_vec[2]])>0,dianji([ii_final_position[0]-ii_coords[3-1][0],ii_final_position[2]-ii_coords[3-1][2]],[ii_end_direction_vec[0],ii_end_direction_vec[2]])>0)]
    solver = z3.Solver()
    solver.add(exp)
    z3.set_option(rational_to_decimal=True)
    z3.set_option(precision=1)
    i = 0
    while solver.check() == z3.sat and i < 10000:
        # print(i)
        i += 1
        model = solver.model()
        for i in range(1, 4):
            all_vehs_waypoints[ii_id].append([toNum(model.evaluate(ii_coords[i-1][0])),ii_initial_position[1],toNum(model.evaluate(ii_coords[i-1][2]))])
        break
    all_vehs_waypoints[ii_id].append(ii_final_position)
    all_vehs_waypoints[ii_id].append(["turn around"])

    jj_coords = [(z3.Real(f"{jj_id}_x{i}"), z3.Real(f"{jj_id}_y{i}"), z3.Real(f"{jj_id}_z{i}")) for i in range(1, 4)]
    exp = [z3.And(jj_coords[1-1][0]-jj_initial_position[0]==jj_coords[2-1][0]-jj_coords[1-1][0],jj_coords[2-1][0]-jj_coords[1-1][0]==jj_coords[3-1][0]-jj_coords[2-1][0],jj_coords[3-1][0]-jj_coords[2-1][0]==jj_middle_position[0]-jj_coords[3-1][0])]+\
          [z3.And(jj_coords[1-1][2]-jj_initial_position[2]==jj_coords[2-1][2]-jj_coords[1-1][2],jj_coords[2-1][2]-jj_coords[1-1][2]==jj_coords[3-1][2]-jj_coords[2-1][2],jj_coords[3-1][2]-jj_coords[2-1][2]==jj_middle_position[2]-jj_coords[3-1][2])]+\
          [z3.And(dianji([jj_coords[1-1][0]-jj_initial_position[0],jj_coords[1-1][2]-jj_initial_position[2]],[jj_start_direction_vec[0],jj_start_direction_vec[2]])>0,dianji([jj_middle_position[0]-jj_coords[1-1][0],jj_middle_position[2]-jj_coords[1-1][2]],[jj_middle_direction_vec[0],jj_middle_direction_vec[2]])>0)]+\
          [z3.And(dianji([jj_coords[2-1][0]-jj_initial_position[0],jj_coords[2-1][2]-jj_initial_position[2]],[jj_start_direction_vec[0],jj_start_direction_vec[2]])>0,dianji([jj_middle_position[0]-jj_coords[2-1][0],jj_middle_position[2]-jj_coords[2-1][2]],[jj_middle_direction_vec[0],jj_middle_direction_vec[2]])>0)]+\
          [z3.And(dianji([jj_coords[3-1][0]-jj_initial_position[0],jj_coords[3-1][2]-jj_initial_position[2]],[jj_start_direction_vec[0],jj_start_direction_vec[2]])>0,dianji([jj_middle_position[0]-jj_coords[3-1][0],jj_middle_position[2]-jj_coords[3-1][2]],[jj_middle_direction_vec[0],jj_middle_direction_vec[2]])>0)]
    solver = z3.Solver()
    solver.add(exp)
    z3.set_option(rational_to_decimal=True)
    z3.set_option(precision=1)
    i = 0
    while solver.check() == z3.sat and i < 10000:
        # print(i)
        i += 1
        model = solver.model()
        for i in range(1, 4):
            all_vehs_waypoints[jj_id].append([toNum(model.evaluate(jj_coords[i-1][0])),jj_initial_position[1],toNum(model.evaluate(jj_coords[i-1][2]))])
        break
    all_vehs_waypoints[jj_id].append(jj_middle_position)

    jj_coords = [(z3.Real(f"{jj_id}_x{i}"), z3.Real(f"{jj_id}_y{i}"), z3.Real(f"{jj_id}_z{i}")) for i in range(1, 4)]
    exp = [z3.And(jj_coords[1-1][0]-jj_middle_position[0]==jj_coords[2-1][0]-jj_coords[1-1][0],jj_coords[2-1][0]-jj_coords[1-1][0]==jj_coords[3-1][0]-jj_coords[2-1][0],jj_coords[3-1][0]-jj_coords[2-1][0]==jj_final_position[0]-jj_coords[3-1][0])]+\
          [z3.And(jj_coords[1-1][2]-jj_middle_position[2]==jj_coords[2-1][2]-jj_coords[1-1][2],jj_coords[2-1][2]-jj_coords[1-1][2]==jj_coords[3-1][2]-jj_coords[2-1][2],jj_coords[3-1][2]-jj_coords[2-1][2]==jj_final_position[2]-jj_coords[3-1][2])]+\
          [z3.And(dianji([jj_coords[1-1][0]-jj_middle_position[0],jj_coords[1-1][2]-jj_middle_position[2]],[jj_middle_direction_vec[0],jj_middle_direction_vec[2]])>0,dianji([jj_final_position[0]-jj_coords[1-1][0],jj_final_position[2]-jj_coords[1-1][2]],[jj_end_direction_vec[0],jj_end_direction_vec[2]])>0)]+\
          [z3.And(dianji([jj_coords[2-1][0]-jj_middle_position[0],jj_coords[2-1][2]-jj_middle_position[2]],[jj_middle_direction_vec[0],jj_middle_direction_vec[2]])>0,dianji([jj_final_position[0]-jj_coords[2-1][0],jj_final_position[2]-jj_coords[2-1][2]],[jj_end_direction_vec[0],jj_end_direction_vec[2]])>0)]+\
          [z3.And(dianji([jj_coords[3-1][0]-jj_middle_position[0],jj_coords[3-1][2]-jj_middle_position[2]],[jj_middle_direction_vec[0],jj_middle_direction_vec[2]])>0,dianji([jj_final_position[0]-jj_coords[3-1][0],jj_final_position[2]-jj_coords[3-1][2]],[jj_end_direction_vec[0],jj_end_direction_vec[2]])>0)]
    solver = z3.Solver()
    solver.add(exp)
    z3.set_option(rational_to_decimal=True)
    z3.set_option(precision=1)
    i = 0
    while solver.check() == z3.sat and i < 10000:
        # print(i)
        i += 1
        model = solver.model()
        for i in range(1, 4):
            all_vehs_waypoints[jj_id].append([toNum(model.evaluate(jj_coords[i-1][0])),jj_initial_position[1],toNum(model.evaluate(jj_coords[i-1][2]))])
        break
    all_vehs_waypoints[jj_id].append(jj_final_position)
    all_vehs_waypoints[jj_id].append(["turn around"])
    return all_vehs_waypoints

def calculate_turn_around_and_go_across(all_vehs_waypoints,ii_id,jj_id,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map):
    if len(all_vehs_waypoints[ii_id]) == 0 and len(all_vehs_waypoints[jj_id]) == 0:
        all_vehs_waypoints[ii_id].append(["start"])
        all_vehs_waypoints[jj_id].append(["start"])
    elif len(all_vehs_waypoints[ii_id]) == 0 and len(all_vehs_waypoints[jj_id]) > 0:
        all_vehs_waypoints[ii_id].append(["start"])
    elif len(all_vehs_waypoints[ii_id]) > 0 and len(all_vehs_waypoints[jj_id]) == 0:
        all_vehs_waypoints[jj_id].append(["start"])
    elif len(all_vehs_waypoints[ii_id]) > 0 and len(all_vehs_waypoints[jj_id]) > 0:
        pass
    ii_initial_position = map[i_front[0]][i_front[1]][1]
    jj_initial_position = map[j_front[0]][j_front[1]][1]
    ii_final_position = map[i_back[0]][i_back[1]][0]
    jj_final_position = map[j_back[0]][j_back[1]][0]
    all_vehs_waypoints[ii_id].append(ii_initial_position)
    all_vehs_waypoints[jj_id].append(jj_initial_position)
    ii_start_direction_vec = map[i_front[0]][5]
    ii_end_direction_vec = map[i_back[0]][5]
    ii_middle_position = [(ii_initial_position[0]+ii_final_position[0])/2*8*ii_start_direction_vec[0],(ii_initial_position[1]+ii_final_position[1])/2*8*ii_start_direction_vec[1],(ii_initial_position[2]+ii_final_position[2])/2*8*ii_start_direction_vec[2]]
    ii_middle_direction_vec = [ii_final_position[0]-ii_initial_position[0],ii_final_position[1]-ii_initial_position[1],ii_final_position[2]-ii_initial_position[2]]
    
    ii_coords = [(z3.Real(f"{ii_id}_x{i}"), z3.Real(f"{ii_id}_y{i}"), z3.Real(f"{ii_id}_z{i}")) for i in range(1, 4)]
    exp = [z3.And(ii_coords[1-1][0]-ii_initial_position[0]==ii_coords[2-1][0]-ii_coords[1-1][0],ii_coords[2-1][0]-ii_coords[1-1][0]==ii_coords[3-1][0]-ii_coords[2-1][0],ii_coords[3-1][0]-ii_coords[2-1][0]==ii_middle_position[0]-ii_coords[3-1][0])]+\
          [z3.And(ii_coords[1-1][2]-ii_initial_position[2]==ii_coords[2-1][2]-ii_coords[1-1][2],ii_coords[2-1][2]-ii_coords[1-1][2]==ii_coords[3-1][2]-ii_coords[2-1][2],ii_coords[3-1][2]-ii_coords[2-1][2]==ii_middle_position[2]-ii_coords[3-1][2])]+\
          [z3.And(dianji([ii_coords[1-1][0]-ii_initial_position[0],ii_coords[1-1][2]-ii_initial_position[2]],[ii_start_direction_vec[0],ii_start_direction_vec[2]])>0,dianji([ii_middle_position[0]-ii_coords[1-1][0],ii_middle_position[2]-ii_coords[1-1][2]],[ii_middle_direction_vec[0],ii_middle_direction_vec[2]])>0)]+\
          [z3.And(dianji([ii_coords[2-1][0]-ii_initial_position[0],ii_coords[2-1][2]-ii_initial_position[2]],[ii_start_direction_vec[0],ii_start_direction_vec[2]])>0,dianji([ii_middle_position[0]-ii_coords[2-1][0],ii_middle_position[2]-ii_coords[2-1][2]],[ii_middle_direction_vec[0],ii_middle_direction_vec[2]])>0)]+\
          [z3.And(dianji([ii_coords[3-1][0]-ii_initial_position[0],ii_coords[3-1][2]-ii_initial_position[2]],[ii_start_direction_vec[0],ii_start_direction_vec[2]])>0,dianji([ii_middle_position[0]-ii_coords[3-1][0],ii_middle_position[2]-ii_coords[3-1][2]],[ii_middle_direction_vec[0],ii_middle_direction_vec[2]])>0)]
    solver = z3.Solver()
    solver.add(exp)
    z3.set_option(rational_to_decimal=True)
    z3.set_option(precision=1)
    i = 0
    while solver.check() == z3.sat and i < 10000:
        # print(i)
        i += 1
        model = solver.model()
        for i in range(1, 4):
            all_vehs_waypoints[ii_id].append([toNum(model.evaluate(ii_coords[i-1][0])),ii_initial_position[1],toNum(model.evaluate(ii_coords[i-1][2]))])
    all_vehs_waypoints[ii_id].append(ii_middle_position)

    ii_coords = [(z3.Real(f"{ii_id}_x{i}"), z3.Real(f"{ii_id}_y{i}"), z3.Real(f"{ii_id}_z{i}")) for i in range(1, 4)]
    exp = [z3.And(ii_coords[1-1][0]-ii_middle_position[0]==ii_coords[2-1][0]-ii_coords[1-1][0],ii_coords[2-1][0]-ii_coords[1-1][0]==ii_coords[3-1][0]-ii_coords[2-1][0],ii_coords[3-1][0]-ii_coords[2-1][0]==ii_final_position[0]-ii_coords[3-1][0])]+\
          [z3.And(ii_coords[1-1][2]-ii_middle_position[2]==ii_coords[2-1][2]-ii_coords[1-1][2],ii_coords[2-1][2]-ii_coords[1-1][2]==ii_coords[3-1][2]-ii_coords[2-1][2],ii_coords[3-1][2]-ii_coords[2-1][2]==ii_final_position[2]-ii_coords[3-1][2])]+\
          [z3.And(dianji([ii_coords[1-1][0]-ii_middle_position[0],ii_coords[1-1][2]-ii_middle_position[2]],[ii_middle_direction_vec[0],ii_middle_direction_vec[2]])>0,dianji([ii_final_position[0]-ii_coords[1-1][0],ii_final_position[2]-ii_coords[1-1][2]],[ii_end_direction_vec[0],ii_end_direction_vec[2]])>0)]+\
          [z3.And(dianji([ii_coords[2-1][0]-ii_middle_position[0],ii_coords[2-1][2]-ii_middle_position[2]],[ii_middle_direction_vec[0],ii_middle_direction_vec[2]])>0,dianji([ii_final_position[0]-ii_coords[2-1][0],ii_final_position[2]-ii_coords[2-1][2]],[ii_end_direction_vec[0],ii_end_direction_vec[2]])>0)]+\
          [z3.And(dianji([ii_coords[3-1][0]-ii_middle_position[0],ii_coords[3-1][2]-ii_middle_position[2]],[ii_middle_direction_vec[0],ii_middle_direction_vec[2]])>0,dianji([ii_final_position[0]-ii_coords[3-1][0],ii_final_position[2]-ii_coords[3-1][2]],[ii_end_direction_vec[0],ii_end_direction_vec[2]])>0)]
    solver = z3.Solver()
    solver.add(exp)
    z3.set_option(rational_to_decimal=True)
    z3.set_option(precision=1)
    i = 0
    while solver.check() == z3.sat and i < 10000:
        # print(i)
        i += 1
        model = solver.model()
        for i in range(1, 4):
            all_vehs_waypoints[ii_id].append([toNum(model.evaluate(ii_coords[i-1][0])),ii_initial_position[1],toNum(model.evaluate(ii_coords[i-1][2]))])
    all_vehs_waypoints[ii_id].append(ii_final_position)
    all_vehs_waypoints[ii_id].append(["turn around"])

    jj_coords = [(z3.Real(f"{jj_id}_x{i}"), z3.Real(f"{jj_id}_y{i}"), z3.Real(f"{jj_id}_z{i}")) for i in range(1, 4)]
    exp = [z3.And(jj_coords[1-1][0]-jj_initial_position[0]==jj_coords[2-1][0]-jj_coords[1-1][0],jj_coords[2-1][0]-jj_coords[1-1][0]==jj_coords[3-1][0]-jj_coords[2-1][0],jj_coords[3-1][0]-jj_coords[2-1][0]==jj_final_position[0]-jj_coords[3-1][0])]+\
          [z3.And(jj_coords[1-1][2]-jj_initial_position[2]==jj_coords[2-1][2]-jj_coords[1-1][2],jj_coords[2-1][2]-jj_coords[1-1][2]==jj_coords[3-1][2]-jj_coords[2-1][2],jj_coords[3-1][2]-jj_coords[2-1][2]==jj_final_position[2]-jj_coords[3-1][2])]
    solver = z3.Solver()
    solver.add(exp)
    z3.set_option(rational_to_decimal=True)
    z3.set_option(precision=1)
    i = 0
    while solver.check() == z3.sat and i < 10000:
        i += 1
        model = solver.model()
        for i in range(1, 4):
            all_vehs_waypoints[jj_id].append([toNum(model.evaluate(jj_coords[i-1][0])),jj_initial_position[1],toNum(model.evaluate(jj_coords[i-1][2]))])
    all_vehs_waypoints[jj_id].append(jj_final_position)
    all_vehs_waypoints[jj_id].append(["go across"])
    return all_vehs_waypoints

def calculate_go_across_and_go_across(all_vehs_waypoints,ii_id,jj_id,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map):
    if len(all_vehs_waypoints[ii_id]) == 0 and len(all_vehs_waypoints[jj_id]) == 0:
        all_vehs_waypoints[ii_id].append(["start"])
        all_vehs_waypoints[jj_id].append(["start"])
    elif len(all_vehs_waypoints[ii_id]) == 0 and len(all_vehs_waypoints[jj_id]) > 0:
        all_vehs_waypoints[ii_id].append(["start"])
    elif len(all_vehs_waypoints[ii_id]) > 0 and len(all_vehs_waypoints[jj_id]) == 0:
        all_vehs_waypoints[jj_id].append(["start"])
    elif len(all_vehs_waypoints[ii_id]) > 0 and len(all_vehs_waypoints[jj_id]) > 0:
        pass
    ii_initial_position = map[i_front[0]][i_front[1]][1]
    jj_initial_position = map[j_front[0]][j_front[1]][1]
    ii_final_position = map[i_back[0]][i_back[1]][0]
    jj_final_position = map[j_back[0]][j_back[1]][0]
    all_vehs_waypoints[ii_id].append(ii_initial_position)
    all_vehs_waypoints[jj_id].append(jj_initial_position)

    ii_coords = [(z3.Real(f"{ii_id}_x{i}"), z3.Real(f"{ii_id}_y{i}"), z3.Real(f"{ii_id}_z{i}")) for i in range(1, 4)]
    exp = [z3.And(ii_coords[1-1][0]-ii_initial_position[0]==ii_coords[2-1][0]-ii_coords[1-1][0],ii_coords[2-1][0]-ii_coords[1-1][0]==ii_coords[3-1][0]-ii_coords[2-1][0],ii_coords[3-1][0]-ii_coords[2-1][0]==ii_final_position[0]-ii_coords[3-1][0])]+\
          [z3.And(ii_coords[1-1][2]-ii_initial_position[2]==ii_coords[2-1][2]-ii_coords[1-1][2],ii_coords[2-1][2]-ii_coords[1-1][2]==ii_coords[3-1][2]-ii_coords[2-1][2],ii_coords[3-1][2]-ii_coords[2-1][2]==ii_final_position[2]-ii_coords[3-1][2])]
    solver = z3.Solver()
    solver.add(exp)
    z3.set_option(rational_to_decimal=True)
    z3.set_option(precision=1)
    i = 0
    while solver.check() == z3.sat and i < 10000:
        i += 1
        model = solver.model()
        for i in range(1, 4):
            all_vehs_waypoints[ii_id].append([toNum(model.evaluate(ii_coords[i-1][0])),ii_initial_position[1],toNum(model.evaluate(ii_coords[i-1][2]))])
        break
    all_vehs_waypoints[ii_id].append(ii_final_position)
    all_vehs_waypoints[ii_id].append(["go across"])

    jj_coords = [(z3.Real(f"{jj_id}_x{i}"), z3.Real(f"{jj_id}_y{i}"), z3.Real(f"{jj_id}_z{i}")) for i in range(1, 4)]
    exp = [z3.And(jj_coords[1-1][0]-jj_initial_position[0]==jj_coords[2-1][0]-jj_coords[1-1][0],jj_coords[2-1][0]-jj_coords[1-1][0]==jj_coords[3-1][0]-jj_coords[2-1][0],jj_coords[3-1][0]-jj_coords[2-1][0]==jj_final_position[0]-jj_coords[3-1][0])]+\
          [z3.And(jj_coords[1-1][2]-jj_initial_position[2]==jj_coords[2-1][2]-jj_coords[1-1][2],jj_coords[2-1][2]-jj_coords[1-1][2]==jj_coords[3-1][2]-jj_coords[2-1][2],jj_coords[3-1][2]-jj_coords[2-1][2]==jj_final_position[2]-jj_coords[3-1][2])]
    solver = z3.Solver()
    solver.add(exp)
    z3.set_option(rational_to_decimal=True)
    z3.set_option(precision=1)
    i = 0
    while solver.check() == z3.sat and i < 10000:
        i += 1
        model = solver.model()
        for i in range(1, 4):
            all_vehs_waypoints[jj_id].append([toNum(model.evaluate(jj_coords[i-1][0])),jj_initial_position[1],toNum(model.evaluate(jj_coords[i-1][2]))])
        break
    all_vehs_waypoints[jj_id].append(jj_final_position)
    all_vehs_waypoints[jj_id].append(["go across"])
    return all_vehs_waypoints

def calculate_change_lane_and_change_lane(all_vehs_waypoints,ii_id,jj_id,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map):
    if len(all_vehs_waypoints[ii_id]) == 0 and len(all_vehs_waypoints[jj_id]) == 0:
        all_vehs_waypoints[ii_id].append(["start"])
        all_vehs_waypoints[jj_id].append(["start"])
    elif len(all_vehs_waypoints[ii_id]) == 0 and len(all_vehs_waypoints[jj_id]) > 0:
        all_vehs_waypoints[ii_id].append(["start"])
    elif len(all_vehs_waypoints[ii_id]) > 0 and len(all_vehs_waypoints[jj_id]) == 0:
        all_vehs_waypoints[jj_id].append(["start"])
    elif len(all_vehs_waypoints[ii_id]) > 0 and len(all_vehs_waypoints[jj_id]) > 0:
        pass
    ii_initial_position = map[i_front[0]][i_front[1]][0]
    jj_initial_position = map[j_front[0]][j_front[1]][0]
    ii_final_position = map[i_back[0]][i_back[1]][1]
    jj_final_position = map[j_back[0]][j_back[1]][1]
    ii_middle_position = [(ii_initial_position[0]+ii_final_position[0])/2,(ii_initial_position[1]+ii_final_position[1])/2,(ii_initial_position[2]+ii_final_position[2])/2]
    jj_middle_position = [(jj_initial_position[0]+jj_final_position[0])/2,(jj_initial_position[1]+jj_final_position[1])/2,(jj_initial_position[2]+jj_final_position[2])/2]
    all_vehs_waypoints[ii_id].append(ii_initial_position)
    all_vehs_waypoints[jj_id].append(jj_initial_position)

    ii_coords = [(z3.Real(f"{ii_id}_x{i}"), z3.Real(f"{ii_id}_y{i}"), z3.Real(f"{ii_id}_z{i}")) for i in range(1, 4)]
    exp = [z3.And(ii_coords[1-1][0]-ii_initial_position[0]==ii_coords[2-1][0]-ii_coords[1-1][0],ii_coords[2-1][0]-ii_coords[1-1][0]==ii_coords[3-1][0]-ii_coords[2-1][0],ii_coords[3-1][0]-ii_coords[2-1][0]==ii_middle_position[0]-ii_coords[3-1][0])]+\
          [z3.And(ii_coords[1-1][2]-ii_initial_position[2]==ii_coords[2-1][2]-ii_coords[1-1][2],ii_coords[2-1][2]-ii_coords[1-1][2]==ii_coords[3-1][2]-ii_coords[2-1][2],ii_coords[3-1][2]-ii_coords[2-1][2]==ii_middle_position[2]-ii_coords[3-1][2])]
    solver = z3.Solver()
    solver.add(exp)
    z3.set_option(rational_to_decimal=True)
    z3.set_option(precision=1)
    i = 0
    while solver.check() == z3.sat and i < 10000:
        # print(i)
        i += 1
        model = solver.model()
        for i in range(1, 4):
            all_vehs_waypoints[ii_id].append([toNum(model.evaluate(ii_coords[i-1][0])),ii_initial_position[1],toNum(model.evaluate(ii_coords[i-1][2]))])
        break
    all_vehs_waypoints[ii_id].append(ii_middle_position)
    
    ii_coords = [(z3.Real(f"{ii_id}_x{i}"), z3.Real(f"{ii_id}_y{i}"), z3.Real(f"{ii_id}_z{i}")) for i in range(1, 4)]
    exp = [z3.And(ii_coords[1-1][0]-ii_middle_position[0]==ii_coords[2-1][0]-ii_coords[1-1][0],ii_coords[2-1][0]-ii_coords[1-1][0]==ii_coords[3-1][0]-ii_coords[2-1][0],ii_coords[3-1][0]-ii_coords[2-1][0]==ii_final_position[0]-ii_coords[3-1][0])]+\
          [z3.And(ii_coords[1-1][2]-ii_middle_position[2]==ii_coords[2-1][2]-ii_coords[1-1][2],ii_coords[2-1][2]-ii_coords[1-1][2]==ii_coords[3-1][2]-ii_coords[2-1][2],ii_coords[3-1][2]-ii_coords[2-1][2]==ii_final_position[2]-ii_coords[3-1][2])]
    solver = z3.Solver()
    solver.add(exp)
    z3.set_option(rational_to_decimal=True)
    z3.set_option(precision=1)
    i = 0
    while solver.check() == z3.sat and i < 10000:
        i += 1
        model = solver.model()
        for i in range(1, 4):
            all_vehs_waypoints[ii_id].append([toNum(model.evaluate(ii_coords[i-1][0])),ii_initial_position[1],toNum(model.evaluate(ii_coords[i-1][2]))])
        break
    all_vehs_waypoints[ii_id].append(ii_final_position)
    all_vehs_waypoints[ii_id].append(["change lane"])

    jj_coords = [(z3.Real(f"{jj_id}_x{i}"), z3.Real(f"{jj_id}_y{i}"), z3.Real(f"{jj_id}_z{i}")) for i in range(1, 4)]
    exp = [z3.And(jj_coords[1-1][0]-jj_initial_position[0]==jj_coords[2-1][0]-jj_coords[1-1][0],jj_coords[2-1][0]-jj_coords[1-1][0]==jj_coords[3-1][0]-jj_coords[2-1][0],jj_coords[3-1][0]-jj_coords[2-1][0]==jj_middle_position[0]-jj_coords[3-1][0])]+\
          [z3.And(jj_coords[1-1][2]-jj_initial_position[2]==jj_coords[2-1][2]-jj_coords[1-1][2],jj_coords[2-1][2]-jj_coords[1-1][2]==jj_coords[3-1][2]-jj_coords[2-1][2],jj_coords[3-1][2]-jj_coords[2-1][2]==jj_middle_position[2]-jj_coords[3-1][2])]
    solver = z3.Solver()
    solver.add(exp)
    z3.set_option(rational_to_decimal=True)
    z3.set_option(precision=1)
    i = 0
    while solver.check() == z3.sat and i < 10000:
        # print(i)
        i += 1
        model = solver.model()
        for i in range(1, 4):
            all_vehs_waypoints[jj_id].append([toNum(model.evaluate(jj_coords[i-1][0])),jj_initial_position[1],toNum(model.evaluate(jj_coords[i-1][2]))])
        break
    all_vehs_waypoints[jj_id].append(jj_middle_position)
    
    jj_coords = [(z3.Real(f"{jj_id}_x{i}"), z3.Real(f"{jj_id}_y{i}"), z3.Real(f"{jj_id}_z{i}")) for i in range(1, 4)]
    exp = [z3.And(jj_coords[1-1][0]-jj_middle_position[0]==jj_coords[2-1][0]-jj_coords[1-1][0],jj_coords[2-1][0]-jj_coords[1-1][0]==jj_coords[3-1][0]-jj_coords[2-1][0],jj_coords[3-1][0]-jj_coords[2-1][0]==jj_final_position[0]-jj_coords[3-1][0])]+\
          [z3.And(jj_coords[1-1][2]-jj_middle_position[2]==jj_coords[2-1][2]-jj_coords[1-1][2],jj_coords[2-1][2]-jj_coords[1-1][2]==jj_coords[3-1][2]-jj_coords[2-1][2],jj_coords[3-1][2]-jj_coords[2-1][2]==jj_final_position[2]-jj_coords[3-1][2])]
    solver = z3.Solver()
    solver.add(exp)
    z3.set_option(rational_to_decimal=True)
    z3.set_option(precision=1)
    i = 0
    while solver.check() == z3.sat and i < 10000:
        i += 1
        model = solver.model()
        for i in range(1, 4):
            all_vehs_waypoints[jj_id].append([toNum(model.evaluate(jj_coords[i-1][0])),jj_initial_position[1],toNum(model.evaluate(jj_coords[i-1][2]))])
        break
    all_vehs_waypoints[jj_id].append(jj_final_position)
    all_vehs_waypoints[jj_id].append(["change lane"])
    return all_vehs_waypoints
    
def calculate_one_veh(all_vehs_waypoints,ii,jj,i_front,i_back,j_front,j_back,i_symbol,j_symbol,map,iiorjj):
    if len(all_vehs_waypoints[iiorjj]) == 0:
        all_vehs_waypoints[iiorjj].append(["start"])
    elif len(all_vehs_waypoints[iiorjj]) > 0:
        pass
    if iiorjj == ii:
        behaviour = i_symbol
        behaviour_front = i_front
        behaviour_back = i_back
    elif iiorjj == jj:
        behaviour = j_symbol
        behaviour_front = j_front
        behaviour_back = j_back
    if behaviour == ["follow lane"]:
        initial_position = map[behaviour_front[0]][behaviour_front[1]][0]
        final_position = map[behaviour_back[0]][behaviour_back[1]][1]
        all_vehs_waypoints[iiorjj].append(initial_position)

        iiorjj_coords = [(z3.Real(f"{iiorjj}_x{i}"), z3.Real(f"{iiorjj}_y{i}"), z3.Real(f"{iiorjj}_z{i}")) for i in range(1, 4)]
        exp = [z3.And(iiorjj_coords[1-1][0]-initial_position[0]==iiorjj_coords[2-1][0]-iiorjj_coords[1-1][0],iiorjj_coords[2-1][0]-iiorjj_coords[1-1][0]==iiorjj_coords[3-1][0]-iiorjj_coords[2-1][0],iiorjj_coords[3-1][0]-iiorjj_coords[2-1][0]==final_position[0]-iiorjj_coords[3-1][0])]+\
            [z3.And(iiorjj_coords[1-1][2]-initial_position[2]==iiorjj_coords[2-1][2]-iiorjj_coords[1-1][2],iiorjj_coords[2-1][2]-iiorjj_coords[1-1][2]==iiorjj_coords[3-1][2]-iiorjj_coords[2-1][2],iiorjj_coords[3-1][2]-iiorjj_coords[2-1][2]==final_position[2]-iiorjj_coords[3-1][2])]
        solver = z3.Solver()
        solver.add(exp)
        z3.set_option(rational_to_decimal=True)
        z3.set_option(precision=1)
        i = 0
        while solver.check() == z3.sat and i < 10000:
            i += 1
            model = solver.model()
            for i in range(1, 4):
                all_vehs_waypoints[iiorjj].append([toNum(model.evaluate(iiorjj_coords[i-1][0])),initial_position[1],toNum(model.evaluate(iiorjj_coords[i-1][2]))])
            break
        all_vehs_waypoints[iiorjj].append(final_position)
        all_vehs_waypoints[iiorjj].append(["follow lane"])
        return all_vehs_waypoints
    elif behaviour == ["turn left"]:
        initial_position = map[behaviour_front[0]][behaviour_front[1]][1]
        final_position = map[behaviour_back[0]][behaviour_back[1]][0]
        all_vehs_waypoints[iiorjj].append(initial_position)
        start_direction_vec = map[behaviour_front[0]][5]
        end_direction_vec = map[behaviour_back[0]][5]

        iiorjj_coords = [(z3.Real(f"{iiorjj}_x{i}"), z3.Real(f"{iiorjj}_y{i}"), z3.Real(f"{iiorjj}_z{i}")) for i in range(1, 4)]
        exp = [z3.And(iiorjj_coords[1-1][0]-initial_position[0]==iiorjj_coords[2-1][0]-iiorjj_coords[1-1][0],iiorjj_coords[2-1][0]-iiorjj_coords[1-1][0]==iiorjj_coords[3-1][0]-iiorjj_coords[2-1][0],iiorjj_coords[3-1][0]-iiorjj_coords[2-1][0]==final_position[0]-iiorjj_coords[3-1][0])]+\
            [z3.And(iiorjj_coords[1-1][2]-initial_position[2]==iiorjj_coords[2-1][2]-iiorjj_coords[1-1][2],iiorjj_coords[2-1][2]-iiorjj_coords[1-1][2]==iiorjj_coords[3-1][2]-iiorjj_coords[2-1][2],iiorjj_coords[3-1][2]-iiorjj_coords[2-1][2]==final_position[2]-iiorjj_coords[3-1][2])]+\
            [z3.And(dianji([iiorjj_coords[1-1][0]-initial_position[0],iiorjj_coords[1-1][2]-initial_position[2]],[start_direction_vec[0],start_direction_vec[2]])>0,dianji([final_position[0]-iiorjj_coords[1-1][0],final_position[2]-iiorjj_coords[1-1][2]],[end_direction_vec[0],end_direction_vec[2]])>0)]+\
            [z3.And(dianji([iiorjj_coords[2-1][0]-initial_position[0],iiorjj_coords[2-1][2]-initial_position[2]],[start_direction_vec[0],start_direction_vec[2]])>0,dianji([final_position[0]-iiorjj_coords[2-1][0],final_position[2]-iiorjj_coords[2-1][2]],[end_direction_vec[0],end_direction_vec[2]])>0)]+\
            [z3.And(dianji([iiorjj_coords[3-1][0]-initial_position[0],iiorjj_coords[3-1][2]-initial_position[2]],[start_direction_vec[0],start_direction_vec[2]])>0,dianji([final_position[0]-iiorjj_coords[3-1][0],final_position[2]-iiorjj_coords[3-1][2]],[end_direction_vec[0],end_direction_vec[2]])>0)]
        solver = z3.Solver()
        solver.add(exp)
        z3.set_option(rational_to_decimal=True)
        z3.set_option(precision=1)
        i = 0
        while solver.check() == z3.sat and i < 10000:
            # print(i)
            i += 1
            model = solver.model()
            for i in range(1, 4):
                all_vehs_waypoints[iiorjj].append([toNum(model.evaluate(iiorjj_coords[i-1][0])),initial_position[1],toNum(model.evaluate(iiorjj_coords[i-1][2]))])
            break
        all_vehs_waypoints[iiorjj].append(final_position)
        all_vehs_waypoints[iiorjj].append(["turn left"])
        return all_vehs_waypoints
    elif behaviour == ["turn right"]:
        initial_position = map[behaviour_front[0]][behaviour_front[1]][1]
        final_position = map[behaviour_back[0]][behaviour_back[1]][0]
        all_vehs_waypoints[iiorjj].append(initial_position)
        start_direction_vec = map[behaviour_front[0]][5]
        end_direction_vec = map[behaviour_back[0]][5]

        iiorjj_coords = [(z3.Real(f"{iiorjj}_x{i}"), z3.Real(f"{iiorjj}_y{i}"), z3.Real(f"{iiorjj}_z{i}")) for i in range(1, 4)]
        exp = [z3.And(iiorjj_coords[1-1][0]-initial_position[0]==iiorjj_coords[2-1][0]-iiorjj_coords[1-1][0],iiorjj_coords[2-1][0]-iiorjj_coords[1-1][0]==iiorjj_coords[3-1][0]-iiorjj_coords[2-1][0],iiorjj_coords[3-1][0]-iiorjj_coords[2-1][0]==final_position[0]-iiorjj_coords[3-1][0])]+\
            [z3.And(iiorjj_coords[1-1][2]-initial_position[2]==iiorjj_coords[2-1][2]-iiorjj_coords[1-1][2],iiorjj_coords[2-1][2]-iiorjj_coords[1-1][2]==iiorjj_coords[3-1][2]-iiorjj_coords[2-1][2],iiorjj_coords[3-1][2]-iiorjj_coords[2-1][2]==final_position[2]-iiorjj_coords[3-1][2])]+\
            [z3.And(dianji([iiorjj_coords[1-1][0]-initial_position[0],iiorjj_coords[1-1][2]-initial_position[2]],[start_direction_vec[0],start_direction_vec[2]])>0,dianji([final_position[0]-iiorjj_coords[1-1][0],final_position[2]-iiorjj_coords[1-1][2]],[end_direction_vec[0],end_direction_vec[2]])>0)]+\
            [z3.And(dianji([iiorjj_coords[2-1][0]-initial_position[0],iiorjj_coords[2-1][2]-initial_position[2]],[start_direction_vec[0],start_direction_vec[2]])>0,dianji([final_position[0]-iiorjj_coords[2-1][0],final_position[2]-iiorjj_coords[2-1][2]],[end_direction_vec[0],end_direction_vec[2]])>0)]+\
            [z3.And(dianji([iiorjj_coords[3-1][0]-initial_position[0],iiorjj_coords[3-1][2]-initial_position[2]],[start_direction_vec[0],start_direction_vec[2]])>0,dianji([final_position[0]-iiorjj_coords[3-1][0],final_position[2]-iiorjj_coords[3-1][2]],[end_direction_vec[0],end_direction_vec[2]])>0)]
        solver = z3.Solver()
        solver.add(exp)
        z3.set_option(rational_to_decimal=True)
        z3.set_option(precision=1)
        i = 0
        while solver.check() == z3.sat and i < 10000:
            # print(i)
            i += 1
            model = solver.model()
            for i in range(1, 4):
                all_vehs_waypoints[iiorjj].append([toNum(model.evaluate(iiorjj_coords[i-1][0])),initial_position[1],toNum(model.evaluate(iiorjj_coords[i-1][2]))])
            break
        all_vehs_waypoints[iiorjj].append(final_position)
        all_vehs_waypoints[iiorjj].append(["turn right"])
        return all_vehs_waypoints
    elif behaviour == ["turn around"]:
        initial_position = map[behaviour_front[0]][behaviour_front[1]][1]
        final_position = map[behaviour_back[0]][behaviour_back[1]][0]
        all_vehs_waypoints[iiorjj].append(initial_position)
        start_direction_vec = map[behaviour_front[0]][5]
        end_direction_vec = map[behaviour_back[0]][5]
        middle_position = [(initial_position[0]+final_position[0])/2*8*start_direction_vec[0],(initial_position[1]+final_position[1])/2*8*start_direction_vec[1],(initial_position[2]+final_position[2])/2*8*start_direction_vec[2]]
        middle_direction_vec = [final_position[0]-initial_position[0],final_position[1]-initial_position[1],final_position[2]-initial_position[2]]

        iiorjj_coords = [(z3.Real(f"{iiorjj}_x{i}"), z3.Real(f"{iiorjj}_y{i}"), z3.Real(f"{iiorjj}_z{i}")) for i in range(1, 4)]
        exp = [z3.And(iiorjj_coords[1-1][0]-initial_position[0]==iiorjj_coords[2-1][0]-iiorjj_coords[1-1][0],iiorjj_coords[2-1][0]-iiorjj_coords[1-1][0]==iiorjj_coords[3-1][0]-iiorjj_coords[2-1][0],iiorjj_coords[3-1][0]-iiorjj_coords[2-1][0]==middle_position[0]-iiorjj_coords[3-1][0])]+\
            [z3.And(iiorjj_coords[1-1][2]-initial_position[2]==iiorjj_coords[2-1][2]-iiorjj_coords[1-1][2],iiorjj_coords[2-1][2]-iiorjj_coords[1-1][2]==iiorjj_coords[3-1][2]-iiorjj_coords[2-1][2],iiorjj_coords[3-1][2]-iiorjj_coords[2-1][2]==middle_position[2]-iiorjj_coords[3-1][2])]+\
            [z3.And(dianji([iiorjj_coords[1-1][0]-initial_position[0],iiorjj_coords[1-1][2]-initial_position[2]],[start_direction_vec[0],start_direction_vec[2]])>0,dianji([middle_position[0]-iiorjj_coords[1-1][0],middle_position[2]-iiorjj_coords[1-1][2]],[middle_direction_vec[0],middle_direction_vec[2]])>0)]+\
            [z3.And(dianji([iiorjj_coords[2-1][0]-initial_position[0],iiorjj_coords[2-1][2]-initial_position[2]],[start_direction_vec[0],start_direction_vec[2]])>0,dianji([middle_position[0]-iiorjj_coords[2-1][0],middle_position[2]-iiorjj_coords[2-1][2]],[middle_direction_vec[0],middle_direction_vec[2]])>0)]+\
            [z3.And(dianji([iiorjj_coords[3-1][0]-initial_position[0],iiorjj_coords[3-1][2]-initial_position[2]],[start_direction_vec[0],start_direction_vec[2]])>0,dianji([middle_position[0]-iiorjj_coords[3-1][0],middle_position[2]-iiorjj_coords[3-1][2]],[middle_direction_vec[0],middle_direction_vec[2]])>0)]
        solver = z3.Solver()
        solver.add(exp)
        z3.set_option(rational_to_decimal=True)
        z3.set_option(precision=1)
        i = 0
        while solver.check() == z3.sat and i < 10000:
            # print(i)
            i += 1
            model = solver.model()
            for i in range(1, 4):
                all_vehs_waypoints[iiorjj].append([toNum(model.evaluate(iiorjj_coords[i-1][0])),initial_position[1],toNum(model.evaluate(iiorjj_coords[i-1][2]))])
            break
        all_vehs_waypoints[iiorjj].append(middle_position)

        iiorjj_coords = [(z3.Real(f"{iiorjj}_x{i}"), z3.Real(f"{iiorjj}_y{i}"), z3.Real(f"{iiorjj}_z{i}")) for i in range(1, 4)]
        exp = [z3.And(iiorjj_coords[1-1][0]-middle_position[0]==iiorjj_coords[2-1][0]-iiorjj_coords[1-1][0],iiorjj_coords[2-1][0]-iiorjj_coords[1-1][0]==iiorjj_coords[3-1][0]-iiorjj_coords[2-1][0],iiorjj_coords[3-1][0]-iiorjj_coords[2-1][0]==final_position[0]-iiorjj_coords[3-1][0])]+\
            [z3.And(iiorjj_coords[1-1][2]-middle_position[2]==iiorjj_coords[2-1][2]-iiorjj_coords[1-1][2],iiorjj_coords[2-1][2]-iiorjj_coords[1-1][2]==iiorjj_coords[3-1][2]-iiorjj_coords[2-1][2],iiorjj_coords[3-1][2]-iiorjj_coords[2-1][2]==final_position[2]-iiorjj_coords[3-1][2])]+\
            [z3.And(dianji([iiorjj_coords[1-1][0]-middle_position[0],iiorjj_coords[1-1][2]-middle_position[2]],[middle_direction_vec[0],middle_direction_vec[2]])>0,dianji([final_position[0]-iiorjj_coords[1-1][0],final_position[2]-iiorjj_coords[1-1][2]],[end_direction_vec[0],end_direction_vec[2]])>0)]+\
            [z3.And(dianji([iiorjj_coords[2-1][0]-middle_position[0],iiorjj_coords[2-1][2]-middle_position[2]],[middle_direction_vec[0],middle_direction_vec[2]])>0,dianji([final_position[0]-iiorjj_coords[2-1][0],final_position[2]-iiorjj_coords[2-1][2]],[end_direction_vec[0],end_direction_vec[2]])>0)]+\
            [z3.And(dianji([iiorjj_coords[3-1][0]-middle_position[0],iiorjj_coords[3-1][2]-middle_position[2]],[middle_direction_vec[0],middle_direction_vec[2]])>0,dianji([final_position[0]-iiorjj_coords[3-1][0],final_position[2]-iiorjj_coords[3-1][2]],[end_direction_vec[0],end_direction_vec[2]])>0)]
        solver = z3.Solver()
        solver.add(exp)
        z3.set_option(rational_to_decimal=True)
        z3.set_option(precision=1)
        i = 0
        while solver.check() == z3.sat and i < 10000:
            # print(i)
            i += 1
            model = solver.model()
            for i in range(1, 4):
                all_vehs_waypoints[iiorjj].append([toNum(model.evaluate(iiorjj_coords[i-1][0])),initial_position[1],toNum(model.evaluate(iiorjj_coords[i-1][2]))])
            break
        all_vehs_waypoints[iiorjj].append(final_position)
        all_vehs_waypoints[iiorjj].append(["turn around"])
        return all_vehs_waypoints
    elif behaviour == ["change lane"]:
        initial_position = map[behaviour_front[0]][behaviour_front[1]][0]
        final_position = map[behaviour_back[0]][behaviour_back[1]][1]
        middle_position = [(initial_position[0]+final_position[0])/2,(initial_position[1]+final_position[1])/2,(initial_position[2]+final_position[2])/2]
        all_vehs_waypoints[iiorjj].append(initial_position)

        iiorjj_coords = [(z3.Real(f"{iiorjj}_x{i}"), z3.Real(f"{iiorjj}_y{i}"), z3.Real(f"{iiorjj}_z{i}")) for i in range(1, 4)]
        exp = [z3.And(iiorjj_coords[1-1][0]-initial_position[0]==iiorjj_coords[2-1][0]-iiorjj_coords[1-1][0],iiorjj_coords[2-1][0]-iiorjj_coords[1-1][0]==iiorjj_coords[3-1][0]-iiorjj_coords[2-1][0],iiorjj_coords[3-1][0]-iiorjj_coords[2-1][0]==middle_position[0]-iiorjj_coords[3-1][0])]+\
            [z3.And(iiorjj_coords[1-1][2]-initial_position[2]==iiorjj_coords[2-1][2]-iiorjj_coords[1-1][2],iiorjj_coords[2-1][2]-iiorjj_coords[1-1][2]==iiorjj_coords[3-1][2]-iiorjj_coords[2-1][2],iiorjj_coords[3-1][2]-iiorjj_coords[2-1][2]==middle_position[2]-iiorjj_coords[3-1][2])]
            
        solver = z3.Solver()
        solver.add(exp)
        z3.set_option(rational_to_decimal=True)
        z3.set_option(precision=1)
        i = 0
        while solver.check() == z3.sat and i < 10000:
            # print(i)
            i += 1
            model = solver.model()
            for i in range(1, 4):
                all_vehs_waypoints[iiorjj].append([toNum(model.evaluate(iiorjj_coords[i-1][0])),initial_position[1],toNum(model.evaluate(iiorjj_coords[i-1][2]))])
            break
        all_vehs_waypoints[iiorjj].append(middle_position)

        iiorjj_coords = [(z3.Real(f"{iiorjj}_x{i}"), z3.Real(f"{iiorjj}_y{i}"), z3.Real(f"{iiorjj}_z{i}")) for i in range(1, 4)]
        exp = [z3.And(iiorjj_coords[1-1][0]-middle_position[0]==iiorjj_coords[2-1][0]-iiorjj_coords[1-1][0],iiorjj_coords[2-1][0]-iiorjj_coords[1-1][0]==iiorjj_coords[3-1][0]-iiorjj_coords[2-1][0],iiorjj_coords[3-1][0]-iiorjj_coords[2-1][0]==final_position[0]-iiorjj_coords[3-1][0])]+\
            [z3.And(iiorjj_coords[1-1][2]-middle_position[2]==iiorjj_coords[2-1][2]-iiorjj_coords[1-1][2],iiorjj_coords[2-1][2]-iiorjj_coords[1-1][2]==iiorjj_coords[3-1][2]-iiorjj_coords[2-1][2],iiorjj_coords[3-1][2]-iiorjj_coords[2-1][2]==final_position[2]-iiorjj_coords[3-1][2])]
            
        solver = z3.Solver()
        solver.add(exp)
        z3.set_option(rational_to_decimal=True)
        z3.set_option(precision=1)
        i = 0
        while solver.check() == z3.sat and i < 10000:
            # print(i)
            i += 1
            model = solver.model()
            for i in range(1, 4):
                all_vehs_waypoints[iiorjj].append([toNum(model.evaluate(iiorjj_coords[i-1][0])),initial_position[1],toNum(model.evaluate(iiorjj_coords[i-1][2]))])
            break
        all_vehs_waypoints[iiorjj].append(final_position)
        all_vehs_waypoints[iiorjj].append(["change lane"])
        return all_vehs_waypoints

    elif behaviour == ["go across"]:
        # print("behaviour front and back",behaviour_front,behaviour_back)
        initial_position = map[behaviour_front[0]][behaviour_front[1]][1]
        final_position = map[behaviour_back[0]][behaviour_back[1]][0]

        all_vehs_waypoints[iiorjj].append(initial_position)

        iiorjj_coords = [(z3.Real(f"{iiorjj}_x{i}"), z3.Real(f"{iiorjj}_y{i}"), z3.Real(f"{iiorjj}_z{i}")) for i in range(1, 4)]
        exp = [z3.And(iiorjj_coords[1-1][0]-initial_position[0]==iiorjj_coords[2-1][0]-iiorjj_coords[1-1][0],iiorjj_coords[2-1][0]-iiorjj_coords[1-1][0]==iiorjj_coords[3-1][0]-iiorjj_coords[2-1][0],iiorjj_coords[3-1][0]-iiorjj_coords[2-1][0]==final_position[0]-iiorjj_coords[3-1][0])]+\
            [z3.And(iiorjj_coords[1-1][2]-initial_position[2]==iiorjj_coords[2-1][2]-iiorjj_coords[1-1][2],iiorjj_coords[2-1][2]-iiorjj_coords[1-1][2]==iiorjj_coords[3-1][2]-iiorjj_coords[2-1][2],iiorjj_coords[3-1][2]-iiorjj_coords[2-1][2]==final_position[2]-iiorjj_coords[3-1][2])]
            
        solver = z3.Solver()
        solver.add(exp)
        z3.set_option(rational_to_decimal=True)
        z3.set_option(precision=1)
        i = 0
        while solver.check() == z3.sat and i < 10000:
            # print(i)
            i += 1
            model = solver.model()
            for i in range(1, 4):
                all_vehs_waypoints[iiorjj].append([toNum(model.evaluate(iiorjj_coords[i-1][0])),initial_position[1],toNum(model.evaluate(iiorjj_coords[i-1][2]))])
            break
        all_vehs_waypoints[iiorjj].append(final_position)
        all_vehs_waypoints[iiorjj].append(["go across"])
        return all_vehs_waypoints

    elif behaviour == ["drive into"]:
        initial_position = map[behaviour_front[0]][3]
        final_position = map[behaviour_back[0]][behaviour_back[1]][0]

        all_vehs_waypoints[iiorjj].append(initial_position)

        iiorjj_coords = [(z3.Real(f"{iiorjj}_x{i}"), z3.Real(f"{iiorjj}_y{i}"), z3.Real(f"{iiorjj}_z{i}")) for i in range(1, 4)]
        exp = [z3.And(iiorjj_coords[1-1][0]-initial_position[0]==iiorjj_coords[2-1][0]-iiorjj_coords[1-1][0],iiorjj_coords[2-1][0]-iiorjj_coords[1-1][0]==iiorjj_coords[3-1][0]-iiorjj_coords[2-1][0],iiorjj_coords[3-1][0]-iiorjj_coords[2-1][0]==final_position[0]-iiorjj_coords[3-1][0])]+\
            [z3.And(iiorjj_coords[1-1][2]-initial_position[2]==iiorjj_coords[2-1][2]-iiorjj_coords[1-1][2],iiorjj_coords[2-1][2]-iiorjj_coords[1-1][2]==iiorjj_coords[3-1][2]-iiorjj_coords[2-1][2],iiorjj_coords[3-1][2]-iiorjj_coords[2-1][2]==final_position[2]-iiorjj_coords[3-1][2])]
            
        solver = z3.Solver()
        solver.add(exp)
        z3.set_option(rational_to_decimal=True)
        z3.set_option(precision=1)
        i = 0
        while solver.check() == z3.sat and i < 10000:
            # print(i)
            i += 1
            model = solver.model()
            for i in range(1, 4):
                all_vehs_waypoints[iiorjj].append([toNum(model.evaluate(iiorjj_coords[i-1][0])),initial_position[1],toNum(model.evaluate(iiorjj_coords[i-1][2]))])
            break
        all_vehs_waypoints[iiorjj].append(final_position)
        all_vehs_waypoints[iiorjj].append(["drive into"])
        return all_vehs_waypoints
    elif behaviour == ["drive off"]:
        initial_position = map[behaviour_front[0]][behaviour_front[1]][0]
        final_position = map[behaviour_back[0]][4]

        all_vehs_waypoints[iiorjj].append(initial_position)

        iiorjj_coords = [(z3.Real(f"{iiorjj}_x{i}"), z3.Real(f"{iiorjj}_y{i}"), z3.Real(f"{iiorjj}_z{i}")) for i in range(1, 4)]
        exp = [z3.And(iiorjj_coords[1-1][0]-initial_position[0]==iiorjj_coords[2-1][0]-iiorjj_coords[1-1][0],iiorjj_coords[2-1][0]-iiorjj_coords[1-1][0]==iiorjj_coords[3-1][0]-iiorjj_coords[2-1][0],iiorjj_coords[3-1][0]-iiorjj_coords[2-1][0]==final_position[0]-iiorjj_coords[3-1][0])]+\
            [z3.And(iiorjj_coords[1-1][2]-initial_position[2]==iiorjj_coords[2-1][2]-iiorjj_coords[1-1][2],iiorjj_coords[2-1][2]-iiorjj_coords[1-1][2]==iiorjj_coords[3-1][2]-iiorjj_coords[2-1][2],iiorjj_coords[3-1][2]-iiorjj_coords[2-1][2]==final_position[2]-iiorjj_coords[3-1][2])]
            
        solver = z3.Solver()
        solver.add(exp)
        z3.set_option(rational_to_decimal=True)
        z3.set_option(precision=1)
        i = 0
        while solver.check() == z3.sat and i < 10000:
            # print(i)
            i += 1
            model = solver.model()
            for i in range(1, 4):
                all_vehs_waypoints[iiorjj].append([toNum(model.evaluate(iiorjj_coords[i-1][0])),initial_position[1],toNum(model.evaluate(iiorjj_coords[i-1][2]))])
            break
        all_vehs_waypoints[iiorjj].append(final_position)
        all_vehs_waypoints[iiorjj].append(["drive off"])
        return all_vehs_waypoints
    elif behaviour == ["retrograde"]:
        initial_position = map[behaviour_front[0]][behaviour_front[1]][1]
        final_position = map[behaviour_back[0]][behaviour_back[1]][0]

        all_vehs_waypoints[iiorjj].append(initial_position)

        iiorjj_coords = [(z3.Real(f"{iiorjj}_x{i}"), z3.Real(f"{iiorjj}_y{i}"), z3.Real(f"{iiorjj}_z{i}")) for i in range(1, 4)]
        exp = [z3.And(iiorjj_coords[1-1][0]-initial_position[0]==iiorjj_coords[2-1][0]-iiorjj_coords[1-1][0],iiorjj_coords[2-1][0]-iiorjj_coords[1-1][0]==iiorjj_coords[3-1][0]-iiorjj_coords[2-1][0],iiorjj_coords[3-1][0]-iiorjj_coords[2-1][0]==final_position[0]-iiorjj_coords[3-1][0])]+\
            [z3.And(iiorjj_coords[1-1][2]-initial_position[2]==iiorjj_coords[2-1][2]-iiorjj_coords[1-1][2],iiorjj_coords[2-1][2]-iiorjj_coords[1-1][2]==iiorjj_coords[3-1][2]-iiorjj_coords[2-1][2],iiorjj_coords[3-1][2]-iiorjj_coords[2-1][2]==final_position[2]-iiorjj_coords[3-1][2])]
            
        solver = z3.Solver()
        solver.add(exp)
        z3.set_option(rational_to_decimal=True)
        z3.set_option(precision=1)
        i = 0
        while solver.check() == z3.sat and i < 10000:
            # print(i)
            i += 1
            model = solver.model()
            for i in range(1, 4):
                all_vehs_waypoints[iiorjj].append([toNum(model.evaluate(iiorjj_coords[i-1][0])),initial_position[1],toNum(model.evaluate(iiorjj_coords[i-1][2]))])
            break
        all_vehs_waypoints[iiorjj].append(final_position)
        all_vehs_waypoints[iiorjj].append(["retrograde"])
        return all_vehs_waypoints
    elif behaviour == ["stop"]:
        return all_vehs_waypoints
    else:
        pass
    
def get_position_from_symbol(symbol,map):
    road = symbol[0]
    lane = symbol[1]
    if symbol[2] == "s":
        return map[road][lane][0]
    elif symbol[2] == "e":
        return map[road][lane][1]

def get_latest_seg_two_indexs(list):
    flag = 0
    index = []
    for i in range(1,len(list)+1):
        if list[-i][0] in ["follow lane","turn left","turn right","turn around","change lane","retrograde","go across","drive off","drive into","stop","start"]:
            index.append(len(list)-i)
            flag +=1
        if flag == 2:
            break
    index[0] = index[0]-1
    index[1] = index[1]+1
    temp = index[0]
    index[0] = index[1]
    index[1] = temp
    return index




