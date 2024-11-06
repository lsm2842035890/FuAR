import random
import z3
from z3 import Or
from approach.utils_lsm import toNum
from approach.utils_lsm import *

def test():
    x = z3.Real("x")
    y = z3.Real("y")
    condition = [z3.And(x >= 0 ,x <= 5)]
    condition1 = [z3.And(y >= 0 ,y <= 1)]
    exp = [x*x + y*y == 25]

    solver = z3.Solver()
    solver.add(condition + condition1 +exp)
    z3.set_option(rational_to_decimal=True)
    z3.set_option(precision=3)

    res = []
    i = 0
    while solver.check() == z3.sat and i < 1000:
        i += 1
        model = solver.model()
        print(toNum(model.evaluate(x)),toNum(model.evaluate(y)))
        solver.add(Or(x == model[x] + 0.001, y == model[y] + 0.001))


    #     waypoints = [[toNum(model.evaluate(w1X)), toNum(model.evaluate(w1Y)), toNum(model.evaluate(speed)), 0, 0, 0],
    #                 [toNum(model.evaluate(w2X)), toNum(model.evaluate(w2Y)), toNum(model.evaluate(speed)), 0, 0, 0],
    #                 [toNum(model.evaluate(w3X)), toNum(model.evaluate(w3Y)), toNum(model.evaluate(speed)), 0, 0, 0],
    #                 [toNum(model.evaluate(w4X)), toNum(model.evaluate(w4Y)), toNum(model.evaluate(speed)), 0, 0, 0],
    #                 [toNum(model.evaluate(w5X)), toNum(model.evaluate(w5Y)), toNum(model.evaluate(speed)), 0, 0, 0]]
    #     res.append(waypoints)

    #     solver.add(Or(w1X == model[w1X] + 0.5, w2X == model[w2X] + 0.5,
    #                 w3X == model[w3X] + 0.5, w4X == model[w4X] + 0.5,
    #                 w1Y == model[w1Y] + 0.5, w2Y == model[w2Y] + 0.5,
    #                 w3Y == model[w3Y] + 0.5, w4Y == model[w4Y] + 0.5))

    # return res[random.randint(0, len(res) - 1)]

def test1():
    x = 0
    y = 5
    x5 = 100
    y5 = 5
    x1 = z3.Real("x1")
    y1 = z3.Real("y1")
    x2 = z3.Real("x2")
    y2 = z3.Real("y2")
    x3 = z3.Real("x3")
    y3 = z3.Real("y3")
    x4 = z3.Real("x4")
    y4 = z3.Real("y4")
    exp = [x1 - x > 10] + [x2 - x1 > 10]+ [x3 - x2 > 10]+ [x4 - x3>10] + [x5 - x4 > 10] + \
          [z3.And(y1 >4,y1<6)] + [z3.And(y2 >4,y2<6)]+ [z3.And(y3 >4,y3<6)]+ [z3.And(y4 >4,y4<6)] + \
          [z3.And(x1-x==x2-x1,x2-x1==x3-x2,x3-x2==x4-x3,x4-x3==x5-x4)]
    solver = z3.Solver()
    solver.add(exp)
    z3.set_option(rational_to_decimal=True)
    z3.set_option(precision=1)
    i = 0
    while solver.check() == z3.sat and i < 10000:
        i += 1
        model = solver.model()
        print(toNum(model.evaluate(x1)),toNum(model.evaluate(y1)),toNum(model.evaluate(x2)),toNum(model.evaluate(y2)),
              toNum(model.evaluate(x3)),toNum(model.evaluate(y3)),toNum(model.evaluate(x4)),toNum(model.evaluate(y4)))
        solver.add(Or(x1 == model[x1] + 1, y1 == model[y1] + 1,x2 == model[x2] + 1, y2 == model[y2] + 1,
                      x3 == model[x3] + 1, y3 == model[y3] + 1,x4 == model[x4] + 1, y4 == model[y4] + 1))
        
def test2():
    x = 0
    y = 5
    x5 = 5
    y5 = 0
    x1 = z3.Real("x1")
    y1 = z3.Real("y1")
    x2 = z3.Real("x2")
    y2 = z3.Real("y2")
    x3 = z3.Real("x3")
    y3 = z3.Real("y3")
    x4 = z3.Real("x4")
    y4 = z3.Real("y4")
    exp = [z3.And(y1 >0,y1<5)] + [z3.And(y2 >0,y2<5)]+ [z3.And(y3 >0,y3<5)]+ [z3.And(y4 >0,y4<5)] + \
          [z3.And(x1-x==x2-x1,x2-x1==x3-x2,x3-x2==x4-x3,x4-x3==x5-x4)] + [x1*x1+y1*y1==25] +[x2*x2+y2*y2==25]+[x3*x3+y3*y3==25]+[x4*x4+y4*y4==25]
    solver = z3.Solver()
    solver.add(exp)
    z3.set_option(rational_to_decimal=True)
    z3.set_option(precision=1)
    i = 0
    while solver.check() == z3.sat and i < 10000:
        i += 1
        model = solver.model()
        print(toNum(model.evaluate(x1)),toNum(model.evaluate(y1)),toNum(model.evaluate(x2)),toNum(model.evaluate(y2)),
              toNum(model.evaluate(x3)),toNum(model.evaluate(y3)),toNum(model.evaluate(x4)),toNum(model.evaluate(y4)))
        solver.add(Or(x1 == model[x1] + 1, y1 == model[y1] + 1,x2 == model[x2] + 1, y2 == model[y2] + 1,
                      x3 == model[x3] + 1, y3 == model[y3] + 1,x4 == model[x4] + 1, y4 == model[y4] + 1))
        
def test3():
        ii_id = 'v1'
        jj_id = 'v2'
        ii_initial_position = [69.0000152587891, 10, -59.999992370605]
        ii_final_position = [37.700008392334, 10, -30]
        jj_initial_position = [68.0000152587891, 10, -58.999992370605]
        jj_final_position = [36.700008392334, 10, -31]
        ii_coords = [(z3.Real(f"{ii_id}_x{i}"), z3.Real(f"{ii_id}_y{i}"), z3.Real(f"{ii_id}_z{i}")) for i in range(1, 4)]
        jj_coords = [(z3.Real(f"{jj_id}_x{i}"), z3.Real(f"{jj_id}_y{i}"), z3.Real(f"{jj_id}_z{i}")) for i in range(1, 4)]
        print(ii_coords)
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
            # print(i)
            i += 1
            model = solver.model()
            for i in range(1, 4):
                print(toNum(model.evaluate(ii_coords[i-1][0])),toNum(model.evaluate(ii_coords[i-1][2])))
            for i in range(1, 4):
                print(toNum(model.evaluate(jj_coords[i-1][0])),toNum(model.evaluate(jj_coords[i-1][2])))
            # solver.add(Or(*(ii_coords[i - 1][0] == model[ii_coords[i - 1][0]] + 0.1 for i in range(1, 4))))
            # solver.add(Or(*(ii_coords[i - 1][2] == model[ii_coords[i - 1][2]] + 0.1 for i in range(1, 4))))
            # solver.add(Or(*(jj_coords[i - 1][0] == model[jj_coords[i - 1][0]] + 0.1 for i in range(1, 4))))
            # solver.add(Or(*(jj_coords[i - 1][2] == model[jj_coords[i - 1][2]] + 0.1 for i in range(1, 4))))
            break

def turn_left():
    ii_initial_position = [1, 0, 0]
    ii_final_position = [0, 0, 1]
    start_direction_vec = [0,1]
    end_direction_vec = [-1,0]
    ii_id = 'v1'
    ii_coords = [(z3.Real(f"{ii_id}_x{i}"), z3.Real(f"{ii_id}_y{i}"), z3.Real(f"{ii_id}_z{i}")) for i in range(1, 4)]
    exp = [z3.And(ii_coords[1-1][0]-ii_initial_position[0]==ii_coords[2-1][0]-ii_coords[1-1][0],ii_coords[2-1][0]-ii_coords[1-1][0]==ii_coords[3-1][0]-ii_coords[2-1][0],ii_coords[3-1][0]-ii_coords[2-1][0]==ii_final_position[0]-ii_coords[3-1][0])]+\
          [z3.And(ii_coords[1-1][2]-ii_initial_position[2]==ii_coords[2-1][2]-ii_coords[1-1][2],ii_coords[2-1][2]-ii_coords[1-1][2]==ii_coords[3-1][2]-ii_coords[2-1][2],ii_coords[3-1][2]-ii_coords[2-1][2]==ii_final_position[2]-ii_coords[3-1][2])]+\
          [z3.And(dianji([ii_coords[1-1][0]-ii_initial_position[0],ii_coords[1-1][2]-ii_initial_position[2]],start_direction_vec)>0,dianji([ii_final_position[0]-ii_coords[1-1][0],ii_final_position[2]-ii_coords[1-1][2]],end_direction_vec)>0)]+\
          [z3.And(dianji([ii_coords[2-1][0]-ii_initial_position[0],ii_coords[2-1][2]-ii_initial_position[2]],start_direction_vec)>0,dianji([ii_final_position[0]-ii_coords[2-1][0],ii_final_position[2]-ii_coords[2-1][2]],end_direction_vec)>0)]+\
          [z3.And(dianji([ii_coords[3-1][0]-ii_initial_position[0],ii_coords[3-1][2]-ii_initial_position[2]],start_direction_vec)>0,dianji([ii_final_position[0]-ii_coords[3-1][0],ii_final_position[2]-ii_coords[3-1][2]],end_direction_vec)>0)]
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
            print(toNum(model.evaluate(ii_coords[i-1][0])),toNum(model.evaluate(ii_coords[i-1][2])))
        break

def turn_around():
    ii_initial_position = [1, 0, 0]
    ii_final_position = [-1, 0, 0]
    ii_middle_position = [(ii_initial_position[0]+ii_final_position[0])/2*8*start_direction_vec[0], (ii_initial_position[1]+ii_final_position[1])/2, (ii_initial_position[2]+ii_final_position[2])/2*8*start_direction_vec[1]]
    start_direction_vec = [0,1]
    middle_direction_vec = [ii_final_position[0]-ii_initial_position[0],ii_final_position[2]-ii_initial_position[2]]
    end_direction_vec = [0,-1]
    ii_id = 'v1'
    ii_coords = [(z3.Real(f"{ii_id}_x{i}"), z3.Real(f"{ii_id}_y{i}"), z3.Real(f"{ii_id}_z{i}")) for i in range(1, 6)]
    exp = [z3.And(ii_coords[1-1][0]-ii_initial_position[0]==ii_coords[2-1][0]-ii_coords[1-1][0],ii_coords[2-1][0]-ii_coords[1-1][0]==ii_coords[3-1][0]-ii_coords[2-1][0],ii_coords[3-1][0]-ii_coords[2-1][0]==ii_final_position[0]-ii_coords[3-1][0])]+\
          [z3.And(ii_coords[1-1][2]-ii_initial_position[2]==ii_coords[2-1][2]-ii_coords[1-1][2],ii_coords[2-1][2]-ii_coords[1-1][2]==ii_coords[3-1][2]-ii_coords[2-1][2],ii_coords[3-1][2]-ii_coords[2-1][2]==ii_final_position[2]-ii_coords[3-1][2])]+\
          [z3.And(dianji([ii_coords[1-1][0]-ii_initial_position[0],ii_coords[1-1][2]-ii_initial_position[2]],start_direction_vec)>0,dianji([ii_final_position[0]-ii_coords[1-1][0],ii_final_position[2]-ii_coords[1-1][2]],end_direction_vec)>0)]+\
          [z3.And(dianji([ii_coords[2-1][0]-ii_initial_position[0],ii_coords[2-1][2]-ii_initial_position[2]],start_direction_vec)>0,dianji([ii_final_position[0]-ii_coords[2-1][0],ii_final_position[2]-ii_coords[2-1][2]],end_direction_vec)>0)]+\
          [z3.And(dianji([ii_coords[3-1][0]-ii_initial_position[0],ii_coords[3-1][2]-ii_initial_position[2]],start_direction_vec)>0,dianji([ii_final_position[0]-ii_coords[3-1][0],ii_final_position[2]-ii_coords[3-1][2]],end_direction_vec)>0)]
    pass

# test2()
# test3()
turn_left()
