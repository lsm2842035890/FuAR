import math
import time
import random
from lgsvl import Transform, Vector
from lgsvl.dreamview import CoordType
from datetime import datetime
from environs import Env
import lgsvl
import threading
from CyberBridge import CyberBridge
from CyberBridge import Topics
from lgsvl_method import CyberBridgeInstance

def cb(data):   
    print(data)

def get_location():
    ######
    print("Waiting for localization")
    cyber = CyberBridge("127.0.0.1", 9090)
    # cyber.add_publisher(Topics.Planning)
    # cyber.add_subscriber(Topics.Planning,cb)
    cyber.add_subscriber(Topics.TrafficLight,cb)
    cyber.spin()
    ######
    # while True:
    #     print("Waiting for localization")
    #     cyber.receive_publish()



def testApollo(tag):
    env = Env()

    SIMULATOR_HOST = env.str("LGSVL__SIMULATOR_HOST", "127.0.0.1")
    SIMULATOR_PORT = env.int("LGSVL__SIMULATOR_PORT", 8181)
    BRIDGE_HOST = env.str("LGSVL__AUTOPILOT_0_HOST", "127.0.0.1")
    BRIDGE_PORT = env.int("LGSVL__AUTOPILOT_0_PORT", 9090)

    LGSVL__AUTOPILOT_HD_MAP = env.str("LGSVL__AUTOPILOT_HD_MAP", "san_francisco")
    LGSVL__AUTOPILOT_0_VEHICLE_CONFIG = env.str("LGSVL__AUTOPILOT_0_VEHICLE_CONFIG", 'Lincoln2017MKZ_LGSVL')
    LGSVL__SIMULATION_DURATION_SECS = 50

    vehicle_conf = env.str("LGSVL__VEHICLE_0", lgsvl.wise.DefaultAssets.ego_lincoln2017mkz_apollo6_modular)
    scene_name = env.str("LGSVL__MAP", lgsvl.wise.DefaultAssets.map_sanfrancisco_correct)
    sim = lgsvl.Simulator(SIMULATOR_HOST, SIMULATOR_PORT)

    if sim.current_scene == scene_name:
        sim.reset()
    else:
        sim.load(scene_name)

    sim.set_date_time(datetime(2022, 6, 22, 11, 0, 0, 0), True)

    spawns = sim.get_spawn()
    state2 = lgsvl.AgentState()
    # state2.transform = spawns[0]
    state2.transform = sim.map_point_on_lane(lgsvl.Vector(-297.000030517578, 10, 4.99996948242188))
    print(state2)

    print("Loading vehicle {}...".format(vehicle_conf))
    ego = sim.add_agent(vehicle_conf, lgsvl.AgentType.EGO, state2)
    print("Connecting to bridge...")
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
    time.sleep(5)
    dv.setup_apollo(592598.94, 4134701.40, default_modules)
    dv.set_destination(592598.94,4134701.40,coord_type=CoordType.Northing)
    sim.run(LGSVL__SIMULATION_DURATION_SECS)
    sim.close()

def run_cyberbridgeinstance():
    cyber = CyberBridgeInstance()
    cyber.register(None,[592598.94,4134701.40],[],"san_francisco",True)

if __name__ == "__main__":
      # 创建一个线程来运行 get_location 函数
    # location_thread = threading.Thread(target=get_location)
    # location_thread.start()  # 启动线程
    # get_location()
    # location_thread = threading.Thread(target=run_cyberbridgeinstance)
    # location_thread.start()  # 启动线程
    # testApollo(1)
    for i in range(2):
        testApollo(i)




