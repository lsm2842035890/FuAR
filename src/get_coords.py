import math
import time
import random
from lgsvl import Transform, Vector
from datetime import datetime
from environs import Env
from lgsvl.dreamview import CoordType
import lgsvl

def sim_update_callback(agent):
    print(agent.state)

def get_coords():
    env = Env()
    SIMULATOR_HOST = env.str("LGSVL__SIMULATOR_HOST", "127.0.0.1")
    SIMULATOR_PORT = env.int("LGSVL__SIMULATOR_PORT", 8181)

    vehicle_conf = env.str("LGSVL__VEHICLE_0", lgsvl.wise.DefaultAssets.ego_lincoln2017mkz_apollo6_modular)
    scene_name = env.str("LGSVL__MAP", lgsvl.wise.DefaultAssets.map_sanfrancisco_correct)
    sim = lgsvl.Simulator(SIMULATOR_HOST, SIMULATOR_PORT)

    #################
    coor = sim.map_from_gps(northing=4134776,easting=592725,altitude=10)
    print(coor)
    ################


    if sim.current_scene == scene_name:
        sim.reset()
    else:
        sim.load(scene_name)

    spawns = sim.get_spawn()
    state2 = lgsvl.AgentState()
    state2.transform = spawns[0]
    print(spawns[0])
    signal = sim.get_controllables()[0]
    print(signal.current_state)
    print(signal.default_control_policy)
    print(signal.control_policy)
    control_policy = "trigger=1000;green=9999;yellow=0;red=0;loop"
    signal.control(control_policy)
    print(signal.control_policy)
    
    sim.set_date_time(datetime(2022, 6, 22, 11, 0, 0, 0), True)

    ego = sim.add_agent(vehicle_conf,lgsvl.AgentType.EGO,state2)
    sim.run()

def npc_coords():
    env = Env()
    SIMULATOR_HOST = env.str("LGSVL__SIMULATOR_HOST", "127.0.0.1")
    SIMULATOR_PORT = env.int("LGSVL__SIMULATOR_PORT", 8181)

    vehicle_conf = env.str("LGSVL__VEHICLE_0", lgsvl.wise.DefaultAssets.ego_lincoln2017mkz_apollo6_modular)
    scene_name = env.str("LGSVL__MAP", lgsvl.wise.DefaultAssets.map_sanfrancisco_correct)
    sim = lgsvl.Simulator(SIMULATOR_HOST, SIMULATOR_PORT)
    if sim.current_scene == scene_name:
        sim.reset()
    else:
        sim.load(scene_name)
    sim.set_date_time(datetime(2022, 6, 22, 11, 0, 0, 0), True)
    coord1 = sim.map_from_gps(northing=4134776,easting=592725,altitude=10)
    coord2 = sim.map_from_gps(northing=4134748,easting=592701,altitude=10)
    coord3 = sim.map_from_gps(northing=4134726,easting=592681,altitude=10)
    print("coord1",coord1)
    one = lgsvl.Vector(coord1.position.x,coord1.position.y,coord1.position.z)
    two = lgsvl.Vector(coord2.position.x,coord2.position.y,coord2.position.z)
    three = lgsvl.Vector(coord3.position.x,coord3.position.y,coord3.position.z)
    # print(one)
    # print(sim.map_point_on_lane(one).rotation)
    state = lgsvl.AgentState()
    state.transform = Transform(position=one,rotation=sim.map_point_on_lane(one).rotation)
    print("state.transform",state.transform)
    npc = sim.add_agent("SUV", lgsvl.AgentType.NPC,state)
    # print('aaaa')
    # waypoints = [
    #     lgsvl.DriveWaypoint(position=one,speed=15,angle=sim.map_point_on_lane(one).rotation),
    #     lgsvl.DriveWaypoint(position=two,speed=15,angle=sim.map_point_on_lane(two).rotation),
    #     lgsvl.DriveWaypoint(position=three,speed=5,angle=sim.map_point_on_lane(three).rotation)
    # ]
    waypoints = [
        lgsvl.DriveWaypoint(position=lgsvl.Vector(-296,10,4),speed=1,angle=lgsvl.Vector(0, 133.700012207031, 0)),
        lgsvl.DriveWaypoint(position=lgsvl.Vector(-269,10,-6.9),speed=15,angle=lgsvl.Vector(0,math.degrees(math.atan2(27,-10.9 )))),
        lgsvl.DriveWaypoint(position=lgsvl.Vector(-248,10,-40),speed=5,angle=lgsvl.Vector(0,math.degrees(math.atan2(21,-33.1))))
    ]
    print(waypoints[0].position,waypoints[0].angle)
    print(waypoints[1].position,waypoints[1].angle)
    print(waypoints[2].position,waypoints[2].angle)
    npc.follow(waypoints,loop=True)
    cameraPosition = Vector(-297, 100, 5)
    cameraRotation = Vector(90, 0, 0)
    camera = Transform(cameraPosition, cameraRotation)
    sim.set_sim_camera(camera)
    sim.run()

def apollo_and_npc():
    env = Env()

    SIMULATOR_HOST = env.str("LGSVL__SIMULATOR_HOST", "127.0.0.1")
    SIMULATOR_PORT = env.int("LGSVL__SIMULATOR_PORT", 8181)
    BRIDGE_HOST = env.str("LGSVL__AUTOPILOT_0_HOST", "127.0.0.1")
    BRIDGE_PORT = env.int("LGSVL__AUTOPILOT_0_PORT", 9090)

    LGSVL__AUTOPILOT_HD_MAP = env.str("LGSVL__AUTOPILOT_HD_MAP", "san_francisco")
    LGSVL__AUTOPILOT_0_VEHICLE_CONFIG = env.str("LGSVL__AUTOPILOT_0_VEHICLE_CONFIG", 'Lincoln2017MKZ_LGSVL')
    LGSVL__SIMULATION_DURATION_SECS = 1500

    vehicle_conf = env.str("LGSVL__VEHICLE_0", lgsvl.wise.DefaultAssets.ego_lincoln2017mkz_apollo6_modular)
    scene_name = env.str("LGSVL__MAP", lgsvl.wise.DefaultAssets.map_sanfrancisco_correct)
    sim = lgsvl.Simulator(SIMULATOR_HOST, SIMULATOR_PORT)

    if sim.current_scene == scene_name:
        sim.reset()
    else:
        sim.load(scene_name)

    sim.set_date_time(datetime(2022, 6, 22, 11, 0, 0, 0), True)
    #####
    signal = sim.get_controllables()[0]
    control_policy = "trigger=1000;green=9999;yellow=0;red=0;loop"
    signal.control(control_policy)
    #####
    ####
    coord = sim.map_from_gps(northing=4134776,easting=592727,altitude=10)
    coord1 = sim.map_from_gps(northing=4134767,easting=592716,altitude=10)
    coord2 = sim.map_from_gps(northing=4134748,easting=592701,altitude=10)
    coord3 = sim.map_from_gps(northing=4134726,easting=592681,altitude=10)
    print(coord1)
    one = lgsvl.Vector(coord1.position.x,coord1.position.y,coord1.position.z)
    two = lgsvl.Vector(coord2.position.x,coord2.position.y,coord2.position.z)
    three = lgsvl.Vector(coord3.position.x,coord3.position.y,coord3.position.z)
    print(one)
    print(sim.map_point_on_lane(one).rotation)
    state = lgsvl.AgentState()
    state.transform = Transform(position=one,rotation=sim.map_point_on_lane(one).rotation)
    npc = sim.add_agent("SUV", lgsvl.AgentType.NPC,state)
    print('aaaa')
    waypoints = [
        lgsvl.DriveWaypoint(position=one,speed=5,angle=sim.map_point_on_lane(one).rotation),
        lgsvl.DriveWaypoint(position=two,speed=5,angle=sim.map_point_on_lane(two).rotation),
        lgsvl.DriveWaypoint(position=three,speed=5,angle=sim.map_point_on_lane(three).rotation)
    ]
    print(waypoints)
    npc.follow(waypoints,loop=False)
    ###

    state_ego = lgsvl.AgentState()
    # state2.transform = spawns[0]
    state_ego.transform = sim.map_point_on_lane(lgsvl.Vector(coord.position.x,coord.position.y,coord.position.z))
    print(state_ego)

    print("Loading vehicle {}...".format(vehicle_conf))
    ego = sim.add_agent(vehicle_conf, lgsvl.AgentType.EGO, state_ego)
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
    dv.setup_apollo(592681, 4134726, default_modules)
    dv.set_destination(592623,4134667,coord_type=CoordType.Northing)
    sim.run(1500)

def get_map_coords_rotation():
    env = Env()
    SIMULATOR_HOST = env.str("LGSVL__SIMULATOR_HOST", "127.0.0.1")
    SIMULATOR_PORT = env.int("LGSVL__SIMULATOR_PORT", 8181)
    vehicle_conf = env.str("LGSVL__VEHICLE_0", lgsvl.wise.DefaultAssets.ego_lincoln2017mkz_apollo6_modular)
    scene_name = env.str("LGSVL__MAP", lgsvl.wise.DefaultAssets.map_sanfrancisco_correct)
    sim = lgsvl.Simulator(SIMULATOR_HOST, SIMULATOR_PORT)
    if sim.current_scene == scene_name:
        sim.reset()
    else:
        sim.load(scene_name)
    #########
    coor = sim.map_from_gps(northing=4134397,easting=592645,altitude=10)
    print(coor)
    one = lgsvl.Vector(coor.position.x,coor.position.y,coor.position.z)
    state = lgsvl.AgentState()
    state.transform = Transform(position=one,rotation=sim.map_point_on_lane(one).rotation)
    print(state.transform.rotation)
    ego = sim.add_agent(vehicle_conf, lgsvl.AgentType.EGO,state)
    #######
    sim.set_date_time(datetime(2022, 6, 22, 5, 25, 0, 0), True)
    cameraPosition = Vector(82, 100, -74)
    cameraRotation = Vector(90, 0, 0)
    camera = Transform(cameraPosition, cameraRotation)
    sim.set_sim_camera(camera)
    sim.run()

# get_coords()
# npc_coords()
apollo_and_npc()
# get_map_coords_rotation()