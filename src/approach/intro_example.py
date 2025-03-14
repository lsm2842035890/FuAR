import math
import time
import random
from lgsvl import Transform, Vector
from lgsvl.dreamview import CoordType
from datetime import datetime
from environs import Env
import lgsvl
def introduction_example():
    env = Env()
    SIMULATOR_HOST = env.str("LGSVL__SIMULATOR_HOST", "127.0.0.1")
    SIMULATOR_PORT = env.int("LGSVL__SIMULATOR_PORT", 8181)
    BRIDGE_HOST = env.str("LGSVL__AUTOPILOT_0_HOST", "127.0.0.1")
    BRIDGE_PORT = env.int("LGSVL__AUTOPILOT_0_PORT", 9090)

    LGSVL__AUTOPILOT_HD_MAP = env.str("LGSVL__AUTOPILOT_HD_MAP", "san_francisco")
    LGSVL__AUTOPILOT_0_VEHICLE_CONFIG = env.str("LGSVL__AUTOPILOT_0_VEHICLE_CONFIG", 'Lincoln2017MKZ_LGSVL')
    LGSVL__SIMULATION_DURATION_SECS = 50

    vehicle_conf = env.str("LGSVL__VEHICLE_0", lgsvl.wise.DefaultAssets.ego_lincoln2017mkz_apollo6_modular)
    scene_name = env.str("LGSVL__MAP", "12da60a7-2fc9-474d-a62a-5cc08cb97fe8")
    sim = lgsvl.Simulator(SIMULATOR_HOST, SIMULATOR_PORT)

    if sim.current_scene == scene_name:
        sim.reset()
    else:
        sim.load(scene_name)

    sim.set_date_time(datetime(2022, 6, 22, 11, 0, 0, 0), True)
    ######
    # print(sim.map_from_gps(northing=4134412.25,easting=592656,altitude=10))
    # print(sim.map_from_gps(northing=4134409,easting=592658.1,altitude=10))
    # print(sim.map_from_gps(northing=4134420.8,easting=592670.7,altitude=10))
    # print(sim.map_from_gps(northing=4134430.7,easting=592673.7,altitude=10))
    # print(sim.map_from_gps(northing=4134455.9,easting=592698,altitude=10))
    ######


    temp_pos = lgsvl.Vector(66.75,10,-64)
    ego_state = lgsvl.AgentState()
    ego_state.transform = Transform(position=lgsvl.Vector(77.8,10,-75.8),rotation=sim.map_point_on_lane(temp_pos).rotation)
    ego = sim.add_agent(vehicle_conf, lgsvl.AgentType.EGO, ego_state)
    ego.connect_bridge(BRIDGE_HOST, BRIDGE_PORT)

    ####
    temp_pos = lgsvl.Vector(-28.7000064849854, 10, 16.1999969482422)
    state = lgsvl.AgentState()
    state.transform = Transform(position=lgsvl.Vector(23.1, 10, -22),rotation=sim.map_point_on_lane(temp_pos).rotation)
    npc = sim.add_agent("SUV", lgsvl.AgentType.NPC,state)
    waypoints = [
        lgsvl.DriveWaypoint(position=lgsvl.Vector(23.1, 10, -22),speed=1.5,angle=sim.map_point_on_lane(temp_pos).rotation),
        lgsvl.DriveWaypoint(position=lgsvl.Vector(48.3, 10, -46.3),speed=1.5,angle=lgsvl.Vector(0,math.degrees(math.atan2(25.2,-24.3)))),
    ]
    npc.follow(waypoints)
    ####
    ####
    temp_pos1 = lgsvl.Vector(70,10,-61.9)
    state1 = lgsvl.AgentState()
    state1.transform = Transform(position=lgsvl.Vector(94,10,-87.1),rotation=sim.map_point_on_lane(temp_pos1).rotation)
    npc1 = sim.add_agent("SUV", lgsvl.AgentType.NPC,state1)
    waypoints1 = [
        lgsvl.DriveWaypoint(position=lgsvl.Vector(94,10,-87.1),speed=7,angle=sim.map_point_on_lane(temp_pos1).rotation),
        lgsvl.DriveWaypoint(position=lgsvl.Vector(70,10,-61.9),speed=8,angle=lgsvl.Vector(0,math.degrees(math.atan2(-12,12.6)))),
        lgsvl.DriveWaypoint(position=lgsvl.Vector(58,10,-49.3),speed=8,angle=lgsvl.Vector(0,math.degrees(math.atan2(-12,12.6)))),
        lgsvl.DriveWaypoint(position=lgsvl.Vector(48.3,10,-46.3),speed=8,angle=lgsvl.Vector(0,math.degrees(math.atan2(-9.8,3)))),
    ]
    npc1.follow(waypoints1)
    ####

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
    dv.setup_apollo(592656, 4134412.25, default_modules)
    dv.set_destination(592688, 4134442.5,coord_type=CoordType.Northing)
    sim.run(LGSVL__SIMULATION_DURATION_SECS)

if __name__ == "__main__":
    introduction_example()