import math
import time
import random
from lgsvl import Transform, Vector
from lgsvl.dreamview import CoordType
from datetime import datetime
from environs import Env
import lgsvl
import asyncio
import websockets
import json
import requests
import threading

def testApollo(tag):
    env = Env()

    SIMULATOR_HOST = env.str("LGSVL__SIMULATOR_HOST", "127.0.0.1")
    SIMULATOR_PORT = env.int("LGSVL__SIMULATOR_PORT", 8181)
    BRIDGE_HOST = env.str("LGSVL__AUTOPILOT_0_HOST", "127.0.0.1")
    BRIDGE_PORT = env.int("LGSVL__AUTOPILOT_0_PORT", 9090)

    LGSVL__AUTOPILOT_HD_MAP = env.str("LGSVL__AUTOPILOT_HD_MAP", "san_francisco")
    LGSVL__AUTOPILOT_0_VEHICLE_CONFIG = env.str("LGSVL__AUTOPILOT_0_VEHICLE_CONFIG", 'Lincoln2017MKZ_LGSVL')
    LGSVL__SIMULATION_DURATION_SECS = 1500

    vehicle_conf = env.str("LGSVL__VEHICLE_0", lgsvl.wise.DefaultAssets.ego_lincoln2017mkz_apollo6_modular)
    scene_name = env.str("LGSVL__MAP", "12da60a7-2fc9-474d-a62a-5cc08cb97fe8")
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

async def monitor_ego_and_npcs_data():
    uri = "ws://localhost:8888/websocket"  # 替换为实际的 WebSocket 地址
    # uri = "ws://localhost:8888"
    try:
        print("[WebSocket] Attempting to connect...")
        async with websockets.connect(uri) as websocket:
            # init_msg = json.dumps({ 'type': 'HMIStatus' })     
            # init_msg = json.dumps({ 'type': 'EgoStatusRequest' })   
            init_msg = json.dumps({ 'type': 'HMIAction' })  
            websocket.send(init_msg)
            print("[WebSocket] Connected to Apollo WebSocket!")
            print("sdasad")
            while True:
                message = await websocket.recv()
                try:
                    # 尝试将接收到的消息解析为 JSON 格式
                    data = json.loads(message)
                    if data["type"] == "HMIAction":
                        print("[WebSocket] Parsed JSON:", json.dumps(data, indent=4))  # 美化打印 JSON
                except json.JSONDecodeError:
                    # 如果无法解析为 JSON 格式，打印原始字节串
                    print("[WebSocket] Error decoding JSON:", message)
            # message = websocket.recv()
            # print(message["DATA"])
    except Exception as e:
        print(f"[WebSocket] Error: {e}")

def start_websocket_thread():
    # 使用 asyncio 运行 WebSocket 监听
    asyncio.run(monitor_ego_and_npcs_data())


if __name__ == "__main__":
    websocket_thread = threading.Thread(target=start_websocket_thread)
    websocket_thread.daemon = True  # 使线程成为守护线程，主线程退出时也会退出
    websocket_thread.start()

    # 启动仿真任务
    testApollo(1)

    # 等待 WebSocket 线程结束（如果需要，也可以设置超时等）
    websocket_thread.join()