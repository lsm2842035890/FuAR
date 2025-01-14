
import os
import glob
from datetime import datetime
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from simulate_report import *
def run_cyberbridgeinstance(directory_name,name):
    global cyber
    cyber = CyberBridgeInstance()
    cyber.register(None,[0,0],[],"san_francisco",True,directory_name,name)

def on_exit():
    if cyber:
        cyber.save_pose_perception_json()


def read_completed_files(time_file_path):
    completed_files = set()  # 用set避免重复
    if os.path.exists(time_file_path):
        with open(time_file_path, 'r') as f:
            lines = f.readlines()
            # print(lines[1],lines[2])
            for i in range(0, len(lines), 3): 
                # print(i)
                if i + 2 <= len(lines)-1:  
                    file_path_line = lines[i].strip()
                    start_line = lines[i + 1].strip()
                    end_line = lines[i + 2].strip()
                    # print(file_path_line,start_line,end_line)
                    # print('/n')

                    # 确保start和end存在且格式正确
                    if start_line.startswith("start") and end_line.startswith("end"):
                        # print(f"Completed file: {file_path_line}")  # 调试输出
                        completed_files.add(file_path_line)
                else:
                    # print(lines[i])
                    # print(lines[i+1])
                    lines.pop(i+1)
                    lines.pop(i)
                    with open(time_file_path, 'w') as f:
                        f.writelines(lines)
                    break
    return completed_files

# directory_path = "/home/lsm/SFTSG_NME/src/approach/combined_reports_strict_rule/n_merged"
directory_path = "/home/lsm/SFTSG_NME/src/approach/combined_reports_strict_rule/two_merged"
json_files = glob.glob(os.path.join(directory_path, '*.json'))
completed_files = read_completed_files("/home/lsm/SFTSG_NME/src/approach/RQ/RQ1/time.txt")
for file_path in json_files:
    if file_path in completed_files:
        print(f"Skipping {file_path}, already completed.") 
        continue
    with open("/home/lsm/SFTSG_NME/src/approach/RQ/RQ1/time.txt","a") as f:
        f.write(file_path)
        f.write('\n')
        f.write('start:'+ datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
        f.write('\n')
    directory_name = os.path.basename(os.path.dirname(file_path))
    file_name_with_extension = os.path.basename(file_path)
    file_name, file_extension = os.path.splitext(file_name_with_extension)
    # print(directory_name,file_name)
    # cyber = None
    # atexit.register(on_exit)  # 注册退出时的清理回调
    location_thread = threading.Thread(target=run_cyberbridgeinstance,args=(directory_name,file_name))
    location_thread.daemon = True
    location_thread.start()  # 启动线程
    simulate_report(file_path,True)
    if cyber:
        cyber.save_pose_perception_json()  # 执行保存操作
        #cyber = None
    with open("/home/lsm/SFTSG_NME/src/approach/RQ/RQ1/time.txt","a") as f:
        f.write('end:'+ datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
        f.write('\n')

# if __name__ == '__main__':
#     print(read_completed_files("/home/lsm/SFTSG_NME/src/approach/RQ/RQ1/time.txt"))
