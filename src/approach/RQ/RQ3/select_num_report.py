import os
import random
import shutil
def select_random_xml_files(source_directory,target_directory, n):
    # 用来存储所有找到的 XML 文件
    xml_files = []

    # 遍历目录及其子目录
    for root, dirs, files in os.walk(source_directory):
        for file in files:
            # 如果文件是 XML 文件，加入到列表中
            if file.endswith('.xml'):
                xml_files.append(os.path.join(root, file))
    
    # 如果 XML 文件数量少于 n，抛出异常
    if len(xml_files) < n:
        raise ValueError(f"目录中 XML 文件数量不足 {n} 个")
    
    # 随机选择 n 个 XML 文件
    selected_files = random.sample(xml_files, n)
      # 确保目标文件夹存在，如果不存在就创建它
    os.makedirs(target_directory, exist_ok=True)
    
    # 复制选中的文件到目标文件夹
    file_index = 0
    for file in selected_files:
        # 获取文件的文件名（去除路径部分）
        file_name = os.path.basename(file)
        destination_path = os.path.join(target_directory, fr"{file_index}.xml")
        
        # 复制文件
        shutil.copy(file, destination_path)
        print(f"已复制: {file} 到 {destination_path}")
        file_index += 1

if __name__ == '__main__':
    source_folder = '/home/lsm/SFTSG_NME/src/file/report/xml'
    target_folder = '/home/lsm/SFTSG_NME/src/approach/RQ/RQ3/raw_reports'
    select_random_xml_files(source_folder, target_folder, 50)