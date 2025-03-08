import os
from openai import OpenAI
import time
import xml.etree.ElementTree as ET
import json
# client = OpenAI(
#     # 若没有配置环境变量，请用百炼API Key将下行替换为：api_key="sk-xxx",
#     api_key='sk-54af485fa92a42adbb24297cd76a8657',  # 如何获取API Key：https://help.aliyun.com/zh/model-studio/developer-reference/get-api-key
#     base_url="https://dashscope.aliyuncs.com/compatible-mode/v1"
# )

# completion = client.chat.completions.create(
#     model="deepseek-r1",  # 此处以 deepseek-r1 为例，可按需更换模型名称。
#     messages=[
#         {'role': 'user', 'content': 'hello'}
#     ]
# )
# # # 通过reasoning_content字段打印思考过程
# # print("思考过程：")
# # print(completion.choices[0].message.reasoning_content)

# # 通过content字段打印最终答案
# print("最终答案：")
# print(completion.choices[0].message.content)

def preprocessReport(report,model_type):
    client = OpenAI(
        api_key='sk-54af485fa92a42adbb24297cd76a8657', 
        base_url="https://dashscope.aliyuncs.com/compatible-mode/v1"
    )

    content = "You should help me process a car accident description. You should analyse each sentence in the description. Once you found a sentence that contains impact actions, then drop all the sentences after." + \
             "Output the processed description." + \
             "The accident description is : " + report

    completion = client.chat.completions.create(
        model=model_type,  # 此处以 deepseek-r1 为例，可按需更换模型名称。
        messages=[
            {'role': 'user', 'content': content}
        ]
    )
    print(completion.choices[0].message.content)
    return completion.choices[0].message.content

def readExtractionPrompt():
    # fileName = "/home/lsm/SFTSG_NME/src/file/extractionPrompt.txt"
    fileName = "/home/lsm/SFTSG_NME/src/file/extractionPromptPro.txt"
    f = open(fileName, "r")
    lines = f.readlines()
    f.close()

    prompt = ""
    for line in lines:
        prompt += line    
    return prompt

def getAnalysisResult(report,model_type):
    client = OpenAI(
        api_key='sk-54af485fa92a42adbb24297cd76a8657', 
        base_url="https://dashscope.aliyuncs.com/compatible-mode/v1"
    )   

    content = readExtractionPrompt() + report

    completion = client.chat.completions.create(
        model=model_type,  # 此处以 deepseek-r1 为例，可按需更换模型名称。
        messages=[
            {'role': 'user', 'content': content}
        ]
    )
    return completion.choices[0].message.content

def processResult(result):
    start = result.index("{")
    end = result.rindex("}")
    return result[start:end + 1]

def deepseek_process_report(rootPath,targetPath,model_type):
    start_time = time.time()
    # rootPath = "/home/lsm/SFTSG_NME/src/file/report"

    # 遍历指定目录，获取所有 .txt 文件的路径
    xml_files = []
    for dirpath, dirnames, filenames in os.walk(rootPath):
        for filename in filenames:
            if filename.endswith('.xml'):
                xml_files.append(os.path.join(dirpath, filename))
    
    for xml_file in xml_files:
        tree = ET.parse(xml_file)
        root = tree.getroot()
        case_id = root.attrib['CaseID']
        summary = root.find('.//SUMMARY')  # 使用 find() 查找 SUMMARY 标签
        # 获取 SUMMARY 标签的文本内容
        description = summary.text.strip() if summary is not None else "No summary found"
        print(case_id)
        ppr = preprocessReport(description, model_type)
        res = getAnalysisResult(ppr, model_type)
        pres = processResult(res)
        print(pres)
        try:
            data = json.loads(pres)
            # add key ID and description
            data['ID'] = case_id
            data['description'] = description
            with open(f'{targetPath}/{case_id}.json', 'w') as json_file:
                json.dump(data, json_file, indent=4)
        except:
            # filepath = f"/home/lsm/SFTSG_NME/src/approach/information_ex_results/{case_id}.txt"
            filepath = f"{targetPath}/{case_id}.txt"
            with open(filepath, "w") as f:  # 使用 'with open' 语句自动管理文件关闭
                f.write(pres)  # 写入第一段文本并换行
                f.write("\n")  # 再换一行
                f.write("'ID': " + case_id)  # 写入第二段文本并换行")
                f.wtire("\n")
                f.write("'description': " + description)  # 写入第三段文本并换行")
    end_time = time.time()
    print("Time used:", end_time - start_time)

deepseek_process_report('/home/lsm/SFTSG_NME/src/approach/RQ/RQ3/raw_reports',"/home/lsm/SFTSG_NME/src/approach/RQ/RQ3/deepseek_r1_answer",'deepseek-v3')
