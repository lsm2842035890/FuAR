import os
import json
import openai 
# from openai import OpenAI
import requests
import xml.etree.ElementTree as ET
openai.api_key = 'sk-proj-cTg42NjK4cyDNYr4os0ET3BlbkFJDBOHTXc6Hnov9dwwMEw0'
import time
def processResult(result):
    start = result.index("{")
    end = result.rindex("}")
    return result[start:end + 1]


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


def getApiKey():
    return openai.api_key

def preprocessReport(report,model_type):
    prompt = "You should help me process a car accident description. You should analyse each sentence in the description. Once you found a sentence that contains impact actions, then drop all the sentences after." + \
             "Output the processed description." + \
             "The accident description is : " + report

    headers = {
        "Authorization": 'Bearer ' + openai.api_key,
    }

    params = {
        "messages": [
            {
                "role": 'user',
                "content": prompt
            }
        ],
        "model": model_type
    }

    response = requests.post(
        "https://api.openai.com/v1/chat/completions",
        headers=headers,
        json=params,
        stream=False
    )

    res = response.json()
    print(res)
    return res['choices'][0]['message']['content']


def getAnalysisResult(report,model_type):
    headers = {
        "Authorization": 'Bearer ' + openai.api_key,
    }

    params = {
        "messages": [
            {
                "role": 'user',
                "content": readExtractionPrompt() + report
            }
        ],
        "model": model_type
    }

    response = requests.post(
        "https://api.openai.com/v1/chat/completions",
        headers=headers,
        json=params,
        stream=False
    )

    res = response.json()
    return res['choices'][0]['message']['content']

def gptProcessReport_35(rootPath,targetPath,model_type):
    start_time = time.time()
    # rootPath = "/home/lsm/SFTSG_NME/src/file/report"

    # 遍历指定目录，获取所有 .txt 文件的路径
    xml_files = []
    for dirpath, dirnames, filenames in os.walk(rootPath):
        for filename in filenames:
            if filename.endswith('.xml'):
                xml_files.append(os.path.join(dirpath, filename))
    # print(txt_files)          
    for xml_file in xml_files:
        tree = ET.parse(xml_file)
        root = tree.getroot()
        case_id = root.attrib['CaseID']
        summary = root.find('.//SUMMARY')  # 使用 find() 查找 SUMMARY 标签
        # 获取 SUMMARY 标签的文本内容
        description = summary.text.strip() if summary is not None else "No summary found"
        print(case_id)
        # print(description)

        # f = open(txt_file, "r")
        # lines = f.readlines()
        # f.close()

        # reportContent = ""
        # for line in description.split('\n'):
        #     reportContent += line
        # print(reportContent)
        prompt = "You should help me process a car accident description. You should analyse each sentence in the description. Once you found a sentence that contains impact actions, then drop all the sentences after." + \
             "Output the processed description." + \
             "The accident description is : " + description
        message = [
            {"role": "user", "content": prompt}
        ]
        response = openai.ChatCompletion.create(
            model = 'gpt-3.5-turbo',
            messages = message,
        )
        print(response)
        message_1 = [
            {"role": "user", "content": readExtractionPrompt() + response.choices[0].message.content}
        ]
        response_1 = openai.ChatCompletion.create(
            model = 'gpt-3.5-turbo',
            messages = message_1,
        )
        pres = processResult(response_1.choices[0].message.content)
        # print(pres)
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
            f = open(filepath, "w")
            f.write(pres)
        #     f.close() 
    end_time = time.time()
    print("Time used:", end_time - start_time)

def gptProcessReport_4(rootPath,targetPath,model_type):
    start_time = time.time()
    # rootPath = "/home/lsm/SFTSG_NME/src/file/report"

    # 遍历指定目录，获取所有 .txt 文件的路径
    xml_files = []
    for dirpath, dirnames, filenames in os.walk(rootPath):
        for filename in filenames:
            if filename.endswith('.xml'):
                xml_files.append(os.path.join(dirpath, filename))
    # print(txt_files)          
    for xml_file in xml_files:
        tree = ET.parse(xml_file)
        root = tree.getroot()
        case_id = root.attrib['CaseID']
        summary = root.find('.//SUMMARY')  # 使用 find() 查找 SUMMARY 标签
        # 获取 SUMMARY 标签的文本内容
        description = summary.text.strip() if summary is not None else "No summary found"
        print(case_id)
        # print(description)

        # f = open(txt_file, "r")
        # lines = f.readlines()
        # f.close()

        # reportContent = ""
        # for line in description.split('\n'):
        #     reportContent += line
        # print(reportContent)
        prompt = "You should help me process a car accident description. You should analyse each sentence in the description. Once you found a sentence that contains impact actions, then drop all the sentences after." + \
             "Output the processed description." + \
             "The accident description is : " + description
        message = [
            {"role": "user", "content": prompt}
        ]
        response = openai.ChatCompletion.create(
            model = 'gpt-4-turbo',
            messages = message,
        )
        print(response)
        message_1 = [
            {"role": "user", "content": readExtractionPrompt() + response.choices[0].message.content}
        ]
        response_1 = openai.ChatCompletion.create(
            model = 'gpt-4-turbo',
            messages = message_1,
        )
        pres = processResult(response_1.choices[0].message.content)
        # print(pres)
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
            f = open(filepath, "w")
            f.write(pres)
        #     f.close() 
    end_time = time.time()
    print("Time used:", end_time - start_time)

def gptProcessReport(rootPath,targetPath,model_type):
    start_time = time.time()
    # rootPath = "/home/lsm/SFTSG_NME/src/file/report"

    # 遍历指定目录，获取所有 .txt 文件的路径
    xml_files = []
    for dirpath, dirnames, filenames in os.walk(rootPath):
        for filename in filenames:
            if filename.endswith('.xml'):
                xml_files.append(os.path.join(dirpath, filename))
    # print(txt_files)          
    for xml_file in xml_files:
        tree = ET.parse(xml_file)
        root = tree.getroot()
        case_id = root.attrib['CaseID']
        summary = root.find('.//SUMMARY')  # 使用 find() 查找 SUMMARY 标签
        # 获取 SUMMARY 标签的文本内容
        description = summary.text.strip() if summary is not None else "No summary found"
        print(case_id)
        # print(description)

        # f = open(txt_file, "r")
        # lines = f.readlines()
        # f.close()

        # reportContent = ""
        # for line in description.split('\n'):
        #     reportContent += line
        # print(reportContent)
        ppr = preprocessReport(description,model_type)
        res = getAnalysisResult(ppr,model_type)
        pres = processResult(res)
        # print(pres)
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
            f = open(filepath, "w")
            f.write(pres)
        #     f.close() 
    end_time = time.time()
    print("Time used:", end_time - start_time)

def test():
    f = open('/home/lsm/SFTSG_NME/src/file/report/txt/crossing/0.txt', "r")
    lines = f.readlines()
    f.close()

    reportContent = ""
    for line in lines:
        reportContent += line
    print(reportContent)

# gptProcessReport('/home/lsm/SFTSG_NME/src/approach/RQ/RQ3/raw_reports','/home/lsm/SFTSG_NME/src/approach/RQ/RQ3/gpt_35_answer','gpt-3.5')
# test()
gptProcessReport_4('/home/lsm/SFTSG_NME/src/approach/RQ/RQ3/raw_reports','/home/lsm/SFTSG_NME/src/approach/RQ/RQ3/gpt_4_answer','gpt-3.5')