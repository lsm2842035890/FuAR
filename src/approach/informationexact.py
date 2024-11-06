import os
import json
import openai 
from openai import OpenAI
import requests
import xml.etree.ElementTree as ET
openai.api_key = 'sk-proj-cTg42NjK4cyDNYr4os0ET3BlbkFJDBOHTXc6Hnov9dwwMEw0'

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

def preprocessReport(report):
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
        "model": 'gpt-4o'
    }

    response = requests.post(
        "https://api.openai.com/v1/chat/completions",
        headers=headers,
        json=params,
        stream=False
    )

    res = response.json()
    return res['choices'][0]['message']['content']


def getAnalysisResult(report):
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
        "model": 'gpt-4o'
    }

    response = requests.post(
        "https://api.openai.com/v1/chat/completions",
        headers=headers,
        json=params,
        stream=False
    )

    res = response.json()
    return res['choices'][0]['message']['content']


def gptProcessReport():
    rootPath = "/home/lsm/SFTSG_NME/src/file/report"

    # 遍历指定目录，获取所有 .txt 文件的路径
    txt_files = []
    for dirpath, dirnames, filenames in os.walk(rootPath):
        for filename in filenames:
            if filename.endswith('.txt'):
                txt_files.append(os.path.join(dirpath, filename))
    # print(txt_files)          
    for txt_file in txt_files:
        dir_path = os.path.dirname(txt_file)
        file_name_with_extension = os.path.basename(txt_file)
        file_name_without_extension = os.path.splitext(file_name_with_extension)[0]
        second_last_dir = os.path.basename(dir_path)
        last_file = file_name_without_extension
        # print(second_last_dir, last_file, file_name_with_extension)
        # get report ID
        xml_path = f"/home/lsm/SFTSG_NME/src/file/report/xml/{second_last_dir}/{file_name_without_extension}.xml"
        tree = ET.parse(xml_path)
        root = tree.getroot()
        case_id = root.attrib['CaseID']

        f = open(txt_file, "r")
        lines = f.readlines()
        f.close()

        reportContent = ""
        for line in lines:
            reportContent += line

        ppr = preprocessReport(reportContent)
        res = getAnalysisResult(ppr)
        pres = processResult(res)
        # print(pres)
        try:
            data = json.loads(pres)
            # add key ID and description
            data['ID'] = case_id
            data['description'] = reportContent
            # with open(f'/home/lsm/SFTSG_NME/src/approach/information_ex_results/{case_id}.json', 'w') as json_file:
            #     json.dump(data, json_file, indent=4)
            with open(f'/home/lsm/SFTSG_NME/src/approach/information_ex_results_pro/{case_id}.json', 'w') as json_file:
                json.dump(data, json_file, indent=4)
        except:
            # filepath = f"/home/lsm/SFTSG_NME/src/approach/information_ex_results/{case_id}.txt"
            filepath = f"/home/lsm/SFTSG_NME/src/approach/information_ex_results_pro/{case_id}.txt"
            f = open(filepath, "w")
            f.write(pres)
            f.close() 
        
        
        
        # fileName = os.path.join(rootPath, "BigModelRacer-latest/src/file/result.txt")
        # f = open(fileName, "w")
        # f.write(pres)
        # f.close()

if __name__ == '__main__':
    gptProcessReport()    