import openai 
from openai import OpenAI
import requests
openai.api_key = 'sk-proj-cTg42NjK4cyDNYr4os0ET3BlbkFJDBOHTXc6Hnov9dwwMEw0'
# messages = [
#     {"role": "assistant", "content": "who are you"},
# ]
# response = openai.ChatCompletion.create(
#     # model="gpt-3.5-turbo-0613",
#     #model="gpt-3.5-turbo",
#     # model="gpt-3.5-turbo-0125",
#     # model="gpt-4o",
#     model = "gpt-4o-mini",
#     messages=messages,
#     # max_tokens = 100
# )
# print(response.choices[0].message.content)

headers = {
    "Content-Type": "application/json",
    "Authorization": 'Bearer ' + 'sk-proj-cTg42NjK4cyDNYr4os0ET3BlbkFJDBOHTXc6Hnov9dwwMEw0',
}

params = {
    "messages": [
        {
            "role": 'user',
            "content": 'who are you'
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
print(res)

# openai.api_key = "sk-proj-cTg42NjK4cyDNYr4os0ET3BlbkFJDBOHTXc6Hnov9dwwMEw0"
# clinet = OpenAI(api_key="sk-proj-cTg42NjK4cyDNYr4os0ET3BlbkFJDBOHTXc6Hnov9dwwMEw0")
# response = clinet.chat.completions.create(
#     model = "gpt-4o",
#     messages = [
#         {
#             "role":"user",
#             "content":"who are you"
#         }
#     ]
# )
# print(response)