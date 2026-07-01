import requests
import json

url = "https://tukggshh8k.execute-api.us-east-1.amazonaws.com/Prod/apicall"

payload = json.dumps({
  "apiname": "get_access_token",
  "username": "your_ceresgate_useraccount",
  "password": "your_ceresgate_password"
})
headers = {
  'Content-Type': 'application/json'
}

response = requests.request("POST", url, headers=headers, data=payload)

retvalue = response.json()
access_token = None

if retvalue['statusCode'] == 200 :
    access_token = retvalue['access_token']

print(response.text)

print(access_token)

if access_token is not None:
    
    url = "https://tukggshh8k.execute-api.us-east-1.amazonaws.com/Prod/downlink"
    
    Token = 'Bearer ' + access_token

    datastring = "Hello, this is a test downlink message."
    datahex = datastring.encode("utf-8").hex()

    payload = json.dumps({"apiname": "downlink_to_ue",
               "username": "your_ceresgate_useraccount",
               "data": {"IMSI": "your_imsi", "Payload": datahex}
              })
    
    print(payload)
    print(Token)
    
    headers = {
      'Authorization': Token,
      'Content-Type': 'application/json'
    }

    response = requests.request("POST", url, headers=headers, data=payload)

    print('downlink')
    print(response.text)
