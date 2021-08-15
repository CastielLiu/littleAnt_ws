#! /usr/bin/env python
# coding=utf-8
import json
from functools import partial
import requests

url = 'http://192.168.0.173:8000/helloworld/a_upload/'

files = {'myfiles': open('requirements.txt', 'rb'), 'myfiles2': open('aa/aa.txt', 'rb')}

# 单个文件
# files = {"file": open('admin.py', 'rb')}

# 多个文件
files = [
            ("myfiles", open('requirements.txt', 'rb')),
            ("myfiles", open('test.py', 'rb')),
            ("myfiles2", open('requirements.txt', 'rb')),
            ("myfiles2", open('test.py', 'rb'))
        ]


r = requests.post(url, files=files)

print(r.text)