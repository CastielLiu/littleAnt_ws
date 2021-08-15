#! /usr/bin/env python
# coding=utf-8
import requests
import json


class Test:
    def __init__(self):
        self.userid = "testcar1"
        self.username = "testcar1"  # 测试车辆1号
        self.userpasswd = "testcar1"
        self.usertoken = ''

    def run(self):
        # login_url = 'http://36.155.113.13:8000/autodrive/login/'
        login_url = 'http://192.168.0.173:8000/autodrive/login/'

        session = requests.session()
        request_login = session.get(url=login_url)
        cookies = session.cookies
        print request_login.cookies
        # print request_login.headers

        # 登录http
        login_dict = dict()
        login_dict['username'] = self.username
        login_dict['userid'] = self.userid
        login_dict['password'] = self.userpasswd
        login_dict["usertype"] = 'car'
        headers = {"X-CSRFToken": cookies.get("csrftoken")}

        request_login = session.post(url=login_url, data=login_dict, headers=headers)
        try:
            data = json.loads(request_login.text)
            if data['code'] == 0:
                self.usertoken = request_login.cookies.get('token', '')
                if self.usertoken == '':
                    return False
                self.userid = request_login.cookies.get('userid', '')
            else:
                print("login http faild")
                return False
        except Exception as e:
            print(e.args)
            return False

        # 登录到ws




        print request_login.text


class Child:
    def __init__(self):
        print("child construct")

    def __del__(self):
        print("child deconstruct")


class Parent(Child):
    def __init__(self):
        # Child.__init__(self)
        print("parent construct")

    # def __del__(self):
    #     Child.__del__(self)
    #     print("parent deconstruct")

# p = Parent()

t = Test()
t.run()
