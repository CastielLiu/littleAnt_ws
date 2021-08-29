#! /usr/bin/env python
# coding=utf-8

# from websocket import create_connection
import shutil
import threading
import os
import websocket
import requests
import time
import rospy
import json
from functools import partial
from requests import ConnectionError
from av_task import RequestAvTask
from scripts import settings

''' 
自动驾驶与web服务器进行数据交互
必须安装websocket-client模块  pip install websocket-client
'''


class WebsocketThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)  # 初始化父类Thread
        self._runing = True

    def stop(self):
        self._runing = False

    def run(self):
        while self._runing:
            pass


# Json格式应答解析类
class JsonResponse:
    def __init__(self, text):
        self.code = -1
        self.data = {}
        self.msg = ""
        self.type = ""
        try:
            text_dict = json.loads(text)
        except Exception as e:
            print("response parse faild! %s" % text)
        self.code = text_dict.get("code", self.code)
        self.data = text_dict.get("data", self.data)
        self.msg = text_dict.get("msg", self.msg)
        self.type = text_dict.get("type", self.type)

    def valid(self):
        return self.code != -1

    def __str__(self):
        return "type: %s/code: %d/msg: %s/data: %s" % (self.type, self.code, self.msg, self.data)


# 自动驾驶客户端
class CloudClient:
    def __init__(self):
        # self.root_url = "36.155.113.13:8000/"
        self.root_url = "127.0.0.1:8000/"
        self.http_root = "http://" + self.root_url
        # 核心数据url 用于传输车辆状态与控制指令
        self.ws_core_url = "ws://" + self.root_url + "ws/autodrive/car/core/"
        self.http_url = "http://" + self.root_url + "autodrive/"

        # 视频数据url
        self.ws_video_url = "ws://" + self.root_url + "ws/autodrive/car/video/"

        self.session = requests.session()
        self.__core_ws = None
        self.ws_login_cv = threading.Condition()
        self.ws_loggedin = False
        self.http_loggedin = False

        self.userid = "testcar1"
        self.username = "testcar1"
        self.userpasswd = "testcar1"
        self.usertoken = ''

        self.csrf_header = {}
        self.running = True

        self.request_av_task = RequestAvTask()  # 请求自动驾驶任务

    def check_login(self):
        if self.ws_loggedin and self.http_loggedin:
            return True
        return False

    def login(self):
        # http登录 ws登录
        if not self.__login_http() or not self.__login_core_ws():
            return False
        return True

    def logout(self):
        self.__logout_http()
        self.__core_ws.close()

    def sendCoreData(self, data):
        try:
            self.__core_ws.send(data)
        except Exception as e:
            print(e)
            return False
        return True

    # 获取导航路径列表
    def get_navpathlist(self):
        # 不传递group时，默认获取用户组 {"group": xxx}
        data = {"type": "req_path_list", "data": {}}
        try:
            request_pathlist = self.session.post(url=self.http_url, json=data)
        except Exception as e:
            return None
        if request_pathlist.status_code != 200:
            return None
        print(request_pathlist.text)
        response = JsonResponse(request_pathlist.text)
        print(response)
        if not response.valid():
            return None

        return response.data["path_list"]

    # 获取并下载导航路径文件
    # 先获取路径文件的urls, 再利用urls进行文件下载
    def download_navpathfile(self, path_dir, pathid):
        try:
            data = {"type": "req_path_files", "data": {"pathid": pathid}}
            # self.session.post(data=data, json=json)
            # data=dict -> 'key1=val1&key2=val2' 与html表单相同
            # json=dict -> {"key1": val1, "key2": val2}
            request_navpath = self.session.post(url=self.http_url, json=data, headers=self.csrf_header)
        except Exception as e:
            print("download_navpathfile error %s" % e)
            return False

        if request_navpath.status_code != 200:
            return False

        try:
            response_dict = json.loads(request_navpath.text)
        except Exception as e:
            print(e)
            return False
        code = response_dict.get('code', -1)
        msg = response_dict.get('msg', "")
        if code != 0:
            print("获取路径失败: %s" % msg)
            return False
        data = response_dict.get('data', {})
        path_urls = data.get('path_urls', [])
        # path_name = data.get('path_name')
        # if len(path_urls) == 0 or path_name is None:
        #     return False

        # 遍历urls进行文件下载
        for path_url in path_urls:
            _, file_name = os.path.split(path_url)
            file_name = os.path.join(path_dir, file_name)
            path_url = self.http_root[:-1] + path_url
            try:
                # stream模式下载文件, 防止大文件直接加载进内存导致内存不足
                path_request = self.session.get(url=path_url, stream=False)
                # print(path_request.status_code)
                if path_request.status_code != 200:
                    print("Not found: ", path_url)
                    return False

                print("downloading %s to %s" % (path_url, file_name))
                with open(file_name, "wb") as f:
                    for chunk in path_request.iter_content():
                        if chunk:
                            f.write(chunk)

                print(file_name, "saved")
            except Exception as e:
                print(e)
                return False
        return True

    # 停止数据传输, 退出登录
    def stop(self):
        print("cloud client stop.")
        self.running = False
        self.logout()

    def __logout_http(self):
        http_logout_url = self.http_url + "logout/"
        try:
            request_logout = self.session.get(url=http_logout_url)
            if request_logout.status_code == 200:
                res_dict = json.loads(request_logout.text)
                if res_dict['code'] == 0:
                    self.http_loggedin = False
                    return True
        except Exception as e:
            pass

        return False

    # http登录
    def __login_http(self):
        http_login_url = self.http_url + "login/"
        while self.running:
            try:
                request_login = self.session.get(url=http_login_url)  # get登录页面以获取cookies
            except ConnectionError as e:
                print("连接服务器超时，正在重新连接")
                time.sleep(1.0)
                continue

            if request_login.status_code != 200:
                print("连接服务器失败")
                continue
            break

        # http登录验证
        login_dict = dict()
        login_dict['username'] = self.username
        login_dict['userid'] = self.userid
        login_dict['password'] = self.userpasswd
        login_dict["usertype"] = 'car'
        self.csrf_header = {"X-CSRFToken": self.session.cookies.get("csrftoken")}
        # 给会话添加csrf防御头
        self.session.headers.setdefault("X-CSRFToken", self.session.cookies.get("csrftoken"))

        try:
            # 会话请求头已经添加csrf, 无需手动添加
            # request_login = self.session.post(url=http_login_url, data=login_dict, headers=self.csrf_header)
            request_login = self.session.post(url=http_login_url, data=login_dict)
            data = json.loads(request_login.text)
            if data['code'] != 0:
                print("login http faild", data.msg)
                return False
            request_main = self.session.get(url=self.http_url)  # get主页以获取token

            self.usertoken = request_main.cookies.get('token')
            self.userid = request_main.cookies.get('userid')
            if self.usertoken is None or self.userid is None:
                print("No usertoken or userid in cookies.")
                return False
        except Exception as e:
            print(e.args)
            return False
        self.http_loggedin = True
        return True

    # 多线程用于websocket阻塞
    def __core_ws_run_forever(self):
        self.__core_ws.run_forever()

    # websocket登录
    def __login_core_ws(self):  # 必须启用多线程
        self.__core_ws = websocket.WebSocketApp(self.ws_core_url,
                                                on_message=self.__onWebSocketMessage,
                                                on_error=self.__onWebSocketError,
                                                on_close=self.__onWebSocketClose,
                                                on_open=self.__onWebSocketOpen)

        # 在新线程中保持websocket运行
        threading.Thread(target=self.__core_ws_run_forever).start()

        # 等待websocket登录成功
        # wait_cnt = 0
        # while not self.ws_loggedin and self.running:
        #     wait_cnt = wait_cnt + 1
        #     if wait_cnt > 3:  # 等待超时
        #         print("登录websocket超时")
        #         break
        #     print("%d: websocket logging in..." % wait_cnt)
        #     time.sleep(1.0)

        # 使用多线程条件变量唤醒
        self.ws_login_cv.acquire()
        self.ws_login_cv.wait(3.0)
        self.ws_login_cv.release()

        if self.ws_loggedin:
            print("core websocket login ok.")
        return self.ws_loggedin

    def __onWebSocketOpen(self, ws):
        # ws启动后进行服务器登录验证
        userinfo = dict()
        userinfo["username"] = self.username
        userinfo["userid"] = self.userid
        userinfo["password"] = self.userpasswd
        login_data = dict()
        login_data['type'] = "req_login"
        login_data['data'] = userinfo
        # print(json.dumps(login_data, encoding='utf-8'))
        self.__core_ws.send(json.dumps(login_data, encoding='utf-8'))
        # print("### open ###")

    # websocket消息回调函数, 上报信息/请求信息/应答信息
    def __onWebSocketMessage(self, ws, message):
        print("ws receive: %s" % message)
        try:
            data_dict = json.loads(message, encoding='utf-8')
            msgtype = data_dict.get('type')
            code = data_dict.get('code', -1)
            data = data_dict.get('data')
        except Exception as e:
            return

        # 登录是否成功
        if not self.ws_loggedin and msgtype == "res_login":
            self.ws_login_cv.acquire()
            if 0 == code:  # 登录成功
                self.ws_loggedin = True
            else:
                self.ws_loggedin = False
            self.ws_login_cv.notify()
            self.ws_login_cv.release()
        elif msgtype == "req_start_task":  # 请求执行自动驾驶任务
            response = {'type': "res_start_task", 'code': -1, 'msg': '', 'data': {}}
            if data.get("car_id", "") != self.userid:
                response['code'] = 1
                response['msg'] = "I am not given car!"
            else:
                pathid = data.get("path_id", "")
                speed = float(data.get("speed", ""))
                path_dir = os.path.join(settings.NAV_PATH_DIR, str(pathid))
                if not os.path.exists(path_dir):
                    # 创建路径文件夹
                    os.mkdir(path_dir)
                    ok = self.download_navpathfile(path_dir, pathid)
                    if not ok:  # 下载失败, 删除文件夹
                        print("download failed. delete directory: %s" % path_dir)
                        # os.removedirs(path_dir)
                        shutil.rmtree(path_dir)

                if not os.path.exists(path_dir):  # 路径文件不存在
                    response['code'] = 1
                    response['msg'] = "Download navigation path failed!"
                else:
                    res, msg = self.request_av_task.do(self.request_av_task.start, (path_dir, speed), 5.0)
                    response['code'] = 0 if res else 1
                    response['msg'] = msg

            print("ws send: %s" % response)
            self.__core_ws.send(json.dumps(response))
        elif msgtype == "req_stop_task":
            response = {'type': msgtype.replace("req", "res"), 'code': -1, 'msg': '', 'data': {}}
            res, msg = self.request_av_task.do(self.request_av_task.stop, (), 5.0)
            response['code'] = 0 if res else 1
            response['msg'] = msg
            self.__core_ws.send(json.dumps(response))

    def __onWebSocketError(self, ws, error):
        print("error", error)

    # websocket已断开
    def __onWebSocketClose(self, ws):
        print("### closed ###")
        self.ws_loggedin = False





