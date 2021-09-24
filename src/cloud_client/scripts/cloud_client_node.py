#! /usr/bin/env python
# coding=utf-8
import json

import rospy
from driverless_common.msg import SystemState
from cloud_client import CloudClient
from datatypes import ChangeDataMonitor
from utils import *


class CloudClientNode(CloudClient):
    def __init__(self):
        CloudClient.__init__(self)
        self.timer1s = None
        self.subSystemState = None
        self.system_state = None
        # rospy.get_param("name", "default_val")

        # 需要监测变化的数据(这些数据一旦变化,需要及时的上报到服务器)
        self.av_status = ChangeDataMonitor()

        self.test_trajectory = gen_test_trajectory()
        self.i = 0

    def init(self):
        if not self.login(10):
            return False

        self.navpath_dir = rospy.get_param("~navpath_dir", "../paths/")
        self.subSystemState = rospy.Subscriber('/driverless/system_state', SystemState, self.systemStateCallback)
        self.timer1s = rospy.Timer(rospy.Duration(1.0), self.timerCallback_1s)
        return True

    # 停止运行
    def stop(self):
        CloudClient.stop(self)  #

    # 下载所有路径文件
    def download_allpath(self):
        pathlist = self.get_navpathlist()
        for path in pathlist:
            self.download_navpathfile(path['id'])

    # 自动驾驶系统状态回调函数
    def systemStateCallback(self, state):
        self.system_state = state
        self.av_status.set(state.task_state)

        if self.av_status.changed():
            self.reportStateData()

    # 定时回调函数
    def timerCallback_1s(self, event):
        if not self.check_login():
            print("您已掉线, 正在尝试重新登录...")
            self.login()  # 重新登录
            return
            # rospy.signal_shutdown("stop")
        self.reportStateData()

    # 上报车辆状态信息
    def reportStateData(self):
        if self.system_state:
            # print ("reportStateData")
            data_dict = dict()
            data_dict['speed'] = self.system_state.vehicle_speed
            data_dict['steer_angle'] = self.system_state.roadwheel_angle
            data_dict['status'] = self.system_state.task_state
            data_dict['x'] = self.system_state.position_x
            data_dict['y'] = self.system_state.position_y

            # data_dict['lng'] = 120.16575
            # data_dict['lat'] = 33.37940

            data_dict['lng'], data_dict['lat'] = self.test_trajectory[self.i]
            self.i = (self.i + 1) % len(self.test_trajectory)

            send_dict = dict()
            send_dict['type'] = "rep_car_state"
            send_dict['data'] = data_dict
            self.sendCoreData(json.dumps(pretty_floats(send_dict, 5)))


# rospy.sleep(rospy.Duration(1.0))
# rospy.signal_shutdown()
def main():
    rospy.init_node("driverless_websocket_client", anonymous=True)
    app = CloudClientNode()
    if not app.init():
        rospy.signal_shutdown("init failed")
    try:
        rospy.spin()
    except Exception as e:
        print("main", e)
    finally:
        app.stop()


if __name__ == "__main__":
    main()
