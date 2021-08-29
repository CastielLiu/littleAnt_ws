#! /usr/bin/env python
# coding=utf-8
import json

import rospy
from driverless_common.msg import SystemState
from cloud_client import CloudClient


class CloudClientNode(CloudClient):
    def __init__(self):
        CloudClient.__init__(self)
        self.timer1s = None
        self.subSystemState = None
        self.system_state = None
        # rospy.get_param("name", "default_val")

    def init(self):
        if not self.login():
            return False
        self.subSystemState = rospy.Subscriber('/driverless/system_state', SystemState, self.systemStateCallback)
        self.timer1s = rospy.Timer(rospy.Duration(1), self.timerCallback_1s)
        return True

    # 停止运行
    def stop(self):
        CloudClient.stop(self)  #
        pass

    def download_allpath(self):
        pathlist = self.get_navpathlist()
        for path in pathlist:
            self.download_navpathfile(path['id'])

    def systemStateCallback(self, state):
        self.system_state = state

    def timerCallback_1s(self, event):
        if not self.check_login():
            rospy.signal_shutdown("stop")

        if self.system_state:
            data_dict = dict()
            data_dict['speed'] = self.system_state.vehicle_speed
            data_dict['steer_angle'] = self.system_state.roadwheel_angle
            data_dict['task_state'] = self.system_state.task_state
            data_dict['x'] = self.system_state.position_x
            data_dict['y'] = self.system_state.position_y
            data_dict['lng'] = 120.16575
            data_dict['lat'] = 33.37940

            send_dict = dict()
            send_dict['type'] = "rep_car_state"
            send_dict['data'] = data_dict
            self.sendCoreData(json.dumps(send_dict))


# rospy.sleep(rospy.Duration(1.0))
# rospy.signal_shutdown()
def main():
    rospy.init_node("driverless_websocket_client", anonymous=True)
    app = CloudClientNode()
    if not app.init():
        return

    rospy.spin()
    app.stop()


if __name__ == "__main__":
    main()
