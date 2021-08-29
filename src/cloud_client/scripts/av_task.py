#! /usr/bin/env python
# coding=utf-8
import os
import threading
import time

import actionlib
import rospy
from driverless_common.msg import DoDriverlessTaskAction, DoDriverlessTaskGoal, DoDriverlessTaskResult
from scripts import settings


class RequestAvTask:
    def __init__(self):
        self.cv = threading.Condition()
        self.res = None
        self.msg = ""

        self.ac = actionlib.SimpleActionClient("do_driverless_task", DoDriverlessTaskAction)
        # self.ac.wait_for_server()  # 等待连接服务器
    
    def do(self, func, params, timeout=0.0):
        if not self.ac.wait_for_server(rospy.Duration(1.0)):
            return False, "Driverless sever offline!"

        self.cv.acquire()
        # print(threading.currentThread().ident)

        if func(*params):
            self.cv.wait(timeout)

        if self.res is None:
            self.cv.release()
            return False, "Request timeout in car"

        self.cv.release()
        return self.res, self.msg

    def start(self, path_dir, speed):
        goal = DoDriverlessTaskGoal()
        goal.task = goal.DRIVE_TASK
        goal.type = goal.FILE_TYPE
        goal.roadnet_file = os.path.join(path_dir, "points_file.txt")
        goal.roadnet_ext_file = os.path.join(path_dir, "extend_file.xml")
        goal.expect_speed = speed
        if not self.ac.wait_for_server(rospy.Duration(1.0)):
            print("Driverless sever offline!")
            self.res = False
            self.msg = "Driverless sever offline!"
            return False

        print("send goal: ", goal)
        self.ac.send_goal(goal, self.action_done_callback, self.action_active_callback,
                          self.action_feedback_callback)
        return True

    def stop(self):
        self.ac.cancel_all_goals()
        return True

    def action_done_callback(self, status, result):
        self.notify(result.success, self.ac.get_goal_status_text())

    def action_active_callback(self):
        print("active")
        pass

    def action_feedback_callback(self, feedback):
        pass

    def done(self):
        ok = self.cv.acquire()
        # print(threading.currentThread().ident)
        time.sleep(2.0)
        self.res = True
        self.msg = "test ok."
        self.cv.notify()
        self.cv.release()

    # 通知http请求
    def notify(self, res, msg):
        self.cv.acquire()
        self.res = res
        self.msg = msg
        self.cv.notify()
        self.cv.release()
