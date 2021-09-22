#! /usr/bin/env python
# coding=utf-8
import os
import threading
import time

import actionlib
import rospy
from driverless_common.msg import DoDriverlessTaskAction, DoDriverlessTaskGoal, DoDriverlessTaskResult


class RequestAvTask:
    def __init__(self, done_cb, feedback_cb):
        self.cv = threading.Condition()
        self.res = None
        self.msg = ""
        self.goal_accepted = False

        self.ac = actionlib.SimpleActionClient("do_driverless_task", DoDriverlessTaskAction)
        self.done_cb = done_cb
        self.feedback_cb = feedback_cb

        # self.ac.wait_for_server()  # 等待连接服务器

    # 执行特定任务
    # @param func 指定任务函数
    # @param params func的参数
    # @param wait 是否需要等待回应
    # @param timeout 等待超时时间
    def do(self, func, params, wait=False, timeout=0.0):
        if not self.ac.wait_for_server(rospy.Duration(1.0)):
            return False, "Driverless sever offline!"

        if not wait:
            func(*params)
            return self.res, self.msg

        self.cv.acquire()
        # print(threading.currentThread().ident)

        if func(*params):
            self.cv.wait(timeout)

        if self.res is None:
            self.cv.release()
            return False, "Request timeout in car"

        self.cv.release()
        return self.res, self.msg

    # 请求开始自动驾驶任务
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

        print("[av_task] send goal: %s" % goal)
        self.ac.send_goal(goal, self.action_done_callback, self.action_active_callback,
                          self.action_feedback_callback)
        self.goal_accepted = False
        return True

    # 请求终止自动驾驶任务
    def stop(self):
        self.ac.cancel_all_goals()
        self.res = True
        self.msg = ""
        return True

    # goal完成回调函数(任务执行完成/任务被终止/任务被打断)
    def action_done_callback(self, status, result):
        # as_->setSucceeded(result, "task completed"); c++
        # self.ac.get_goal_status_text() "task completed"
        if not self.goal_accepted:
            self.__notify(result.success, self.ac.get_goal_status_text())
        self.done_cb(result, self.ac.get_goal_status_text())

    # 请求已被接受, 仅代表服务器接受了此请求并开始进行分析, 至于是否执行, 尚未可知
    # 因为服务器使用的为SimpleActionServer, 默认收到请求则接受(accept)
    def action_active_callback(self):
        print("active")

    # 服务器执行反馈, 首次反馈表明服务器已接受任务并开始执行
    def action_feedback_callback(self, feedback):
        if not self.goal_accepted:
            self.__notify(True, "task start running")
            self.goal_accepted = True
        self.feedback_cb(feedback)

    # 唤醒请求线程
    def __notify(self, res, msg):
        self.cv.acquire()
        self.res = res
        self.msg = msg
        self.cv.notify()
        self.cv.release()
