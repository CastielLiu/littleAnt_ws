import os
import threading
import time

import actionlib
from driverless_common.msg import DoDriverlessTaskAction, DoDriverlessTaskGoal, DoDriverlessTaskResult
from scripts import settings


class RequestAvTask:
    def __init__(self):
        self.cv = threading.Condition()
        self.res = None
        self.msg = ""

        self.ac = actionlib.ActionClient("")

    def req(self, path_dir, speed):
        pass



        threading.Thread(target=self.done).start()
        pass

    def action_done_callback(self, status, result):
        pass

    def action_active_callback(self, status, result):
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
