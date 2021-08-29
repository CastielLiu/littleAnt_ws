# !usr/bin/env python
# -*- coding:utf-8 _*-
"""
@Author:何以解忧
@Blog(个人博客地址): https://www.codersrc.com
@Github:www.github.com

@File:python_.py
@Time:2019/10/21 21:25

@Motto:不积跬步无以至千里，不积小流无以成江海，程序人生的精彩需要坚持不懈地积累！
"""

# 导入线程模块
import threading

# 创建条件变量condition
con = threading.Condition()


def thread_one(name):
    # 条件变量condition 线程上锁
    con.acquire()

    print("{}:成语接龙准备好了吗".format(name))
    # 唤醒正在等待(wait)的线程
    con.notify()

    # 等待对方回应消息，使用wait阻塞线程，等待对方通过notify唤醒本线程
    con.wait()
    print("{}:一干二净".format(name))
    # 唤醒对方
    con.notify()

    # 等待消息答应
    con.wait()
    print("{}:一天就知道看抖音美女，给你来个简单点的，来了：毛手毛脚".format(name))
    # 唤醒对方
    con.notify()

    # 等待消息答应
    con.wait()
    print("{}:哟哟哟，不错不错！".format(name))
    # 唤醒对方
    con.notify()

    # 条件变量condition 线程释放锁
    con.release()


def thread_two(name):
    # 条件变量condition 线程上锁
    con.acquire()

    # wait阻塞状态，等待其他线程通过notify唤醒本线程
    con.wait()
    print("{}:准备好了~开始吧！".format(name))
    # 唤醒对方
    con.notify()

    # 等待消息答应
    con.wait()
    print("{}:净你妹啊，没法接...来个简单点的...".format(name))
    # 唤醒对方
    con.notify()

    # 等待消息答应
    con.wait()
    print("{}:嘿,这个我知道：脚踏实地".format(name))
    # 唤醒对方
    con.notify()

    con.release()


if __name__ == "__main__":
    # 创建并初始化线程
    t1 = threading.Thread(target=thread_one, args=("A", ))
    t2 = threading.Thread(target=thread_two, args=("B", ))

    # 启动线程 -- 注意线程启动顺序，启动顺序很重要
    t2.start()
    t1.start()

    # 阻塞主线程，等待子线程结束
    t1.join()
    t2.join()

    print("程序结束！")