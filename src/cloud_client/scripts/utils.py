#! /usr/bin/env python
# coding=utf-8
from functools import partial


# 对字典浮点数据进行精度处理
# @param obj 字典对象
# @param cnt 保留小数点位数
def pretty_floats(obj, cnt):
    if isinstance(obj, float):
        return round(obj, cnt)
    elif isinstance(obj, dict):
        return dict((k, pretty_floats(v, cnt)) for k, v in obj.items())
    elif isinstance(obj, (list, tuple)):
        # map(fun, list) 根据提供的函数对指定的序列做映射, 用fun依次处理列表数据, 然后返回一个新列表
        # partial(fun, params) 给fun绑定一个参数然后返回一个新的可调用对象
        return list(map(partial(pretty_floats, cnt=cnt), obj))
    return obj


def gen_test_trajectory(_id=0, _name="path1"):
    start_point = (33.3794, 120.16574722)
    end_point = (33.372647, 120.155741)

    size = 30
    dx = (end_point[0] - start_point[0]) / size
    dy = (end_point[1] - start_point[1]) / size

    path = []

    for i in range(size):
        point = [start_point[1] + dy * i, start_point[0] + dx * i]
        path.append(point)
    return path
