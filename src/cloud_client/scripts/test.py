#! /usr/bin/env python
# coding=utf-8
import json
from functools import partial
import requests


class Child:
    def __init__(self):
        print("child construct")
        pass

    def start(self):
        self.run()

    def __del__(self):
        print("child destruct")

    def run(self):
        print("child running")


class Parent(Child):
    def __init__(self):
        self.obj = self
        print("parent construct")
        pass

    def __del__(self):
        print("parent destruct")

    def run(self):
        print("parent running")

    def test(self):
        print hasattr(self, "ok")

    def test_args(self, *args, **kwargs):
        print("args: ", args)
        print("kwargs: ", kwargs)


def main():
    # p = Parent()
    # p.start()
    # p.test()
    # p.test_args(1, 2, 3, key=10, key2=20)
    ps = dict()
    ps['a'] = Parent()

    print ("a")

    ps.pop('a')

    print ("b")


if __name__ == '__main__':
    main()

# b1=b'sdf'
# s1='sag'
# print(type(b1),type(s1))#<class 'bytes'> <class 'str'>
# b2=b1.decode('utf8')#bytes按utf8的方式解码成str print(type(b2))#<class 'str'>
# s2=s1.encode('utf8')#str按utf8的方式编码成bytes
# print(type(s2))#<class 'bytes'>
