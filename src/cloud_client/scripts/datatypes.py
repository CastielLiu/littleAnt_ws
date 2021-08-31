
class ChangeDataMonitor(object):
    def __init__(self):
        self.__data = [None, None]
        self.__index = 0

    def set(self, other):
        self.__index = (self.__index + 1) % 2
        self.__data[self.__index] = other

    def get(self):
        return self.__data[self.__index]

    def last(self):
        return self.__data[(self.__index+1) % 2]

    def changed(self):
        return self.__data[0] != self.__data[1]
