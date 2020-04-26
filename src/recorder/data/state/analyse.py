
# coding=utf-8
import matplotlib.pyplot as plt
import numpy as np
import math
import sys	
	
#0 utm_x
#1 utm_y
#2 yaw
#3 方向盘转角
#4 车速
#5,6,7 角速度xyz
#8,9,10 线加速度xyz
#11 横向偏差
#12 航向偏差
#13，14 经纬度

def main(argv):
	if(len(argv) != 2):
		print("please input file name")
		return
	file_name = sys.argv[1]
	
	datas = []
	with open(file_name,'r') as f:
		line = f.readline()
		while True:
			line = f.readline()
			if(len(line)==0):
				break
			data = line.split()
			data = [float(i) for i in data]
			datas.append(data)
	
	datas = np.array(datas)
	latErrs = np.abs(datas[:,11])
	print("mean latErr: %f" %np.mean(latErrs))


main(sys.argv)
