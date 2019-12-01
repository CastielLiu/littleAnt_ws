
# -*-coding=utf-8-*-

import numpy as np
import math
import matplotlib.pyplot as plt
import os
import sys

class Point:
	def __init__(self,x,y):
		self.x = x
		self.y = y
		self.yaw = 0
		self.curvature = 0

def loadPoints(file_name,offset_x=0,offset_y=0):
	points = []
	with open(file_name,'r') as f:
		while True:
			line = f.readline()
			if(len(line) == 0):
				break
			data = line.split()
			x = float(data[0])+offset_x
			y = float(data[1])+offset_y
			points.append(Point(x,y))
	return points

def main(argv):
	if(len(argv) <2):
		print("please input file name");
		return
	colors = ['y-','r-*','g-.','b-*','k-*']
	for i in range(1,len(argv)):
		points = loadPoints(argv[i])
		xs = [point.x for point in points]
		ys = [point.y for point in points]
		plt.plot(xs, ys, colors[i],label=colors[i])

	plt.axis('equal')
	plt.legend()
	plt.show()

if __name__ == "__main__":
	main(sys.argv)
