
# -*-coding=utf-8-*-

import numpy as np
import math
import matplotlib.pyplot as plt
import os
import sys

class Point:
	def __init__(self,x,y,yaw,cur):
		self.x = x
		self.y = y
		self.yaw = yaw
		self.curvature = cur

def loadPoints(file_name):
	points = []
	with open(file_name,'r') as f:
		while True:
			line = f.readline()
			if(len(line) == 0):
				break
			data = line.split()
			x = float(data[0])
			y = float(data[1])
			yaw = float(data[2])
			cur = float(data[3])
			points.append(Point(x,y,yaw, cur))
	return points

def dumpPoints(file_name, points):
	with open(file_name,'w') as f:
		for point in points:
			f.write('%.3f\t%.3f\t%.3f\t%.5f\n' %(point.x,point.y,point.yaw,point.curvature))

def offsetPoints(points, offset):
	for i in range(len(points)):
		points[i].x += offset * math.cos(points[i].yaw)
		points[i].y += -offset* math.sin(points[i].yaw)
	return points

def main(argv):
	if(len(argv) <2):
		print("please input file name");
		return
	points = loadPoints(argv[1])
	
	xs = [point.x for point in points]
	ys = [point.y for point in points]
	
	plt.plot(xs, ys, 'r-')
	
	points = offsetPoints(points, 1.3)
	
	if(len(argv) > 2):
		dumpPoints(argv[2], points)
	
	xs = [point.x for point in points]
	ys = [point.y for point in points]
	plt.plot(xs, ys, 'g-')

	plt.axis('equal')
	plt.legend()
#	plt.show()

if __name__ == "__main__":
	main(sys.argv)
