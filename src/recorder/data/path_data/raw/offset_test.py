import matplotlib.pyplot as plt
import numpy as np
import math
import sys

class Points:
	def __init__(self):
		self.x = []
		self.y = []
		self.yaw = []
		self.curvature = []
		
	def load(self,file_name):
		with open(file_name,'r') as f:
			lines = f.readlines()
		for line in lines:
			x,y,yaw = line.split()
			self.x.append(float(x))
			self.y.append(float(y))
			self.yaw.append(float(yaw))
	def offset(self,val):
		for i in range(len(self.x)):
			self.x[i] = self.x[i] + val * math.cos(self.yaw[i])
			self.y[i] = self.y[i] - val * math.sin(self.yaw[i])
		
def main():
	points = Points()
	points.load("path2.txt")
	plt.plot(points.x[0],points.y[0],'r*')
	plt.plot(points.x,points.y,'r-')
	points.offset(-1.5)
	plt.plot(points.x,points.y,'b-')
	plt.axis('equal')
	plt.show()
	
main()
