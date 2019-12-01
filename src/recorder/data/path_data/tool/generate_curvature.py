
import matplotlib.pyplot as plt
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
	
	def dump(self,file_name):
		with open(file_name,'w') as f:
			for i in range(len(self.x)):
				f.write('%.3f\t%.3f\t%.3f\t%.5f\n' %(self.x[i],self.y[i],self.yaw[i],self.curvature[i]))
	
	def clear(self):
		self.x.clear()
		self.y.clear()
		self.yaw.clear()
		self.curvature.clear()
		
	def calculateCurvature(self):
		self.curvature = [0.0]*len(self.x)
		for i in range(len(self.x)-1):
			theta = self.yaw[i+1] - self.yaw[i]
			dis = self.__disBetweenPoints(i,i+1)
			if(dis==0):
				self.curvature[i] = 0.0
			else:
				self.curvature[i] = 2*math.sin(theta/2)/dis
				#self.curvature[i] = theta/dis
			if(self.curvature[i]>0.3):
				self.curvature[i] = 0.3;
			elif(self.curvature[i]<-0.3):
				self.curvature[i] = -0.3
		self.curvature[len(self.x)-1] = 0.0
	
	def curvatureFilter(self,count):
		count = int(count)
		if(count%2 ==0):
			count = count + 1 
		num = count/2
		temp_cur = self.curvature[:]
		
		for i in range(len(temp_cur))[num:-num]:
			temp_list = self.curvature[i-num:i+num+1]
			temp_cur[i] = sum(temp_list)/count
		self.curvature = temp_cur[:]
	
	def __disBetweenPoints(self,i,j):
		x = self.x[i] - self.x[j]
		y = self.y[i] - self.y[j]
		return math.sqrt(x*x+y*y)
			

def main(argv):
	raw_file = ''
	result_file = ''
	
	if(len(argv)>2):
		raw_file = argv[1]
		result_file = argv[2]
	else:
		print("please input file name")
		return 
		
	path_points = Points()
	path_points.load(raw_file)
	
	path_points.calculateCurvature()
	path_points.curvatureFilter(15)
	path_points.dump(result_file)


if __name__ == '__main__':
	main(sys.argv)
