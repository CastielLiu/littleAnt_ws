
import matplotlib.pyplot as plt
import numpy as np
import math
import sys


def main(argv):
	if(len(sys.argv) != 2):
		print("please input file name")
		return
	file_name = argv[1]
	datas = []
	with open(file_name,'r') as f:
		line = f.readline()
		while True:
			line = f.readline()
			if (len(line) == 0):
				break
			data = line.split()
			data = [float(x) for x in data]
			datas.append(data)
	datas = np.array(datas)
	plt.plot(range(datas.shape[0]),datas[:,3])
#	plt.plot(datas[:,0],datas[:,1])
	
#	plt.plot(path_points.y,path_points.x,'r.',label="reference path")
#	plt.plot(result_points.y,result_points.x,'k-',label="trajectory")
#	plt.legend()
#	#plt.ylim((0,30))
#	#plt.xticks(np.arange(0,200,2))
#	plt.grid('on')
#	plt.savefig('a.pdf')
	plt.show()

if __name__=="__main__":
	main(sys.argv)

