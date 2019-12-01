#from mpl_toolkits.mplot3d import axes3d

from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
import numpy as np
import math
import sys

def main(argv):
	file_name = argv[1]
	with open(file_name,'r') as f:
		lines = f.readlines()
	if lines is None:
		return False
	
	msgs = []
	
	for line in lines:
		#[x,y,yaw,lon,lat,speed,roadwheelAngle,lat_err,yaw_err] 
		
		msg = line.split()
		msg = [float(i) for i in msg]
		msgs.append(msg)
	msgs = np.array(msgs)
	
	fig1 = plt.figure(1)
	ax1 = fig1.gca(projection='3d')

	ax1.set_title("3D_Curve")
	ax1.set_xlabel("x")
	ax1.set_ylabel("y")
	ax1.set_zlabel("z")

	figure = ax1.plot(msgs[:,0], msgs[:,1], msgs[:,5], c='r')
	figure = ax1.plot(msgs[:,0], msgs[:,1], np.zeros(msgs.shape[0]), c='k')
	

	plt.show()

if __name__ == '__main__':
	if(len(sys.argv) != 2):
		print("please input file name")
	else:
		main(sys.argv)
