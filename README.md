1. ./src/rtcm3.2/main  
	[启动千寻差分, 电脑务必连接4G网络] [启动后，等待数据稳定再执行下一步, RTK灯，常亮]

2. ./record_path.sh path.txt
	[记录路径GPS坐标点 并自动生成路径曲率, path.txt 为文件名]
3. source devel/setup.bash
	
4. roslaunch little_ant path_traking.launch  pathTracking_file_name:=path.txt  max_speed:="30.0"
	[启动路径跟踪,file_name 应与记录时的文件名一致;  如需其他参数,查看launch文件]

[1,2,3指令只能在当前目录下执行]
