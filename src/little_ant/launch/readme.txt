整定l_min    
roslaunch little_ant path_tracking.launch file_name:="path" max_speed:="5.0" k1:=0.0 k2:=0.0 l_min:=2.0
roslaunch little_ant path_tracking.launch file_name:="path" max_speed:="5.0" k1:=0.0 k2:=0.0 l_min:=3.0
roslaunch little_ant path_tracking.launch file_name:="path" max_speed:="5.0" k1:=0.0 k2:=0.0 l_min:=4.0
roslaunch little_ant path_tracking.launch file_name:="path" max_speed:="5.0" k1:=0.0 k2:=0.0 l_min:=5.0
roslaunch little_ant path_tracking.launch file_name:="path" max_speed:="5.0" k1:=0.0 k2:=0.0 l_min:=6.0

整定k1
roslaunch little_ant path_tracking.launch file_name:="path" max_speed:="30." k2:=0 l_min:=4.0  k1:=0.5
roslaunch little_ant path_tracking.launch file_name:="path" max_speed:="30." k2:=0 l_min:=4.0  k1:=0.6
roslaunch little_ant path_tracking.launch file_name:="path" max_speed:="30." k2:=0 l_min:=4.0  k1:=0.7
roslaunch little_ant path_tracking.launch file_name:="path" max_speed:="30." k2:=0 l_min:=4.0  k1:=0.8


整定k2
roslaunch little_ant path_tracking.launch file_name:="path" max_speed:="30." l_min:=5.0  k1:=1.0  k2:=-1.0
roslaunch little_ant path_tracking.launch file_name:="path" max_speed:="30." l_min:=5.0  k1:=1.0  k2:=-2.0
roslaunch little_ant path_tracking.launch file_name:="path" max_speed:="30." l_min:=5.0  k1:=1.0  k2:=-3.0

不同车速
roslaunch little_ant path_tracking.launch file_name:="path" l_min:=5.0  k1:=1.0  k2:=-1.0  max_speed:="5.0"
roslaunch little_ant path_tracking.launch file_name:="path" l_min:=5.0  k1:=1.0  k2:=-1.0  max_speed:="10."
roslaunch little_ant path_tracking.launch file_name:="path" l_min:=5.0  k1:=1.0  k2:=-1.0  max_speed:="15."
roslaunch little_ant path_tracking.launch file_name:="path" l_min:=5.0  k1:=1.0  k2:=-1.0  max_speed:="20."

不同ay  
roslaunch little_ant path_tracking.launch file_name:="path" l_min:=5.0  k1:=1.0  k2:=-1.0  max_speed:="30." max_ay:="1.0"
roslaunch little_ant path_tracking.launch file_name:="path" l_min:=5.0  k1:=1.0  k2:=-1.0  max_speed:="30." max_ay:="0.9"
roslaunch little_ant path_tracking.launch file_name:="path" l_min:=5.0  k1:=1.0  k2:=-1.0  max_speed:="30." max_ay:="0.8"




