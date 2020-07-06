#include<little_ant_msgs/State4.h>  //steeringAngle  hz 35
#include<little_ant_msgs/State2.h>  //vehicle_speed  hz 19
#include <ros/ros.h>
#include <unistd.h>
#include <fstream>
#include <cmath>

class Recorder
{
public:
	Recorder(){};
	~Recorder()
	{
		if(out_file_.is_open())
			out_file_.close();
	}
	bool init()
	{
		ros::NodeHandle nh;
		ros::NodeHandle private_nh("~");
	
		private_nh.param<std::string>("file_path",file_path_,"");
		private_nh.param<std::string>("file_name",file_name_,"");
	
		if(file_path_.empty() || file_name_.empty())
		{
			ROS_ERROR("please input file path and file name in launch file!!!");
			return false;
		}
		std::string full_file_name = file_path_ + file_name_;
		out_file_.open(full_file_name);
		if(!out_file_.is_open())
		{
			ROS_ERROR("open %s failed!!!", full_file_name.c_str());
			return false;
		}
		
		sub_state2_ = nh.subscribe("/vehicleState2", 1, &Recorder::state2_callback, this);
		sub_state4_ = nh.subscribe("/vehicleState4", 1, &Recorder::state4_callback, this);
		return true;
		
	}
	void state2_callback(const little_ant_msgs::State2::ConstPtr& state2)
	{
		speed_ = state2->vehicle_speed;
	}
	
	void state4_callback(const little_ant_msgs::State4::ConstPtr& state4)
	{
		out_file_ << std::fixed << std::setprecision(3) << state4->header.stamp.toSec() << "\t"
				  << state4->roadwheelAngle << "\t" << speed_ << "\n";
	}
	
	
private:
	std::string file_path_;
	std::string	file_name_;
	std::ofstream out_file_;
	
	ros::Subscriber sub_state2_;
	ros::Subscriber sub_state4_;
	
	float roadwheel_angle_;
	float speed_;
};

int main(int argc,char**argv)
{
	ros::init(argc,argv,"record_vehicle_state_node");
	Recorder recorder;
	if(!recorder.init())
		return 1;
	ros::spin();
	return 0;
}


