#include <little_ant_msgs/State4.h>  //steeringAngle  hz 35
#include <little_ant_msgs/State2.h>  //vehicle_speed  hz 19
#include <path_tracking/State.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
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
		ros::NodeHandle nh_private("~");
	
		nh_private.param<std::string>("file_path",file_path_,"");
		nh_private.param<std::string>("file_name",file_name_,"");
	
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
		
		out_file_ << "utm_x\tutm_y\tyaw\t roadwheelAngle\t speed\t angular_velocity_x\t y\t z\t linear_acc_x\t y\t z\t lat_err\t yaw_err\t longitude\t latitude" << std::endl;
		
		sub_state2_ = nh.subscribe("/vehicleState2", 1, &Recorder::state2_callback, this);
		sub_state4_ = nh.subscribe("/vehicleState4", 1, &Recorder::state4_callback, this);
		sub_imu_ = nh.subscribe(nh_private.param<std::string>("imu_topic","/imu"),1,&Recorder::imu_callback, this);
		sub_utm_odom_ = nh.subscribe(nh_private.param<std::string>("utm_topic","/ll2utm_odom") ,1,&Recorder::utm_callback,this);
		sub_state_ = nh.subscribe(nh_private.param<std::string>("state_topic", ""), 1, &Recorder::state_callback, this);
		
		timer_ = nh.createTimer(ros::Duration(0.03), &Recorder::timer_callback, this);
		
		return true;
	}
	
	void state2_callback(const little_ant_msgs::State2::ConstPtr& state2)
	{
		speed_ = state2->vehicle_speed;
	}
	
	void state4_callback(const little_ant_msgs::State4::ConstPtr& state4)
	{
		roadwheel_angle_ = state4->roadwheelAngle;
	}
	
	void imu_callback(const sensor_msgs::Imu::ConstPtr& imu)
	{
		imu_msg_ = *imu;
	}
	
	void utm_callback(const nav_msgs::Odometry::ConstPtr& msg)
	{
		odom_msg_ = *msg;
	}
	
	void state_callback(const path_tracking::State::ConstPtr& state)
	{
		state_msg_ = *state;
	}
	
	void timer_callback(const ros::TimerEvent&)
	{
		static int delay = 50;
		if(delay > 0)
		{
			delay -- ;
			return ;
		}
			
		out_file_ << std::fixed << std::setprecision(3)
				  << odom_msg_.pose.pose.position.x << "\t" << odom_msg_.pose.pose.position.y << "\t" << odom_msg_.pose.covariance[0] << "\t"
				  << roadwheel_angle_ << "\t" << speed_ << "\t"
				  << imu_msg_.angular_velocity.x << "\t" << imu_msg_.angular_velocity.y << "\t" << imu_msg_.angular_velocity.x << "\t"
				  << imu_msg_.linear_acceleration.x << "\t" << imu_msg_.linear_acceleration.y << "\t" << imu_msg_.linear_acceleration.z << "\t"
				  << state_msg_.lateral_error << "\t" << state_msg_.yaw_error  << "\t"
				  << std::setprecision(7)
				  << odom_msg_.pose.covariance[1] << "\t" << odom_msg_.pose.covariance[2]<< std::endl;
		out_file_.flush();
	}
	
	
private:
	std::string file_path_;
	std::string	file_name_;
	std::ofstream out_file_;
	
	ros::Subscriber sub_state2_;
	ros::Subscriber sub_state4_;
	ros::Subscriber sub_imu_;
	ros::Subscriber sub_utm_odom_;
	ros::Subscriber sub_state_;
	
	ros::Timer timer_;
	
	float roadwheel_angle_;
	float speed_;
	sensor_msgs::Imu imu_msg_;
	nav_msgs::Odometry odom_msg_;
	path_tracking::State state_msg_;
};

int main(int argc,char**argv)
{
	ros::init(argc,argv,"record_manual_node");
	Recorder recorder;
	if(!recorder.init())
		return 1;
	ros::spin();
	return 0;
}


