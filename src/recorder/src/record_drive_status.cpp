#include <ant_msgs/State4.h>  //steeringAngle  hz 35
#include <ant_msgs/State2.h>  //vehicle_speed  hz 19
#include <driverless/TrackingState.h>
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
	
		if(file_path_.empty())
		{
			ROS_ERROR("please set file path in launch file!");
			return false;
		}

		out_file_.open(file_path_);
		
		if(!out_file_.is_open())
		{
			ROS_ERROR("open %s failed!!!", file_path_.c_str());
			return false;
		}
		
		out_file_ << "utm_x\tutm_y\tyaw\t roadwheelAngle\t speed\t angular_velocity_x\t y\t z\t linear_acc_x\t y\t z\t lat_err\t yaw_err\t longitude\t latitude" << std::endl;
		
		sub_state2_ = nh.subscribe("/vehicleState2", 1, &Recorder::state2_callback, this);
		sub_state4_ = nh.subscribe("/vehicleState4", 1, &Recorder::state4_callback, this);
		sub_imu_ = nh.subscribe(nh_private.param<std::string>("imu_topic","/imu"),1,&Recorder::imu_callback, this);
		sub_utm_odom_ = nh.subscribe(nh_private.param<std::string>("utm_topic","/ll2utm_odom") ,1,&Recorder::utm_callback,this);
		sub_state_ = nh.subscribe(nh_private.param<std::string>("state_topic", ""), 1, &Recorder::state_callback, this);
		
		timer_ = nh.createTimer(ros::Duration(0.1), &Recorder::timer_callback, this);
		
		return true;
	}
	
	void state2_callback(const ant_msgs::State2::ConstPtr& state2)
	{
		speed_ = state2->vehicle_speed;
	}
	
	void state4_callback(const ant_msgs::State4::ConstPtr& state4)
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
	
	void state_callback(const driverless::TrackingState::ConstPtr& state)
	{
		state_msg_ = *state;
	}
	
	bool is_stationary(const float& speed)
	{
		if(fabs(speed) < 0.02)
			return true;
		return false;
	}
	
	void timer_callback(const ros::TimerEvent&)
	{
		//等待所有数据就绪
		static int delay = 30;
		if(delay-- > 0)
			return;
		
		static bool start = false;
		
		//车辆起步后才开始记录
		if(!start && !is_stationary(speed_))
			start = true;
		if(!start) return;
		
		//车辆停止后停止记录
		static float last_speed = 1.0;
		if(is_stationary(speed_) && is_stationary(last_speed))
			return;
		last_speed = speed_;
		
		out_file_ << std::fixed << std::setprecision(3)
				  << odom_msg_.pose.pose.position.x << "\t" << odom_msg_.pose.pose.position.y << "\t" << odom_msg_.pose.covariance[0] << "\t"
				  << roadwheel_angle_ << "\t" << speed_ << "\t"
				  << imu_msg_.angular_velocity.x << "\t" << imu_msg_.angular_velocity.y << "\t" << imu_msg_.angular_velocity.x << "\t"
				  << imu_msg_.linear_acceleration.x << "\t" << imu_msg_.linear_acceleration.y << "\t" << imu_msg_.linear_acceleration.z << "\t"
				  << state_msg_.lateral_error << "\t" << state_msg_.yaw_error  << "\t"
				  << std::setprecision(7)
				  << odom_msg_.pose.covariance[1] << "\t" << odom_msg_.pose.covariance[2]<< std::endl; //longitude latitude
		out_file_.flush();
	}
	
	
private:
	std::string file_path_;
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
	driverless::TrackingState state_msg_;
};

int main(int argc,char**argv)
{
	ros::init(argc,argv,"record_drive_status");
	Recorder recorder;
	if(!recorder.init())
		return 1;
	ros::spin();
	return 0;
}


