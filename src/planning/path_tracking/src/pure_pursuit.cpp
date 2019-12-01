#include<ros/ros.h>
#include<little_ant_msgs/ControlCmd.h>
#include<vector>
#include<memory>
#include<little_ant_msgs/State2.h>  //speed
#include<little_ant_msgs/State4.h>  //steerAngle

#include<message_filters/subscriber.h>
#include<message_filters/time_synchronizer.h>
#include<message_filters/sync_policies/approximate_time.h>
#include<path_tracking/State.h>

#include"gps_msgs/Inspvax.h"
#include<nav_msgs/Odometry.h> 
#include<ant_math/ant_math.h>
#include<climits>

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <thread>
using std::unique_ptr;

class PurePursuit
{
public:
	PurePursuit();
	~PurePursuit();
	bool init(ros::NodeHandle nh,ros::NodeHandle nh_private);
	void run();
	
	void pub_cmd_callback(const ros::TimerEvent&);
	void gps_odom_callback(const nav_msgs::Odometry::ConstPtr& utm);
					  
	void vehicleState4_callback(const little_ant_msgs::State4::ConstPtr& msg);
	void vehicleSpeed_callback(const little_ant_msgs::State2::ConstPtr& msg);

	bool is_gps_data_valid(gpsMsg_t& point);
	void rosSpinThread(){ros::spin();}

private:
	void publishPathTrackingState();
	
private:
	ros::Subscriber sub_vehicleState2_;
	ros::Subscriber sub_vehicleState4_;
	ros::Subscriber sub_utm_odom_;
	
	ros::Timer timer_;
	ros::Publisher pub_gps_cmd_;
	ros::Publisher pub_tracking_state_;
	
	boost::shared_ptr<boost::thread> new_thread_;
	
	std::string path_points_file_;
	
	std::vector<gpsMsg_t> path_points_;
	
	size_t target_point_index_;
	size_t nearest_point_index_;
	
	gpsMsg_t current_point_;
	gpsMsg_t target_point_;
	
	float disThreshold_;
	
	little_ant_msgs::ControlCmd cmd_;
	path_tracking::State tracking_state_;
	
	float path_tracking_speed_;
	
	bool vehicle_speed_status_;
	
	float vehicle_speed_;
	float current_roadwheelAngle_;
	
	float max_roadwheelAngle_;
	
	float lateral_err_;
	float yaw_err_;
};


PurePursuit::PurePursuit():
	vehicle_speed_status_(false),
	target_point_index_(0),
	nearest_point_index_(0),
	max_roadwheelAngle_(25.0)
{
	cmd_.origin = little_ant_msgs::ControlCmd::_GPS;
	cmd_.status = true;
	
	cmd_.cmd2.set_gear =1;
	cmd_.cmd2.set_speed =0.0;
	cmd_.cmd2.set_brake=0.0;
	cmd_.cmd2.set_accelerate =0.0;
	cmd_.cmd2.set_roadWheelAngle =0.0;
	cmd_.cmd2.set_emergencyBrake =0;
	
	cmd_.cmd1.set_driverlessMode =true;
}

PurePursuit::~PurePursuit()
{
}

bool PurePursuit::init(ros::NodeHandle nh,ros::NodeHandle nh_private)
{
	
	sub_vehicleState2_ = nh.subscribe("/vehicleState2",1,&PurePursuit::vehicleSpeed_callback,this);
	
	sub_vehicleState4_ = nh.subscribe("/vehicleState4",1,&PurePursuit::vehicleState4_callback,this);
	
	pub_gps_cmd_ = nh.advertise<little_ant_msgs::ControlCmd>("/sensor_decision",1);
	pub_tracking_state_ = nh.advertise<path_tracking::State>("/tracking_state",1);
	
	timer_ = nh.createTimer(ros::Duration(0.01),&PurePursuit::pub_cmd_callback,this);
	
	nh_private.param<std::string>("path_points_file",path_points_file_,"");

	nh_private.param<float>("speed",path_tracking_speed_,3.0);

	nh_private.param<float>("disThreshold",disThreshold_,5.0);
	
	if(path_points_file_.empty())
	{
		ROS_ERROR("no input path points file !!");
		return false;
	}
	
	//start the ros::spin() thread
	new_thread_ = boost::shared_ptr<boost::thread >(new boost::thread(boost::bind(&PurePursuit::rosSpinThread, this)));
	
	if(!loadPathPoints(path_points_file_, path_points_))
		return false;
	
	ROS_INFO("pathPoints size:%d",path_points_.size());
	
	while(ros::ok() && !is_gps_data_valid(current_point_))
	{
		ROS_INFO("gps data is invalid, please check the gps topic or waiting...");
		sleep(1);
	}
	
	target_point_index_ = findNearestPoint(path_points_,current_point_);
	
	if(target_point_index_ > path_points_.size() - 10)
	{
		ROS_ERROR("target index:%d ?? file read over, No target point was found !!!",target_point_index_);
		return false;
	}
	
	while(!vehicle_speed_status_ && ros::ok())
	{
		ROS_INFO("waiting for vehicle speed data ok ...");
		usleep(200000);
	}
	
	target_point_ = path_points_[target_point_index_];
	return true;
}

void PurePursuit::publishPathTrackingState()
{
	tracking_state_.header.stamp = ros::Time::now();
	tracking_state_.position_x = current_point_.x;
	tracking_state_.position_y = current_point_.y;
	tracking_state_.yaw = current_point_.yaw;
	tracking_state_.vehicle_speed =  vehicle_speed_;
	tracking_state_.roadwheel_angle = current_roadwheelAngle_;
	tracking_state_.lateral_error = lateral_err_;
	tracking_state_.yaw_error = yaw_err_;
	
	tracking_state_.target_index = target_point_index_;
	tracking_state_.current_index = nearest_point_index_;
	pub_tracking_state_.publish(tracking_state_);
}

void PurePursuit::run()
{
	size_t i =0;
	
	ros::Rate loop_rate(30);
	
	while(ros::ok() && target_point_index_ < path_points_.size()-2)
	{
		try
		{
			lateral_err_ = calculateDis2path(current_point_.x,current_point_.y,path_points_,
											 target_point_index_,&nearest_point_index_) ;
		}catch(const char* str)
		{
			ROS_INFO("%s",str);
			break;
		}
									 
		std::pair<float, float> dis_yaw = get_dis_yaw(target_point_, current_point_);

		if( dis_yaw.first < disThreshold_)
		{
			target_point_ = path_points_[++target_point_index_];
			continue;
		}
		
		yaw_err_ = dis_yaw.second - current_point_.yaw;
		
		if(yaw_err_==0.0) continue;
		
		float turning_radius = (-0.5 * dis_yaw.first)/sin(yaw_err_);

		float t_roadWheelAngle = generateRoadwheelAngleByRadius(turning_radius);
		
		if(t_roadWheelAngle > max_roadwheelAngle_)
			t_roadWheelAngle = max_roadwheelAngle_;
		else if(t_roadWheelAngle < -max_roadwheelAngle_)
			t_roadWheelAngle = -max_roadwheelAngle_;
		
		cmd_.cmd2.set_speed = path_tracking_speed_;
		
		cmd_.cmd2.set_roadWheelAngle = t_roadWheelAngle;
		
		this->publishPathTrackingState();
		
		if(i%30==0)
		{
			ROS_INFO("dis2target:%.2f\t yaw_err:%.2f\t lat_err:%.2f",dis_yaw.first,yaw_err_*180.0/M_PI,lateral_err_);
			ROS_INFO("disThreshold:%f\t expect roadwheel angle:%.2f",disThreshold_,t_roadWheelAngle);
		}
		i++;
		
		loop_rate.sleep();
	}
	
	ROS_INFO("driverless completed...");
	
	cmd_.cmd2.set_roadWheelAngle = 0.0;
	cmd_.cmd2.set_speed = 0.0;
	
	while(ros::ok())
	{
		sleep(1);
	}
}


void PurePursuit::pub_cmd_callback(const ros::TimerEvent&)
{
	pub_gps_cmd_.publish(cmd_);
}

void PurePursuit::gps_odom_callback(const nav_msgs::Odometry::ConstPtr& utm)
{
	current_point_.x = utm->pose.pose.position.x;
	current_point_.y = utm->pose.pose.position.y;
	current_point_.yaw = utm->pose.covariance[0];
}

void PurePursuit::vehicleSpeed_callback(const little_ant_msgs::State2::ConstPtr& msg)
{
	if(vehicle_speed_ >20.0)
		return;
	vehicle_speed_status_ = true;
	vehicle_speed_ = msg->vehicle_speed; //  m/s
}

void PurePursuit::vehicleState4_callback(const little_ant_msgs::State4::ConstPtr& msg)
{
	current_roadwheelAngle_ = msg->roadwheelAngle;
}


bool PurePursuit::is_gps_data_valid(gpsMsg_t& point)
{
	if(fabs(point.x) > 100 && fabs(point.y) > 100)
		return true;
	return false;
}

int main(int argc,char**argv)
{
	ros::init(argc,argv,"pure_pursuit");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	
	PurePursuit path_tracking;
	if(!path_tracking.init(nh,nh_private))
		return 1;
	path_tracking.run();
	
	ROS_INFO("path tracking completed.");
	ros::shutdown();

	return 0;
}
