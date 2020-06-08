#include<ros/ros.h>
#include<little_ant_msgs/ControlCmd.h>
#include<little_ant_msgs/State2.h>  //speed
#include<little_ant_msgs/State4.h>  //steerAngle
#include<diagnostic_msgs/DiagnosticStatus.h>
#include<std_msgs/Float32.h>
#include<std_msgs/UInt32.h>
#include<std_msgs/UInt8.h>
#include<vector>

#include<nav_msgs/Odometry.h> 
#include<geometry_msgs/Quaternion.h>
#include<tf/transform_datatypes.h>
#include<std_msgs/Float32.h>

#include<ant_math/ant_math.h>
#include<path_tracking/State.h>
#include<climits>

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <thread>

class PathTracking
{
public:
	PathTracking();
	~PathTracking();
	bool init(ros::NodeHandle nh,ros::NodeHandle nh_private);
	void run();
	
	void pub_cmd_timer(const ros::TimerEvent&);
	void gps_odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
	
	void vehicleState4_callback(const little_ant_msgs::State4::ConstPtr& msg);
	void vehicleSpeed_callback(const little_ant_msgs::State2::ConstPtr& msg);
	void avoiding_flag_callback(const std_msgs::Float32::ConstPtr& msg);

	bool is_gps_data_valid(gpsMsg_t& point);
	void rosSpinThread(){ros::spin();}

private:
	float limitSpeedByDestination(const float& speed,const float& acc=1.5);
	void publishDiagnostics(uint8_t level,const std::string& msg);
	gpsMsg_t pointOffset(const gpsMsg_t& point,float offset);
	void publishPathTrackingState();
	float disToEnd();
private:
	ros::Subscriber sub_utm_odom_;
	ros::Subscriber sub_vehicleState2_;
	ros::Subscriber sub_vehicleState4_;
	ros::Subscriber sub_avoiding_from_lidar_;
	ros::Timer timer_;
	
	ros::Publisher pub_cmd_;
	ros::Publisher pub_diagnostic_;
	little_ant_msgs::ControlCmd gps_controlCmd_;
	diagnostic_msgs::DiagnosticStatus diagnostic_msg_;
	
	ros::Publisher pub_tracking_state_;
	path_tracking::State tracking_state_;
	
	boost::shared_ptr<boost::thread> rosSpin_thread_ptr_;
	
	std::string path_points_file_;
	std::vector<gpsMsg_t> path_points_;
	float path_points_resolution_;
	
	gpsMsg_t current_point_, target_point_;
	
	float min_foresight_distance_;
	float disThreshold_;
	float avoiding_offset_;
	
	float track_speed_;
	
	bool vehicle_speed_status_;
	
	float vehicle_speed_;
	float current_roadwheelAngle_;
	
	float safety_distance_front_;
	float danger_distance_front_;
	
	float max_roadwheelAngle_;
	float max_side_accel_;
	bool is_avoiding_;
	float lateral_err_;
	float yaw_err_;
	
	size_t target_point_index_;
	size_t nearest_point_index_;
	
	float foreSightDis_speedCoefficient_;
	float foreSightDis_latErrCoefficient_;
};

PathTracking::PathTracking():
	vehicle_speed_status_(false),
	target_point_index_(0),
	nearest_point_index_(0),
	avoiding_offset_(0.0),
	max_roadwheelAngle_(25.0),
	is_avoiding_(false)
{
	gps_controlCmd_.origin = little_ant_msgs::ControlCmd::_GPS;
	gps_controlCmd_.status = true;
	
	gps_controlCmd_.cmd2.set_gear =1;
	gps_controlCmd_.cmd2.set_speed =0.0;
	gps_controlCmd_.cmd2.set_brake=0.0;
	gps_controlCmd_.cmd2.set_accelerate =0.0;
	gps_controlCmd_.cmd2.set_roadWheelAngle =0.0;
	gps_controlCmd_.cmd2.set_emergencyBrake =0;
	
	gps_controlCmd_.cmd1.set_driverlessMode =true;
	diagnostic_msg_.hardware_id = "path_tracking";
	path_points_resolution_ = 0.1;
}

PathTracking::~PathTracking()
{
}

void PathTracking::publishDiagnostics(uint8_t level,const std::string& msg)
{
	diagnostic_msg_.level = level;
	diagnostic_msg_.message = msg;
	pub_diagnostic_.publish(diagnostic_msg_);
}

bool PathTracking::init(ros::NodeHandle nh,ros::NodeHandle nh_private)
{
	std::string utm_odom_topic = nh_private.param<std::string>("utm_odom_topic","/ll2utm_odom");
	std::string tracking_info_topic = nh_private.param<std::string>("tracking_info_topic","/tracking_state")；

	pub_diagnostic_ = nh.advertise<diagnostic_msgs::DiagnosticStatus>("driverless/diagnostic",1);
	
	pub_tracking_state_ = nh.advertise<path_tracking::State>(tracking_info_topic,1);

	nh_private.param<float>("speed",track_speed_,5.0);
	nh_private.param<float>("foreSightDis_speedCoefficient", foreSightDis_speedCoefficient_,0.8);
	nh_private.param<float>("foreSightDis_latErrCoefficient", foreSightDis_latErrCoefficient_,-3.0);
	nh_private.param<float>("min_foresight_distance",min_foresight_distance_,4.0);
	nh_private.param<float>("max_side_accel",max_side_accel_,1.0);
	

	target_point_index_ = findNearestPoint(path_points_,current_point_);
	
	if(target_point_index_ > path_points_.size() - 10)
	{
		ROS_ERROR("target index:%d\t, No target point was found !!!",target_point_index_);
		publishDiagnostics(diagnostic_msgs::DiagnosticStatus::ERROR,"Remaind target path is too short!");
		return false;
	}
	
	target_point_ = path_points_[target_point_index_];
	return true;
}

gpsMsg_t PathTracking::pointOffset(const gpsMsg_t& point,float offset)
{
	gpsMsg_t result = point;
	result.x =  offset * cos(point.yaw) + point.x;
	result.y = -offset * sin(point.yaw) + point.y;
	return result;
}

float PathTracking::disToEnd()
{
	float dis1 = path_points_resolution_ * fabs(path_points_.size()-nearest_point_index_); //估计路程
	float dis2 = disBetweenPoints(path_points_[path_points_.size()-1],path_points_[nearest_point_index_]);//直线距离
	
	return dis1 > dis2 ? dis1 : dis2;
}

float PathTracking::limitSpeedByDestination(const float& speed,const float& acc)
{
	float dis2end = disToEnd();
	float maxSpeed = sqrt(2*acc*dis2end);
	if(dis2end < 0.5) //到达终点附近
		return 0;

	return speed > maxSpeed ? maxSpeed : speed;
}

void PathTracking::run()
{
	size_t i =0;
	
	ros::Rate loop_rate(30);
	
	while(ros::ok() && target_point_index_ < path_points_.size()-2)
	{
		if( avoiding_offset_ != 0.0)
			target_point_ = pointOffset(path_points_[target_point_index_],avoiding_offset_);
		
		try
		{
			lateral_err_ = calculateDis2path(current_point_.x,current_point_.y,path_points_,
											 target_point_index_,&nearest_point_index_) - avoiding_offset_;
		}
		catch(const char* str)
		{
			ROS_INFO("%s",str);
			break;
		}
		
		disThreshold_ = foreSightDis_speedCoefficient_ * vehicle_speed_ + foreSightDis_latErrCoefficient_ * fabs(lateral_err_);
	
		if(disThreshold_ < min_foresight_distance_) 
			disThreshold_  = min_foresight_distance_;
			
//		 disThreshold_ = min_foresight_distance_ + 
//						 foreSightDis_speedCoefficient_ * vehicle_speed_ + 
//						 foreSightDis_latErrCoefficient_ * fabs(lateral_err_);
	
		//ROS_INFO("disThreshold:%f\t lateral_err:%f",disThreshold_,lateral_err_);
									 
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
		
		t_roadWheelAngle = limitRoadwheelAngleBySpeed(t_roadWheelAngle,vehicle_speed_);
		
		//ROS_INFO("t_roadWheelAngle :%f\n",t_roadWheelAngle);
		
		//find the index of a path point x meters from the current point
		size_t index = findIndexForGivenDis(path_points_,nearest_point_index_,disThreshold_ + 13); 
		if(index ==0)
		{
			ROS_INFO("findIndexForGivenDis faild!");
			break;
		}
		float max_curvature = maxCurvatureInRange(path_points_, nearest_point_index_, index);
		float max_speed = generateMaxTolarateSpeedByCurvature(max_curvature, max_side_accel_);

		max_speed = limitSpeedByDestination(max_speed); 
		
		gps_controlCmd_.cmd2.set_speed = track_speed_ > max_speed ? max_speed : track_speed_;
		gps_controlCmd_.cmd2.set_roadWheelAngle = t_roadWheelAngle;
		
		publishPathTrackingState();

		if((i++)%20==0)
		{
			ROS_INFO("min_r:%.3f\t max_speed:%.1f",1.0/max_curvature, max_speed);
			ROS_INFO("set_speed:%f\t speed:%f",gps_controlCmd_.cmd2.set_speed ,vehicle_speed_*3.6);
			ROS_INFO("dis2target:%.2f\t yaw_err:%.2f\t lat_err:%.2f",dis_yaw.first,yaw_err_*180.0/M_PI,lateral_err_);
			ROS_INFO("disThreshold:%f\t expect roadwheel angle:%.2f",disThreshold_,t_roadWheelAngle);
			ROS_INFO("avoiding_offset_:%f\n",avoiding_offset_);
			publishDiagnostics(diagnostic_msgs::DiagnosticStatus::OK,"Running");
		}
		
		loop_rate.sleep();
	}
	
	publishDiagnostics(diagnostic_msgs::DiagnosticStatus::OK,"Arrived at the destination.");
	ROS_INFO("driverless completed...");
	
	gps_controlCmd_.cmd2.set_roadWheelAngle = 0.0;
	gps_controlCmd_.cmd2.set_speed = 0.0;
	
	while(ros::ok())
	{
		sleep(1);
	}
}

void PathTracking::publishPathTrackingState()
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

void PathTracking::pub_cmd_timer(const ros::TimerEvent&)
{
	pub_cmd_.publish(gps_controlCmd_);
}


void PathTracking::gps_odom_callback(const nav_msgs::Odometry::ConstPtr& utm)
{
	current_point_.x = utm->pose.pose.position.x;
	current_point_.y = utm->pose.pose.position.y;
	current_point_.yaw = utm->pose.covariance[0];
}

void PathTracking::vehicleSpeed_callback(const little_ant_msgs::State2::ConstPtr& msg)
{
	if(vehicle_speed_ >20.0)
		return;
	vehicle_speed_status_ = true;
	vehicle_speed_ = msg->vehicle_speed; //  m/s
}

void PathTracking::vehicleState4_callback(const little_ant_msgs::State4::ConstPtr& msg)
{
	current_roadwheelAngle_ = msg->roadwheelAngle;
}

void PathTracking::avoiding_flag_callback(const std_msgs::Float32::ConstPtr& msg)
{
	//avoid to left(-) or right(+) the value presents the offset
	avoiding_offset_ = msg->data;
}

bool PathTracking::is_gps_data_valid(gpsMsg_t& point)
{
	if(point.x !=0 && point.y !=0)
		return true;
	return false;
}

int main(int argc,char**argv)
{
	ros::init(argc,argv,"path_tracking");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	
	PathTracking path_tracking;
	if(!path_tracking.init(nh,nh_private))
		return 1;
	path_tracking.run();
	
	ROS_INFO("path tracking completed.");
	ros::shutdown();

	return 0;
}
