#ifndef PATH_TRACKING_H_
#define PATH_TRACKING_H_
#include<ros/ros.h>
#include<little_ant_msgs/ControlCmd.h>
#include<std_msgs/Float32.h>
#include<std_msgs/UInt32.h>
#include<array_msgs/UInt32Array.h>
#include<vector>

#include<little_ant_msgs/State2.h>  //speed
#include<little_ant_msgs/State4.h>  //steerAngle
#include"gps_msgs/Inspvax.h"
#include<nav_msgs/Odometry.h> 
#include<geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include<std_msgs/Float32.h>

#include<std_msgs/Bool.h>
#include<ant_math/ant_math.h>
#include<path_tracking/State.h>
#include<climits>

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <thread>

#include<std_msgs/UInt8.h>

class PathTracking
{
public:
	PathTracking();
	~PathTracking();
	bool init(ros::NodeHandle nh,ros::NodeHandle nh_private);
	void run();
	
	float point2point_dis(gpsMsg_t &point1,gpsMsg_t &point2);
	std::pair<float, float>  get_dis_yaw(gpsMsg_t &point1,gpsMsg_t &point2);
	float dis2Points(const gpsMsg_t& point1, const gpsMsg_t& point2,bool is_sqrt=true);
	
	void pub_gps_cmd_callback(const ros::TimerEvent&);
	void gps_callback(const gps_msgs::Inspvax::ConstPtr& msg);
	
	void cartesian_gps_callback(const nav_msgs::Odometry::ConstPtr& msg);
	
	void vehicleState4_callback(const little_ant_msgs::State4::ConstPtr& msg);
	void vehicleSpeed_callback(const little_ant_msgs::State2::ConstPtr& msg);
	void avoiding_flag_callback(const std_msgs::Float32::ConstPtr& msg);

	bool is_gps_data_valid(gpsMsg_t& point);
	void rosSpinThread();

private:
	void publishMaxTolerateSpeed();	
	size_t findNearestPoint(const std::vector<gpsMsg_t>& path_points,
									 const gpsMsg_t& current_point);
	gpsMsg_t PathTracking::pointOffset(const gpsMsg_t& point,float offset)
	void publishPathTrackingState();
private:
	ros::Subscriber sub_gps_;
	
	ros::Subscriber sub_cartesian_gps_;
	
	ros::Subscriber sub_vehicleState2_;
	ros::Subscriber sub_vehicleState4_;
	
	ros::Subscriber sub_avoiding_from_lidar_;
	
	ros::Timer timer_;
	
	ros::Publisher pub_gps_cmd_;
	ros::Publisher pub_tracking_state_;
	
	ros::Publisher pub_max_tolerate_speed_;
	
	boost::shared_ptr<boost::thread> rosSpin_thread_ptr_;
	
	std::string path_points_file_;
	
	std::vector<gpsMsg_t> path_points_;
	
	path_tracking::State tracking_state_;
	
	gpsMsg_t current_point_;
	gpsMsg_t target_point_;
	
	float min_foresight_distance_;
	float disThreshold_;
	
	float avoiding_offset_;
	
	little_ant_msgs::ControlCmd gps_controlCmd_;
	
	float path_tracking_speed_;
	
	uint8_t gps_status_;
	bool vehicle_speed_status_;
	
	
	float vehicle_speed_;
	float current_roadwheelAngle_;
	
	float safety_distance_front_;
	float danger_distance_front_;
	
	float max_roadwheelAngle_;
	
	bool is_avoiding_;
	
	float lateral_err_;
	float yaw_err_;
	
	size_t target_point_index_;
	size_t nearest_point_index_;
	
	float foreSightDis_speedCoefficient_;
	float foreSightDis_latErrCoefficient_;
	
};


#endif

