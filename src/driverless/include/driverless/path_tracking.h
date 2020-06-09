#include<ros/ros.h>

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
#include<driverless/TrackingState.h>
#include<climits>

#include <thread>
#include <mutex>

#include "structs.h"

class PathTracking
{
public:
	PathTracking();
	~PathTracking();
	bool init(ros::NodeHandle nh,ros::NodeHandle nh_private);
	bool setPath(const std::vector<gpsMsg_t>& path);
	bool setExpectSpeed(float speed);
	void start();
	void stop();
	bool isRunning();
	controlCmd_t getControlCmd();
	bool updateStatus(const gpsMsg_t& pose,const float& speed, const float& roadWheelAngle);

private:
	void trackingThread();
	float limitSpeedByDestination(const float& speed,const float& acc=1.5);
	void publishDiagnostics(uint8_t level,const std::string& msg);
	gpsMsg_t pointOffset(const gpsMsg_t& point,float offset);
	void publishPathTrackingState();
	float disToEnd();
private:
	
	ros::Timer timer_;
	ros::Publisher pub_diagnostic_;
	ros::Publisher pub_tracking_state_;

	diagnostic_msgs::DiagnosticStatus diagnostic_msg_;
	driverless::TrackingState tracking_state_;
	
	std::vector<gpsMsg_t> path_points_;
	float path_points_resolution_;

	std::mutex cmd_mutex_;
	controlCmd_t cmd_;
	
	//state
	std::mutex state_mutex_;
	gpsMsg_t current_point_;
	float expect_speed_;
	float vehicle_speed_;
	float roadwheel_angle_;
	float lateral_err_;
	float yaw_err_;
	size_t target_point_index_;
	size_t nearest_point_index_;
	bool is_ready_; //是否准备就绪
	bool is_running_;
	
	//param
	float foreSightDis_speedCoefficient_;
	float foreSightDis_latErrCoefficient_;
	float min_foresight_distance_;
	float disThreshold_;
	float avoiding_offset_;
	float max_side_accel_;
};