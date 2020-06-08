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
#include<path_tracking/State.h>
#include<climits>

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <thread>

#include <structs.h>

class PathTracking
{
public:
	PathTracking();
	~PathTracking();
	bool init(ros::NodeHandle nh,ros::NodeHandle nh_private);
	void run();
	
	const controlCmd_t getControlCmd() const;

	void updateStatus(const gpsMsg_t& pose,const float& speed, const float& roadWheelAngle);

	bool is_gps_data_valid(gpsMsg_t& point);
	void rosSpinThread(){ros::spin();}

private:
	float limitSpeedByDestination(const float& speed,const float& acc=1.5);
	void publishDiagnostics(uint8_t level,const std::string& msg);
	gpsMsg_t pointOffset(const gpsMsg_t& point,float offset);
	void publishPathTrackingState();
	float disToEnd();
private:
	
	ros::Timer timer_;
	
	ros::Publisher pub_diagnostic_;
	diagnostic_msgs::DiagnosticStatus diagnostic_msg_;
	
	ros::Publisher pub_tracking_state_;

	path_tracking::State tracking_state_;
	
	boost::shared_ptr<boost::thread> rosSpin_thread_ptr_;
	
	std::vector<gpsMsg_t> * const path_points_;

	float path_points_resolution_;
	
	gpsMsg_t current_point_, target_point_;
	
	float min_foresight_distance_;
	float disThreshold_;
	float avoiding_offset_;
	
	float track_speed_;
	bool vehicle_speed_status_;
	
	float vehicle_speed_;
	float current_roadwheelAngle_;
	
	float max_roadwheelAngle_;
	float max_side_accel_;

	float lateral_err_;
	float yaw_err_;
	
	size_t target_point_index_;
	size_t nearest_point_index_;
	
	float foreSightDis_speedCoefficient_;
	float foreSightDis_latErrCoefficient_;
};