#include <ros/ros.h>
#include <fstream>
#include <vector>
#include <climits>
#include <thread>
#include <mutex>

#include "auto_drive_base.hpp"
#include <std_msgs/UInt32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>

#include <ant_math/ant_math.h>
#include <driverless/TrackingState.h>


class PathTracking : public AutoDriveBase
{
public:
	PathTracking();
	virtual ~PathTracking();
	virtual bool start();
	bool setExpectSpeed(float speed);
	bool init(ros::NodeHandle nh,ros::NodeHandle nh_private);
	
	size_t getNearestPointIndex();
	bool updateStatus(const gpsMsg_t& pose,const float& speed, const float& roadWheelAngle);
	virtual bool setParkingPoints(const std::vector<parkingPoint_t>& points);

private:
	void  trackingThread();
	gpsMsg_t pointOffset(const gpsMsg_t& point,float offset);
	bool  extendGlobalPath(float extendDis);
	void  publishPathTrackingState();
	void  publishNearestIndex();
	float disToParkingPoint(const parkingPoint_t& parkingPoint);
	float limitSpeedByParkingPoint(const float& speed,const float& acc=5);
	std::pair<float, float> getDisAndYaw(const gpsMsg_t &point1, const gpsMsg_t &point2);
	inline float generateRoadwheelAngleByRadius(const float& radius);
	float limitRoadwheelAngleBySpeed(const float& angle, const float& speed);
	float generateMaxTolarateSpeedByCurvature(const std::vector<gpsMsg_t>& path_points,
											const size_t& start_search_index,
											const size_t& end_search_index,
											float max_side_accel);
	float generateMaxTolarateSpeedByCurvature(const float& curvature, const float& max_accel);
	float limitSpeedByCurrentRoadwheelAngle(float speed,float angle);
	
private:
	ros::Timer timer_;
	ros::Publisher pub_tracking_state_;
	ros::Publisher pub_nearest_index_;

	driverless::TrackingState tracking_state_;
	
	//state
	std::mutex state_mutex_;
	float expect_speed_;
	float lateral_err_;
	float yaw_err_;
	size_t target_point_index_;
	std::mutex nearest_point_index_mutex_;
	size_t nearest_point_index_;
	
	//param
	float foreSightDis_speedCoefficient_;
	float foreSightDis_latErrCoefficient_;
	float min_foresight_distance_;
	float max_target_yaw_err_; //车辆沿圆弧到达预瞄点时的航向与预瞄点航向的偏差最大值
	float disThreshold_;
	float max_side_accel_;

};
