#ifndef CAR_FOLLOWING_H_
#define CAR_FOLLOWING_H_

#include <ros/ros.h>
#include <esr_radar/Object.h>
#include <esr_radar/ObjectArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
#include <ant_math/ant_math.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <std_msgs/UInt8.h>
#include <vector>
#include <thread>
#include <mutex>
#include "structs.h"

class CarFollowing
{
public:
	CarFollowing();
	~CarFollowing(){};
	
	bool setGlobalPath(const std::vector<gpsMsg_t>& path);
	bool updateStatus(const gpsMsg_t& pose,const float& speed, const size_t& nearest_point_index);
	bool init(ros::NodeHandle nh,ros::NodeHandle nh_private);
	bool start();
	void stop();
	bool isRunning();
	controlCmd_t getControlCmd();

private:
	void publishCarFollowingStats(bool status);
	void object_callback(const esr_radar::ObjectArray::ConstPtr& objects);
	void updateTimer_callback(const ros::TimerEvent&);
	void carFollowingThread();
	void publishLocalPath();
	
private:
	ros::NodeHandle nh_;
	ros::NodeHandle nh_private_;
	ros::Subscriber sub_objects_;
	ros::Publisher  pub_diagnostic_;
	ros::Publisher  pub_local_path_;

	ros::Timer      update_timer_;
	std::string     objects_topic_;

	std::vector<gpsMsg_t> path_points_;
	float path_points_resolution_;

	std::mutex cmd_mutex_;
	controlCmd_t cmd_;

	//state
	std::mutex state_mutex_;
	gpsMsg_t   vehicle_pose_;
	size_t     nearest_point_index_;
	float      vehicle_speed_;
	float      roadwheel_angle_;
	bool       is_ready_; //是否准备就绪
	bool       is_running_;

	float follow_distance_;
	float safety_side_dis_;

	uint8_t targetId_;
	double  cmd_update_time_;
	
	//雷达在基坐标系的位置
	float   radar_in_base_x_;
	float   radar_in_base_y_;
	float   radar_in_base_yaw_;
	
	float max_target_search_distance_;
	diagnostic_msgs::DiagnosticStatus diagnostic_msg_;
};



#endif





