#ifndef CAR_FOLLOWING_H_
#define CAR_FOLLOWING_H_

#include <ros/ros.h>
#include "auto_drive_base.h"
#include <esr_radar/Object.h>
#include <esr_radar/ObjectArray.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
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
#include "utils.hpp"

class CarFollowing : public AutoDriveBase
{
public:
	CarFollowing();
	virtual ~CarFollowing(){};
	
	bool init(ros::NodeHandle nh,ros::NodeHandle nh_private);
	virtual bool start();
	virtual void stop();

private:
	void publishCarFollowingStats(bool status);
	void object_callback(const esr_radar::ObjectArray::ConstPtr& objects);
	void updateTimer_callback(const ros::TimerEvent&);
	void carFollowingThread();
	void publishLocalPath();
	
private:
	ros::Subscriber sub_objects_;
	ros::Publisher  pub_diagnostic_;
	ros::Publisher  pub_local_path_;

	ros::Timer      update_timer_;
	std::string     objects_topic_;

	size_t  dst_index_; //终点索引
	float   follow_distance_;
	float   safety_side_dis_;

	uint8_t targetId_;
	double  cmd_update_time_;
	
	//雷达在基坐标系的位置
	std::string base_link_frame_;
	tf::TransformListener tf_listener;
	tf::StampedTransform transform_;
	float   radar_in_base_x_;
	float   radar_in_base_y_;
	float   radar_in_base_yaw_;
	
	float max_target_search_distance_;
	int   target_repeat_threshold_; //目标重复检测次数阈值
};



#endif





