#ifndef AVOIDING_H_
#define AVOIDING_H_

#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <euclidean_cluster/ObjectPolygon.h>
#include <euclidean_cluster/ObjectPolygonArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <iostream>
#include <std_msgs/Float32.h>
#include <vector>
#include <assert.h>
#include <structs.h>
#include "utils.hpp"

typedef enum
{
	SafetyArea = 0,
	AvoidingArea =1,
	DangerArea =2,
	PedestrianDetectionArea = 3
}whatArea_t;

class Avoiding
{
public:
	Avoiding();
	~Avoiding(){}
	
	bool init(ros::NodeHandle nh,ros::NodeHandle nh_private);
	bool setGlobalPath(const std::vector<gpsMsg_t>& path);
	void start();
	void stop();
	bool isRunning();
	controlCmd_t getControlCmd();
	bool updateStatus(const gpsMsg_t& pose,const float& speed, const float& roadWheelAngle);
private:
	void  avoidingThread();
	void  publishDiagnostics(uint8_t level,const std::string& msg);

private:
	enum object_type_t
	{
		Unknown = 0,
		Person = 1,
		Vehicle = 2
	};

	void get_obstacle_msg(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& objects,
						  size_t objectIndex,
						  whatArea_t *obstacleArea,
						  float ** obstacleVertex_x_y,
						  float *obstacleDistance, 
						  size_t *obstacleIndex,
						  size_t &obstacleSequence);
								   
	whatArea_t which_area(float& x,float& y);

	float deceleration_2_brakingAperture(const float & deceleration);

	float brakingAperture_2_deceleration(const float & brakingAperture);

	void bubbleSort(const float * distance, size_t * index, size_t length);
	
	float calculate_dis2path(const double& X_,const double& Y_);
	float dis2path2(const double& X_,const double& Y_);
	std::pair<double,double> vehicleToWorldCoordination(float x,float y);
	void decision(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& objects, 
				const float dis2vehicleArray[],const size_t indexArray[],const float dis2pathArray[],const int n_object);
	bool is_backToOriginalLane(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& objects, 
						const float dis2vehicleArray[],const size_t indexArray[],const float dis2pathArray[],const int& n_object);
						
	bool is_dangerous(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& objects, 
					const float dis2vehicleArray[],const size_t indexArray[],const float dis2pathArray[],const int& n_object);

	void backToOriginalLane();
	
private:
	ros::Subscriber sub_objects_;
	ros::Publisher  pub_diagnostic_;
	diagnostic_msgs::DiagnosticStatus diagnostic_msg_;
	
	std::string objects_topic_;
	std::vector<gpsMsg_t> path_points_;
	float path_points_resolution_;

	std::mutex cmd_mutex_;
	controlCmd_t cmd_;
	
	float max_deceleration_;
	
	float safety_distance_front_;
	float safety_distance_side_ ;
	
	float danger_distance_front_;
	float danger_distance_side_;
	
	float vehicle_speed_; //m/s
	float deceleration_cofficient_;
	
	size_t target_point_index_;
	size_t nearest_point_index_;
	
	gpsMsg_t current_point_,target_point_;
	
	float avoiding_offest_;
	
	float maxOffset_right_,maxOffset_left_;
	bool is_ready_; //是否准备就绪
	bool is_running_;
	
};




#endif
