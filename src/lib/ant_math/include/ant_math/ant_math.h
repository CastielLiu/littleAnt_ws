#ifndef ANT_MATH_H_
#define ANT_MATH_H_

#include<cstring>
#include<cmath>
#include<assert.h>
#include<string>
#include<vector>
#include<cstdio>
#include<ros/ros.h>
#include<limits.h>
#include<exception>
#include<fstream>

#define IS_POLAR_COORDINATE_GPS 0

#define WHEEL_DISTANCE  1.2
#define AXIS_DISTANCE  1.5
#define MAX_SPEED 30.0

enum
{
	TrafficSign_None = 0,
	TrafficSign_TrafficLight =1,
	TrafficSign_Avoid = 2,
	TrafficSign_TurnLeft = 3,
	TrafficSign_CarFollow = 4,//?
	TrafficSign_LaneNarrow = 5,
	TrafficSign_IllegalPedestrian = 6,
	TrafficSign_NoTrafficLight = 7,
	TrafficSign_PickUp = 8,
	TrafficSign_Ambulance = 9,//?
	TrafficSign_Railway = 10,
	TrafficSign_TempStop = 11,//?
	TrafficSign_UTurn = 12,
	TrafficSign_School = 13,
	TrafficSign_AvoidStartingCar = 14,
	TrafficSign_OffDutyPerson = 15,
	TrafficSign_Bridge = 16,
	TrafficSign_AccidentArea = 17,
	TrafficSign_JamArea = 18,
	TrafficSign_BusStop = 19,
	TrafficSign_NonVehicle = 20,
	TrafficSign_StopArea = 21, //?
	
	TrafficSign_CloseTurnLight = 22,
	TrafficSign_TurnRight = 23,
	TrafficSign_Stop = 24,
};

typedef struct
{
	double longitude;
	double latitude;
	double yaw;
	double x;
	double y;
	float curvature;
	
	float maxOffset_left;
	float maxOffset_right;
	uint8_t traffic_sign;
	uint8_t other_info;
	
}gpsMsg_t;

typedef struct
{
	double x,y;
}point_t;

extern const float g_vehicle_width;
extern const float g_vehicle_length;
extern const float g_max_deceleration;


inline float generateRoadwheelAngleByRadius(const float& radius)
{
	assert(radius!=0);
	//return asin(AXIS_DISTANCE /radius)*180/M_PI;  //the angle larger
	return atan(AXIS_DISTANCE/radius)*180/M_PI;    //correct algorithm 
}

inline double sinDeg(const double& deg)
{
	return sin(deg*M_PI/180.0);
}

inline float saturationEqual(float value,float limit)
{
	//ROS_INFO("value:%f\t limit:%f",value,limit);
	assert(limit>=0);
	if(value>limit)
		value = limit;
	else if(value < -limit)
		value = -limit;
	return value;
}

inline int sign(float num)
{
	return num > 0? 1 : -1;
}

inline float deg2rad(float deg)
{
	return  (deg/180.0)*M_PI;
}

inline float generateDangerDistanceBySpeed(const float &speed)
{
	return 0.5* speed * speed /g_max_deceleration  + 3.0; 
}

inline float generateSafetyDisByDangerDis(const float &danger_dis)
{
	return danger_dis *3 + 5.0;
}

float limitRoadwheelAngleBySpeed(const float& angle, const float& speed);
float limitSpeedByPathCurvature(const float& speed,const float& curvature);
float limitSpeedByCurrentRoadwheelAngle(float speed,float angle);
bool  loadPathPoints(std::string file_path,std::vector<gpsMsg_t>& points);
float calculateDis2path(const double& x,const double& y,
						 const std::vector<gpsMsg_t>& path_points, 
						 size_t   ref_point_index, //参考点索引
						 size_t * const nearest_point_index_ptr=NULL);
						 
float calculateDis2path(const double& x,const double& y,
						 const std::vector<gpsMsg_t>& path_points, 
						 size_t  ref_point_index, //参考点索引
						 size_t  max_search_index);
float maxRoadWheelAngleWhenChangeLane(const float& offset,const float& distance);
float generateDangerDistanceBySpeed(const float &speed);
float generateMaxTolarateSpeedByCurvature(const float& curvature, const float& max_accel);
float generateMaxTolarateSpeedByCurvature(const std::vector<gpsMsg_t>& path_points,
											const size_t& nearest_point_index,
											const size_t& target_point_index);

float disBetweenPoints(const gpsMsg_t& point1, const gpsMsg_t& point2);
size_t findIndexForGivenDis(const std::vector<gpsMsg_t>& path_points, size_t startIndex,float dis);
float minCurvatureInRange(const std::vector<gpsMsg_t>& path_points, size_t startIndex,size_t endIndex);
float maxCurvatureInRange(const std::vector<gpsMsg_t>& path_points, size_t startIndex,size_t endIndex);
float maxCurvatureInRange(const std::vector<gpsMsg_t>& path_points, size_t startIndex,float dis);
std::pair<float, float> coordinationConvert(float X,float Y,float Theta, float x,float y);
std::pair<float, float> local2global(float origin_x,float origin_y,float theta,float local_x,float local_y);
std::pair<float, float> global2local(float origin_x,float origin_y,float theta,float global_x,float global_y);

float dis2Points(const gpsMsg_t& point1, const gpsMsg_t& point2,bool is_sqrt=true);
size_t findNearestPoint(const std::vector<gpsMsg_t>& path_points, const gpsMsg_t& current_point);
std::pair<float, float> get_dis_yaw(gpsMsg_t &point1,gpsMsg_t &point2);



#endif


