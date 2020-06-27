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

enum
{
	TrafficSign_None = 0,
	TrafficSign_TrafficLight =1,
	TrafficSign_Avoid = 2,
	TrafficSign_TurnLeft = 3,
	TrafficSign_CarFollow = 4,
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
}gpsMsg_t;

typedef struct
{
	double x,y;
}point_t;


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

inline float generateDangerDistanceBySpeed(const float &speed, float max_decel)
{
	return 0.5* speed * speed /max_decel  + 3.0; 
}

inline float generateSafetyDisByDangerDis(const float &danger_dis)
{
	return danger_dis *3 + 5.0;
}

float loadPathPoints(std::string file_path,std::vector<gpsMsg_t>& points);
float calculateDis2path(const double& x,const double& y,
						 const std::vector<gpsMsg_t>& path_points, 
						 size_t   ref_point_index, //参考点索引
						 size_t * const nearest_point_index_ptr=NULL);
						 
float calculateDis2path(const double& x,const double& y,
						 const std::vector<gpsMsg_t>& path_points, 
						 size_t  ref_point_index, //参考点索引
						 size_t  max_search_index);
float generateDangerDistanceBySpeed(const float &speed);

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



#endif


