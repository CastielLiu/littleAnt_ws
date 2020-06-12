#include "ant_math/ant_math.h"

#define MAX_ROAD_WHEEL_ANGLE 25.0

const float g_vehicle_width = 1.7 ;// m
const float g_vehicle_length = 3.5; 

const float g_max_deceleration = 5.0; // m/s/s

static const float max_side_acceleration = 1.5; // m/s/s



float limitRoadwheelAngleBySpeed(const float& angle, const float& speed)
{
	float min_steering_radius = speed*speed/max_side_acceleration;
	if(min_steering_radius <3.0)  //radius = 3.0 -> steeringAngle = 30.0
		min_steering_radius = 3.0;
	
	float max_roadwheelAngle = fabs(generateRoadwheelAngleByRadius(min_steering_radius));
	if(max_roadwheelAngle > MAX_ROAD_WHEEL_ANGLE - 2.0)
	   max_roadwheelAngle = MAX_ROAD_WHEEL_ANGLE -2.0;
	//ROS_INFO("max_angle:%f\t angle:%f",max_roadwheelAngle,angle);
	return saturationEqual(angle,max_roadwheelAngle);
}

float limitSpeedByCurrentRoadwheelAngle(float speed,float angle)
{
	float steering_radius = fabs(AXIS_DISTANCE/tan(angle*M_PI/180.0));
	float max_speed =  sqrt(steering_radius*max_side_acceleration);
	
	return speed>max_speed? max_speed: speed;
}


//bool loadPathPoints(std::string file_path,std::vector<gpsMsg_t>& points)
//{
//	FILE *fp = fopen(file_path.c_str(),"r");
//	if(fp==NULL)
//	{
//		ROS_ERROR("open %s failed",file_path.c_str());
//		return false;
//	}
//	gpsMsg_t point;
//	while(!feof(fp))
//	{
//		fscanf(fp,"%lf\t%lf\t%lf\t%f\n",&point.x,&point.y,&point.yaw,&point.curvature);
//		points.push_back(point);
//	}
//	fclose(fp);
//	return true;
//}


bool loadPathPoints(std::string file_path,std::vector<gpsMsg_t>& points)
{
	std::ifstream in_file(file_path.c_str());
	if(!in_file.is_open())
	{
		ROS_ERROR("open %s failed",file_path.c_str());
		return false;
	}
	gpsMsg_t point;
	std::string line;
	
	while(in_file.good())
	{
		getline(in_file,line);
		std::stringstream ss(line);
		ss >> point.x >> point.y >> point.yaw >> point.curvature;
		points.push_back(point);
	}
	
	in_file.close();
	return true;
}


float dis2Points(const gpsMsg_t& point1, const gpsMsg_t& point2,bool is_sqrt)
{
	float x = point1.x - point2.x;
	float y = point1.y - point2.y;
	
	if(is_sqrt)
		return sqrt(x*x +y*y);
	return x*x+y*y;
}


size_t findNearestPoint(const std::vector<gpsMsg_t>& path_points, const gpsMsg_t& current_point)
{
	size_t index = 0;
	float min_dis2 = FLT_MAX;
	
	for(size_t i=0; i<path_points.size(); ++i)
	{
		float dis2 = dis2Points(path_points[i],current_point,false);
		if(dis2 < min_dis2)
		{
			min_dis2 = dis2;
			index = i;
		}
	}
	if(min_dis2 > 15*15)
	{
		ROS_ERROR("current_point x:%f\ty:%f",current_point.x,current_point.y);
		ROS_ERROR("find correct nearest point failed! the nearest point distance over 15 meters");
		return path_points.size();
	}
		
	return index;
}

float calculateDis2path(const double& x,const double& y,
						 const std::vector<gpsMsg_t>& path_points, 
						 size_t   ref_point_index, //参考点索引
						 size_t * const nearest_point_index_ptr)
{
	int searchDir; //搜索方向 -1:向后搜索， 1：向前搜索， 0 搜索完毕
	if(ref_point_index == 0)
		searchDir = 1;
	else if(ref_point_index == path_points.size())
		searchDir = -1;
	else
	{
		float dis2ref  = pow(path_points[ref_point_index].x   - x, 2) + 
					     pow(path_points[ref_point_index].y   - y, 2);
		float dis2last = pow(path_points[ref_point_index+1].x - x, 2) + 
					     pow(path_points[ref_point_index+1].y - y, 2);
		float dis2next = pow(path_points[ref_point_index-1].x - x, 2) + 
					     pow(path_points[ref_point_index-1].y - y, 2);
		if(dis2next > dis2ref && dis2last > dis2ref) 
			searchDir = 0;
		else if(dis2next > dis2ref && dis2ref > dis2last)
			searchDir = -1;
		else
			searchDir = 1;
	}

	while(ref_point_index>0 && ref_point_index<path_points.size()-1)
	{
		float dis2ref  = pow(path_points[ref_point_index].x   - x, 2) + 
							pow(path_points[ref_point_index].y   - y, 2);
		float dis2last = pow(path_points[ref_point_index+1].x - x, 2) + 
							pow(path_points[ref_point_index+1].y - y, 2);
		float dis2next = pow(path_points[ref_point_index-1].x - x, 2) + 
							pow(path_points[ref_point_index-1].y - y, 2);
		
		if(dis2next > dis2ref && dis2last > dis2ref)
			break;

		ref_point_index += searchDir;
	}
	float anchor_x,anchor_y, anchor_yaw; //锚点的位置和航向
	anchor_x = path_points[ref_point_index].x;
	anchor_y = path_points[ref_point_index].y;
	anchor_yaw = path_points[ref_point_index].yaw;

	if(nearest_point_index_ptr != NULL)
		*nearest_point_index_ptr = ref_point_index;

	return (x-anchor_x)*cos(anchor_yaw) - (y-anchor_y) * sin(anchor_yaw);
}


float limitSpeedByPathCurvature(const float& speed,const float& curvature)
{
	if(curvature == 0.0)
		return speed;
	
	float max_speed =  sqrt(1.0/fabs(curvature)*max_side_acceleration) *3.6;
	return speed>max_speed? max_speed: speed;
}

//km/h
float generateMaxTolarateSpeedByCurvature(const float& curvature, const float& max_accel)
{
	float abs_cur = fabs(curvature);
	if(abs_cur < 0.001)
		return 100.0;

	return sqrt(1.0/abs_cur*max_accel) *3.6;
}

float generateMaxTolarateSpeedByCurvature(const std::vector<gpsMsg_t>& path_points,
											const size_t& nearest_point_index,
											const size_t& target_point_index)
{
	float max_cuvature = 0.0001;
	size_t endIndex = target_point_index + 10;
	if(endIndex >= path_points.size())
		endIndex = path_points.size() -1;
		
	for(size_t i=nearest_point_index; i < endIndex; i++)
	{
		if(fabs(path_points[i].curvature) > max_cuvature)
			max_cuvature = fabs(path_points[i].curvature);
	}
	return sqrt(1.0/max_cuvature*1.5) *3.6;
}

float limitSpeedByLateralAndYawErr(float speed,float latErr,float yawErr)
{
	///??
}

//offset: change lane offset
//distance: longitudianal displacement of vehicle in the course of change lane
float maxRoadWheelAngleWhenChangeLane(const float& offset,const float& distance)
{
	float theta = 2*atan(fabs(offset)/distance);
	float radius = 0.5*distance/sin(theta);
	return generateRoadwheelAngleByRadius(radius);
}


//查找与当前点距离为dis的路径点索引
/*
 *@param path_points 路径点集
 *@param startIndex  搜索起点索引
 *@param dis         期望距离
*/
size_t findIndexForGivenDis(const std::vector<gpsMsg_t>& path_points, 
							size_t startIndex,float dis)
{
	float sum_dis = 0.0;
	for(size_t i =startIndex; i<path_points.size()-1; ++i)
	{
		sum_dis	+= disBetweenPoints(path_points[i],path_points[i+1]);
		if(sum_dis >= dis)
			return startIndex+i;
	}
	return path_points.size(); //搜索到终点扔未找到合适距离点
}

float disBetweenPoints(const gpsMsg_t& point1, const gpsMsg_t& point2)
{
	float x = point1.x - point2.x;
	float y = point1.y - point2.y;
	
	return sqrt(x*x+y*y);
}

float minCurvatureInRange(const std::vector<gpsMsg_t>& path_points, size_t startIndex,size_t endIndex)
{
	float min = FLT_MAX;
	for(size_t i=startIndex; i<endIndex; i++)
	{
		if(path_points[i].curvature < min)
			min = path_points[i].curvature;
	}
	return min;
}

float maxCurvatureInRange(const std::vector<gpsMsg_t>& path_points, size_t startIndex,float dis)
{
	float sum_dis = 0.0;
	float max_cuvature = 0.0;
	float now_cuvature;
	for(size_t i =startIndex; i<path_points.size()-1; ++i)
	{
		now_cuvature = fabs(path_points[i].curvature);
		if(max_cuvature < now_cuvature)
			max_cuvature = now_cuvature;

		sum_dis	+= disBetweenPoints(path_points[i],path_points[1+i]);
		if(sum_dis >= dis)
			break;
	}
	return max_cuvature;
}

float maxCurvatureInRange(const std::vector<gpsMsg_t>& path_points, size_t startIndex,size_t endIndex)
{
	float max = 0.0;
	for(size_t i=startIndex; i<endIndex; i++)
	{
		if(fabs(path_points[i].curvature) > max)
			max = fabs(path_points[i].curvature);
	}
	return max;
}


std::pair<float, float> get_dis_yaw(gpsMsg_t &point1,gpsMsg_t &point2)
{
	float x = point1.x - point2.x;
	float y = point1.y - point2.y;
	
	std::pair<float, float> dis_yaw;
	dis_yaw.first = sqrt(x * x + y * y);
	dis_yaw.second = atan2(x,y);
	
	if(dis_yaw.second <0)
		dis_yaw.second += 2*M_PI;
	return dis_yaw;
}


