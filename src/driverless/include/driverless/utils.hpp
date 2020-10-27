#ifndef UTILS_H_
#define UTILS_H_

#include "structs.h"
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

/*@param 从文件载入路径点
 *@return 路径点分辨率,0表示载入失败
 */
static float loadPathPoints(std::string file_path,std::vector<gpsMsg_t>& points)
{
	std::ifstream in_file(file_path.c_str());
	if(!in_file.is_open())
	{
		ROS_ERROR("open %s failed",file_path.c_str());
		return 0.0;
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
	float reslution = 0.1;
	return reslution;
}

static float saturationEqual(float value,float limit)
{
	assert(limit>=0);
	if(value > limit) value = limit;
	else if(value < -limit) value = -limit;
	return value;
}

static float dis2Points(const gpsMsg_t& point1, const gpsMsg_t& point2,bool is_sqrt)
{
	float x = point1.x - point2.x;
	float y = point1.y - point2.y;
	
	if(is_sqrt)
		return sqrt(x*x +y*y);
	return x*x+y*y;
}

static size_t findNearestPoint(const std::vector<gpsMsg_t>& path_points, const gpsMsg_t& current_point)
{
	size_t index = 0;
	float min_dis2 = FLT_MAX;
	
	for(size_t i=0; i<path_points.size(); ++i)
	{
		float yawErr = path_points[i].yaw-current_point.yaw;
		if(yawErr > M_PI)
			yawErr -= 2*M_PI;
		else if(yawErr < -M_PI)
			yawErr += 2*M_PI;
				
		if(fabs(yawErr) > M_PI/2)
			continue;
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
		std::cout <<index << std::endl;
	return index;
}

/*@brief 计算目标点到达路径的距离,点在路径左侧为负,右侧为正
 *@brief 该函数主要用于计算主体车辆到路径的距离(横向偏差),并更新最近点索引
 *@param x,y         目标点坐标
 *@param path_points 路径点集
 *@param ref_point_index 参考点索引，以此参考点展开搜索，加速计算
 *@param nearest_point_index_ptr 输出与目标点最近的路径点索引(可选参数)
 */
static float calculateDis2path(const double& x,const double& y,
						 const std::vector<gpsMsg_t>& path_points, 
						 size_t   ref_point_index, //参考点索引
						 size_t * const nearest_point_index_ptr)
{
	int searchDir; //搜索方向 -1:向后搜索， 1：向前搜索， 0 搜索完毕
	if(ref_point_index == 0)
	{
		ref_point_index = 1;
		searchDir = 1;
	}
	else if(ref_point_index == path_points.size()-1)
	{
		ref_point_index = path_points.size()-2;
		searchDir = -1;
	}
	else
	{
		float dis2ref  = pow(path_points[ref_point_index].x   - x, 2) + 
					     pow(path_points[ref_point_index].y   - y, 2);
		float dis2last = pow(path_points[ref_point_index-1].x - x, 2) + 
					     pow(path_points[ref_point_index-1].y - y, 2);
		float dis2next = pow(path_points[ref_point_index+1].x - x, 2) + 
					     pow(path_points[ref_point_index+1].y - y, 2);
		if(dis2next > dis2ref && dis2last > dis2ref) 
			searchDir = 0;
		else if(dis2next > dis2ref && dis2ref > dis2last)
			searchDir = -1;
		else
			searchDir = 1;
	}
	
	//std::cout  <<  "searchDir:"  << "\t" << searchDir << "\r\n";
	while(ref_point_index>0 && ref_point_index<path_points.size()-1)
	{
		float dis2ref  = pow(path_points[ref_point_index].x   - x, 2) + 
							pow(path_points[ref_point_index].y   - y, 2);
		float dis2last = pow(path_points[ref_point_index-1].x - x, 2) + 
							pow(path_points[ref_point_index-1].y - y, 2);
		float dis2next = pow(path_points[ref_point_index+1].x - x, 2) + 
							pow(path_points[ref_point_index+1].y - y, 2);
	//std::cout  << ref_point_index << "\t" <<  sqrt(dis2last)  << "\t" << sqrt(dis2ref) << "\t" << sqrt(dis2next) << "\r\n";		
		if((searchDir == 1 && dis2next > dis2ref) ||
		   (searchDir ==-1 && dis2last > dis2ref) ||
		   (searchDir == 0))
			break;

		ref_point_index += searchDir;
	}
	float anchor_x,anchor_y, anchor_yaw; //锚点的位置和航向
	anchor_x = path_points[ref_point_index].x;
	anchor_y = path_points[ref_point_index].y;
	anchor_yaw = path_points[ref_point_index].yaw;

	if(nearest_point_index_ptr != NULL)
		*nearest_point_index_ptr = ref_point_index;
	//float dis2anchor = sqrt((x-anchor_x)*(x-anchor_x)+(y-anchor_y)*(y-anchor_y));
	float dx = (x-anchor_x)*cos(anchor_yaw) - (y-anchor_y) * sin(anchor_yaw);
	return dx;
}

/*@brief 计算目标点到达路径的距离,点在路径左侧为负,右侧为正
 *@brief 该函数主要用于计算目标到路径的距离,并考虑路径终点问题
 *@param x,y         目标点坐标
 *@param path_points 路径点集
 *@param ref_point_index 参考点索引，以此参考点展开搜索，加速计算
 *@param max_search_index 最大搜索索引,超出此索引的目标物则输出距离为FLT_MAX
 */
static float calculateDis2path(const double& x,const double& y,
						 const std::vector<gpsMsg_t>& path_points, 
						 size_t  ref_point_index, //参考点索引
						 size_t  max_search_index)
{
	int searchDir; //搜索方向 -1:向后搜索， 1：向前搜索， 0 搜索完毕
	if(ref_point_index == 0)
	{
		ref_point_index = 1;
		searchDir = 1;
	}
	else if(ref_point_index == path_points.size()-1)
	{
		ref_point_index = path_points.size()-2;
		searchDir = -1;
	}
	else
	{
		float dis2ref  = pow(path_points[ref_point_index].x   - x, 2) + 
					     pow(path_points[ref_point_index].y   - y, 2);
		float dis2last = pow(path_points[ref_point_index-1].x - x, 2) + 
					     pow(path_points[ref_point_index-1].y - y, 2);
		float dis2next = pow(path_points[ref_point_index+1].x - x, 2) + 
					     pow(path_points[ref_point_index+1].y - y, 2);
		if(dis2next > dis2ref && dis2last > dis2ref) 
			searchDir = 0;
		else if(dis2next > dis2ref && dis2ref > dis2last)
			searchDir = -1;
		else
			searchDir = 1;
	}
	
	//std::cout  <<  "searchDir:"  << "\t" << searchDir << "\r\n";
	while(ref_point_index>0 && ref_point_index<path_points.size()-1)
	{
		float dis2ref  = pow(path_points[ref_point_index].x   - x, 2) + 
							pow(path_points[ref_point_index].y   - y, 2);
		float dis2last = pow(path_points[ref_point_index-1].x - x, 2) + 
							pow(path_points[ref_point_index-1].y - y, 2);
		float dis2next = pow(path_points[ref_point_index+1].x - x, 2) + 
							pow(path_points[ref_point_index+1].y - y, 2);
	//std::cout  << ref_point_index << "\t" <<  sqrt(dis2last)  << "\t" << sqrt(dis2ref) << "\t" << sqrt(dis2next) << "\r\n";		
		if((searchDir == 1 && dis2next > dis2ref) ||
		   (searchDir ==-1 && dis2last > dis2ref) ||
		   (searchDir == 0))
			break;

		ref_point_index += searchDir;
	}
	float anchor_x,anchor_y, anchor_yaw; //锚点的位置和航向
	anchor_x = path_points[ref_point_index].x;
	anchor_y = path_points[ref_point_index].y;
	anchor_yaw = path_points[ref_point_index].yaw;
	
	//若参考索引大于最大搜索索引,且目标物与车辆纵向距离大于一定阈值,则输出 FLT_MAX
	if(ref_point_index >= max_search_index)
	{
		anchor_x = path_points[max_search_index].x;
		anchor_y = path_points[max_search_index].y;
		anchor_yaw = path_points[max_search_index].yaw;
		float dy = (x-anchor_x)*sin(anchor_yaw) + (y-anchor_y) * cos(anchor_yaw);
		//float dx = (x-anchor_x)*cos(anchor_yaw) - (y-anchor_y) * sin(anchor_yaw);
	//printf("dx:%.2f\tdy:%.2f\tref_point_index:%d\tmax_search_index:%d\n",dx,dy,ref_point_index,max_search_index);
		
		if(dy > 0.5)
			return FLT_MAX;
	}
	
	//printf("dx:%.2f\tdy:%.2f\tref_point_index:%d\n",dx,dy,ref_point_index);
	
	return (x-anchor_x)*cos(anchor_yaw) - (y-anchor_y) * sin(anchor_yaw);
}

static float limitSpeedByLateralAndYawErr(float speed,float latErr,float yawErr)
{
	//
}

static float disBetweenPoints(const gpsMsg_t& point1, const gpsMsg_t& point2)
{
	float x = point1.x - point2.x;
	float y = point1.y - point2.y;
	
	return sqrt(x*x+y*y);
}

//查找与当前点距离为dis的路径点索引
/*
 *@param path_points 路径点集
 *@param startIndex  搜索起点索引
 *@param dis         期望距离
*/
static size_t findIndexForGivenDis(const std::vector<gpsMsg_t>& path_points, 
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

static float minCurvatureInRange(const std::vector<gpsMsg_t>& path_points, size_t startIndex,size_t endIndex)
{
	float min = FLT_MAX;
	for(size_t i=startIndex; i<endIndex; i++)
	{
		if(path_points[i].curvature < min)
			min = path_points[i].curvature;
	}
	return min;
}
/*@brief 搜索从startIndex开始到dis距离区间的最大曲率
 *@brief剩余距离小于期望距离时输出剩余部分的最大曲率
*/
static float maxCurvatureInRange(const std::vector<gpsMsg_t>& path_points, size_t startIndex,float dis)
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

static float maxCurvatureInRange(const std::vector<gpsMsg_t>& path_points, size_t startIndex,size_t endIndex)
{
	float max = 0.0;
	for(size_t i=startIndex; i<endIndex; i++)
	{
		if(fabs(path_points[i].curvature) > max)
			max = fabs(path_points[i].curvature);
	}
	return max;
}


/*@brief local2global坐标变换
 *@param origin_x,origin_y   局部坐标系在全局坐标下的位置
 *@param theta 局部坐标系在全局坐标下的角度
 *@param local_x,local_y   点在局部坐标系下的坐标
 *@return      点在全局坐标系下的坐标
 */
static std::pair<float, float> 
local2global(float origin_x,float origin_y,float theta, float local_x,float local_y)
{
	std::pair<float, float> global;
	global.first  = local_x*cos(theta) - local_y*sin(theta) + origin_x;
	global.second = local_x*sin(theta) + local_y*cos(theta) + origin_y;
	return global;
}

/*@brief global2local坐标变换
 *@param origin_x,origin_y   局部坐标系在全局坐标下的位置
 *@param theta 局部坐标系在全局坐标下的角度
 *@param local_x,local_y   点在局部坐标系下的坐标
 *@return      点在全局坐标系下的坐标
 */
static std::pair<float, float> 
global2local(float origin_x,float origin_y,float theta, float global_x,float global_y)
{
	std::pair<float, float> local;
	local.first  = (global_x-origin_x)*cos(theta) + (global_y-origin_y)*sin(theta);
	local.second = -(global_x-origin_x)*sin(theta) + (global_y-origin_y)*cos(theta);
	return local;
}

/*@brief 坐标变换
 *@param origin   局部坐标系原点在全局坐标下的位置
 *@param theta    局部坐标系在全局坐标下的角度
 *@param local   点在局部坐标系下的位置
 *@return        点在全局坐标系下的位置
 */
static point_t coordinationConvert(point_t origin,float theta, point_t local)
{
	point_t global;
	global.x = local.x*cos(theta) - local.y*sin(theta) + origin.x;
	global.y = local.x*sin(theta) + local.y*cos(theta) + origin.y;
	return global;
}



#endif