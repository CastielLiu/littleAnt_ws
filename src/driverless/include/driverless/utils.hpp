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

/*@brief 角度归一化，(-pi, pi]
 */
static double normalizeRadAngle(double angle)
{
	double res = angle;
	while(res <= -M_PI)
		res += 2*M_PI;

	while(res > M_PI)
		res -= 2*M_PI;
	return res;
}

/*@brief 获取两点间的距离以及航向
 *@param point1 终点
 *@param point2 起点
 */
static std::pair<float, float> getDisAndYaw(const Pose& point1, const Pose& point2)
{
	float x = point1.x - point2.x;
	float y = point1.y - point2.y;
	
	std::pair<float, float> dis_yaw;
	dis_yaw.first = sqrt(x * x + y * y);
	dis_yaw.second = atan2(y,x);
	
	if(dis_yaw.second <0)
		dis_yaw.second += 2*M_PI;
	return dis_yaw;
}

static double max(double val_1, double val_2)
{
	return val_1 > val_2 ? val_1 : val_2;
}

/*@brief 获取两点间的航向
 *@param point1 终点
 *@param point2 起点
 */
static float getYaw(const Point& point1, const Point& point2)
{
	float yaw = atan2(point1.y - point2.y, point1.x - point2.x);
	
	if(yaw <0) yaw += 2*M_PI;
	return yaw;
}

/*@brief 获取两点间的距离
 *@param point1 终点
 *@param point2 起点
 */
static float getDistance(const Point& point1, const Point& point2)
{
	float x = point1.x - point2.x;
	float y = point1.y - point2.y;
	return sqrt(x * x + y * y);
}

/*@brief 计算路径各离散点处的曲率
 *@param path 引用原始路径
 */
static bool calPathCurvature(Path& path)
{
	if(path.has_curvature) //路径已经包含曲率信息，无需重复计算
		return true;
	auto& points = path.points;
	size_t size = points.size();
	for(int i=0; i<size-1; ++i)
	{
		float delta_theta = normalizeRadAngle(points[i+1].yaw - points[i].yaw); //旋转角
		float arc_length  = getDistance(points[i+1], points[i]); //利用两点间距近似弧长
		if(arc_length == 0)
			points[i].curvature = 0.0; //绝对值偏大
		else
			points[i].curvature = delta_theta/arc_length; //绝对值偏大
	}
	
	//均值滤波
	int n = 10;
	float curvature_n_sum = 0.0;
	for(int i=0; i < size; ++i)
	{
		if(i<n)
			curvature_n_sum+=points[i].curvature;
		else
		{
			points[i-n/2].curvature = curvature_n_sum/n;
			curvature_n_sum += (points[i].curvature - points[i-n].curvature);
			//std::cout << std::fixed << std::setprecision(2) << points[i].curvature << std::endl;
		}
	}
	
	path.has_curvature = true;
	return true;
}

/*@param 从文件载入路径点,包括位置，航向，以及路径曲率
 *       若文件中不包含曲率信息，则调用曲率计算函数进行计算
 *@return false: 载入失败
 */
static bool loadPathPoints(std::string file_path,Path& path)
{
	std::ifstream in_file(file_path.c_str());
	if(!in_file.is_open())
	{
		ROS_ERROR("LoadPathPoints: Open %s failed",file_path.c_str());
		return false;
	}
	GpsPoint point;
	std::string line;
	path.clear();  //首先清除历史路径点信息

	bool has_curvature = false;
	while(in_file.good())
	{
		getline(in_file,line);
		if(line.length() == 0)  //处理当文件为空或者末尾空行的情况
			break;

		std::stringstream ss(line);
		ss >> point.x >> point.y >> point.yaw >> point.curvature;
		if(!has_curvature && point.curvature!=0)
			has_curvature = true;
		path.points.push_back(point);
	}
	in_file.close();

	if(path.points.size() == 0)
		return false;

	float resolution = 0.1;   //实则应从文件读取
	path.resolution  = resolution;
	path.final_index = path.points.size() - 1 ;  //设置终点索引为最后一个点
	path.has_curvature = has_curvature;
	if(!has_curvature)
		calPathCurvature(path);

	//算法根据停车点距离控制车速，若没有附加路径信息将导致到达终点前无法减速停车！
	//因此，此处将终点设为一个永久停车点，
	path.park_points.points.emplace_back(path.final_index, 0.0); 
	return true;
}

/*@brief 从xml文件载入路径信息
 *@1. 停车点-若文件不包含终点信息，手动添加√
 *@2. 转向区间-控制转向灯　
 *@3. 
*/
#include <tinyxml2.h>
static bool loadPathAppendInfos(const std::string& file, Path& global_path, const std::string& user)
{
	if(global_path.size() == 0)
	{
		ROS_ERROR("[%s] please load global path points first!",user.c_str());
		return false;
	}

	ParkingPoints& park_points = global_path.park_points;
	TurnRanges&    turn_ranges = global_path.turn_ranges;
	
	using namespace tinyxml2;
	XMLDocument Doc;  
	XMLError res = Doc.LoadFile(file.c_str());
	
	if(XML_ERROR_FILE_NOT_FOUND == res)
	{
		ROS_ERROR_STREAM("[" << user <<"] " << "path infomation file: "<< file << " not exist!");
		return false;
	}
	else if(XML_SUCCESS != res)
	{
		ROS_ERROR_STREAM("[" << user <<"] " << "path infomation file: "<< file << " parse error!");
		return false;
	}
	tinyxml2::XMLElement *pRoot=Doc.RootElement();//根节点
	if(pRoot == nullptr)
	{
		ROS_ERROR_STREAM("[" << user <<"] " << "path infomation file: "<< file << " no root node!");
		return false;
	}
	tinyxml2::XMLElement *pParkingPoints = pRoot->FirstChildElement("ParkingPoints"); //一级子节点
	if(pParkingPoints)
	{
		bool has_dst_parking_point = false;//是否有终点
		tinyxml2::XMLElement *pParkingPoint = pParkingPoints->FirstChildElement("ParkingPoint"); //二级子节点
		while (pParkingPoint)
		{
			uint32_t id    = pParkingPoint->Unsigned64Attribute("id");
			uint32_t index = pParkingPoint->Unsigned64Attribute("index");
			float duration = pParkingPoint->FloatAttribute("duration");
			park_points.points.emplace_back(index,duration);
			//std::cout << id << "\t" << index << "\t" << duration << std::endl;
			if(duration == 0)
				has_dst_parking_point = true;
			//转到下一子节点
			pParkingPoint = pParkingPoint->NextSiblingElement("ParkingPoint");  
		}

		//如果路径信息中不包含终点停车点，手动添加路径终点为停车点
		if(!has_dst_parking_point)
			park_points.points.emplace_back(global_path.final_index, 0.0); 
		
		park_points.sort();  //停车点小到大排序
		park_points.print(user); //打印到终端显示
			
		ROS_INFO("[%s] load Parking Points ok.",user.c_str());
	}
	else
		ROS_INFO("[%s] No Parking Points in path info file!",user.c_str());

	tinyxml2::XMLElement *pTurnRanges = pRoot->FirstChildElement("TurnRanges"); //一级子节点
	if(pTurnRanges)
	{
		tinyxml2::XMLElement *pTurnRange = pTurnRanges->FirstChildElement("TurnRange"); //二级子节点
		while (pTurnRange)
		{
			int    type  = pTurnRange->IntAttribute("type");
			size_t start = pTurnRange->Unsigned64Attribute("start");
			size_t end   = pTurnRange->Unsigned64Attribute("end");
			turn_ranges.ranges.emplace_back(type,start,end);
			//std::cout << type << "\t" << start << "\t" << end << std::endl;
			
			//转到下一子节点
			pTurnRange = pTurnRange->NextSiblingElement("TurnRange"); 
		}
		for(auto &range : turn_ranges.ranges)
			ROS_INFO("[%s] turn range: type:%d  start:%lu  end:%lu", user.c_str(), range.type,range.start_index, range.end_index);
		
		ROS_INFO("[%s] load turn ranges ok.",user.c_str());
	}
	else
		ROS_INFO("[%s] No tutn ranges in path info file!",user.c_str());
	return true;
}

/*@brief 拓展路径,防止车辆临近终点时无法预瞄
 *@param path  需要拓展的路径
 *@param extendDis 拓展长度，保证实际终点距离虚拟终点大于等于extendDis
 */
static bool extendPath(Path& path, float extendDis)
{
	std::vector<GpsPoint>& path_points = path.points;
	//取最后一个点与倒数第n个点的连线向后插值
	//总路径点不足n个,退出
	int n = 5;
	//std::cout << "extendPath: " << path_points.size() << "\t" << path_points.size()-1 << std::endl;
	if(path_points.size()-1 < n)
	{
		ROS_ERROR("path points is too few (%lu), extend path failed",path_points.size()-1);
		return false;
	}
	int endIndex = path_points.size()-1;
	
	float dx = (path_points[endIndex].x - path_points[endIndex-n].x)/n;
	float dy = (path_points[endIndex].y - path_points[endIndex-n].y)/n;
	float ds = sqrt(dx*dx+dy*dy);

	GpsPoint point;
	float remaind_dis = 0.0;
	for(size_t i=1;;++i)
	{
		point.x = path_points[endIndex].x + dx*i;
		point.y = path_points[endIndex].y + dy*i;
		point.curvature = 0.0;
		path_points.push_back(point);
		remaind_dis += ds;
		if(remaind_dis > extendDis)
			break;
	}
	return true;
}

static float saturationEqual(float value,float limit)
{
	assert(limit>=0);
	if(value > limit) value = limit;
	else if(value < -limit) value = -limit;
	return value;
}

static float dis2Points(const Pose& point1, const Pose& point2,bool is_sqrt)
{
	float x = point1.x - point2.x;
	float y = point1.y - point2.y;
	
	if(is_sqrt)
		return sqrt(x*x +y*y);
	return x*x+y*y;
}

/*@brief 在目标路径path中查找距离pose最近的点，返回该点的索引值
	滤除与当前航向偏差大于90度的点，当未查找到最近点或最近点过远时返回路径最大索引值
 *@param path 目标路径
 *@param pose 目标点
*/
static size_t findNearestPoint(const Path& path, const Pose& pose)
{
	size_t index = 0;
	float min_dis2 = FLT_MAX;

	const std::vector<GpsPoint>& path_points = path.points;
	
	for(size_t i=0; i<path_points.size(); ++i)
	{
		float yawErr = path_points[i].yaw-pose.yaw;
		if(yawErr > M_PI)
			yawErr -= 2*M_PI;
		else if(yawErr < -M_PI)
			yawErr += 2*M_PI;
				
		if(fabs(yawErr) > M_PI/2)
			continue;
		float dis2 = dis2Points(path_points[i],pose,false);
		if(dis2 < min_dis2)
		{
			min_dis2 = dis2;
			index = i;
		}
	}
	if(min_dis2 > 15*15)
	{
		ROS_ERROR("[findNearestPoint] current_pose x:%f\ty:%f",pose.x,pose.y);
		ROS_ERROR("[findNearestPoint] find correct nearest point failed! the nearest point distance over 15 meters");
		return path_points.size();
	}
		std::cout <<index << std::endl;
	return index;
}

/*@brief 计算目标点到达路径的距离,点在路径左侧为负,右侧为正
 *@brief 该函数主要用于计算主体车辆到路径的距离(横向偏差),并更新最近点索引
 *@param x,y         目标点坐标
 *@param path        路径点集
 *@param ref_point_index 参考点索引，以此参考点展开搜索，加速计算
 *@param nearest_point_index_ptr 输出与目标点最近的路径点索引(可选参数)
 */
static float calculateDis2path(const double& x,const double& y,
						 const Path& path, 
						 size_t   ref_point_index, //参考点索引
						 size_t * const nearest_point_index_ptr=NULL)
{
	const std::vector<GpsPoint>& path_points = path.points;

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
//	std::cout  << ref_point_index << "\t" <<  sqrt(dis2last)  << "\t" << sqrt(dis2ref) << "\t" << sqrt(dis2next) << "\r\n";		
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
 *@param path        路径
 *@param ref_point_index 参考点索引，以此参考点展开搜索，加速计算
 *@param max_search_index 最大搜索索引,超出此索引的目标物则输出距离为FLT_MAX
 */
static float calculateDis2path(const double& x,const double& y,
						 const Path& path, 
						 size_t  ref_point_index, //参考点索引
						 size_t  max_search_index)
{
	const std::vector<GpsPoint>& path_points = path.points;

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

static float disBetweenPoints(const GpsPoint& point1, const GpsPoint& point2)
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
static size_t findIndexForGivenDis(const std::vector<GpsPoint>& path_points, 
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

static float minCurvatureInRange(const std::vector<GpsPoint>& path_points, size_t startIndex,size_t endIndex)
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
static float maxCurvatureInRange(const Path& path, size_t startIndex,float dis)
{
	float sum_dis = 0.0;
	float max_cuvature = 0.0;
	float now_cuvature;
	const std::vector<GpsPoint>& path_points = path.points;
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

static float maxCurvatureInRange(const Path& path,  size_t startIndex,size_t endIndex)
{
	float max = 0.0;
	const std::vector<GpsPoint>& path_points = path.points;
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
 *@param origin   局部坐标系原点在全局坐标下的位姿
 *@param local   点在局部坐标系下的位置
 *@return        点在全局坐标系下的位置
 */
static Point local2global(const Pose& origin, const Point& local)
{
	Point global;
	global.x = local.x*cos(origin.yaw) - local.y*sin(origin.yaw) + origin.x;
	global.y = local.x*sin(origin.yaw) + local.y*cos(origin.yaw) + origin.y;
	return global;
}

static Point global2local(const Pose& origin, const Point& global)
{
	Point local;
	local.x  = (global.x-origin.x)*cos(origin.yaw) + (global.y-origin.y)*sin(origin.yaw);
	local.y = -(global.x-origin.x)*sin(origin.yaw) + (global.y-origin.y)*cos(origin.yaw);
	return local;
}


/*@brief 利用转弯半径计算前轮转角
 *@param radius 转弯半径
 *@return 前轮转角
 */
static float generateRoadwheelAngleByRadius(float wheel_base, float radius)
{
	assert(radius!=0);
	//return asin(vehicle_params_.wheelbase /radius)*180/M_PI;  //the angle larger
	return atan(wheel_base/radius)*180/M_PI;    //correct algorithm 
}

#endif
