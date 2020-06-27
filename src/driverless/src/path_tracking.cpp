#include "driverless/path_tracking.h"
#define __NAME__ "path_tracking"

PathTracking::PathTracking():
	AutoDriveBase(__NAME__),
	nearest_point_index_(0),
	expect_speed_(10.0) //defult expect speed
{
	path_points_resolution_ = 0.1;
}

PathTracking::~PathTracking()
{
}

bool PathTracking::setExpectSpeed(float speed)
{
	expect_speed_ = speed;
}

bool PathTracking::updateStatus(const gpsMsg_t& pose,const float& speed, const float& roadWheelAngle)
{
	if(!is_ready_) is_ready_ = true;
	std::lock_guard<std::mutex> lock(state_mutex_);
	vehicle_pose_ = pose;
	vehicle_speed_ = speed;
	roadwheel_angle_ = roadWheelAngle;
}

bool PathTracking::init(ros::NodeHandle nh,ros::NodeHandle nh_private)
{
	std::string tracking_info_topic = 
	nh_private.param<std::string>("tracking_info_topic","/tracking_state");
	nh_private.param<float>("foreSightDis_speedCoefficient", foreSightDis_speedCoefficient_,1.0);
	nh_private.param<float>("foreSightDis_latErrCoefficient", foreSightDis_latErrCoefficient_,-3.0);
	nh_private.param<float>("min_foresight_distance",min_foresight_distance_,5.0);
	nh_private.param<float>("max_side_accel",max_side_accel_,1.0);
	
	max_target_yaw_err_ = nh_private.param<float>("max_target_yaw_err",50.0)*M_PI/180.0;

	pub_tracking_state_ = nh.advertise<driverless::TrackingState>(tracking_info_topic,1);
	pub_nearest_index_  = nh.advertise<std_msgs::UInt32>("/driverless/nearest_index",1);

	initDiagnosticPublisher(nh,__NAME__);
		
	return true;
}

//启动跟踪线程
bool PathTracking::start()
{
	is_running_ = false;
	if(path_points_.size()==0)
	{
		ROS_ERROR("[%s] please set global path first",__NAME__);
		return false;
	}
	if(parking_points_.size()==0)
	{
		ROS_ERROR("[%s] please set parking points first",__NAME__);
		return false;
	}
	if(!extendGlobalPath(20.0)) 
		return false;

	if(vehicle_.validity == false)
	{
		ROS_ERROR("[%s] Vehicle parameters is invalid, please set them firstly.",__NAME__);
		return false;
	}

	is_running_ = true;
	std::thread t(&PathTracking::trackingThread,this);
	t.detach();
	return true;
}

/*@brief 拓展全局路径,防止车辆临近终点时无法预瞄
 *@param extendDis 拓展长度，保证实际终点距离虚拟终点大于等于extendDis
 */
bool PathTracking::extendGlobalPath(float extendDis)
{
	//取最后一个点与倒数第n个点的连线向后插值
	//总路径点不足n个,退出
	int n = 5;
	//std::cout << "extendGlobalPath: " << path_points_.size() << "\t" << path_points_.size()-1 << std::endl;
	if(path_points_.size()-1 < n)
	{
		ROS_ERROR("[%s] global path points is too few (%d), extend global path failed",path_points_.size()-1, __NAME__);
		return false;
	}
	int endIndex = path_points_.size()-1;
	
	float dx = (path_points_[endIndex].x - path_points_[endIndex-n].x)/n;
	float dy = (path_points_[endIndex].y - path_points_[endIndex-n].y)/n;
	float ds = sqrt(dx*dx+dy*dy);

	gpsMsg_t point;
	float remaindDis = 0.0;
	for(size_t i=1;;++i)
	{
		point.x = path_points_[endIndex].x + dx*i;
		point.y = path_points_[endIndex].y + dy*i;
		point.curvature = 0.0;
		path_points_.push_back(point);
		remaindDis += ds;
		if(remaindDis > extendDis)
			break;
	}
	return true;
}

//跟踪线程
void PathTracking::trackingThread()
{
	while(ros::ok() && !is_ready_) //等待就绪
		usleep(500000);
	nearest_point_index_ = findNearestPoint(path_points_,vehicle_pose_); 

	if(nearest_point_index_ > path_points_.size() - 10)
	{
		ROS_ERROR("Remaind target path is too short! nearest_point_index:%d",nearest_point_index_);
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::ERROR,"Remaind target path is too short!");
		is_running_ = false;
		return ;
	}

	size_t i =0;
	ros::Rate loop_rate(30);

	float vehicle_speed,roadwheel_angle;
	gpsMsg_t now_point;
	while(ros::ok() && is_running_)
	{
		//加锁，状态数据拷贝
		state_mutex_.lock();
		vehicle_speed = vehicle_speed_;
		roadwheel_angle = roadwheel_angle_;
		now_point = vehicle_pose_;
		state_mutex_.unlock();
		
		nearest_point_index_mutex_.lock();
		//横向偏差,左偏为负,右偏为正
		lateral_err_ = calculateDis2path(now_point.x, now_point.y,path_points_,nearest_point_index_,&nearest_point_index_);
		nearest_point_index_mutex_.unlock();
		//航向偏差,左偏为正,右偏为负
		yaw_err_ = path_points_[nearest_point_index_].yaw - now_point.yaw;
		if(yaw_err_ > M_PI)
			yaw_err_ -= 2*M_PI;
		else if(yaw_err_ < -M_PI)
			yaw_err_ += 2*M_PI;
		
		disThreshold_ = foreSightDis_speedCoefficient_ * vehicle_speed + foreSightDis_latErrCoefficient_ * fabs(lateral_err_);
	
		if(disThreshold_ < min_foresight_distance_) 
			disThreshold_  = min_foresight_distance_;
		size_t target_index = nearest_point_index_+1;//跟踪目标点索引
		
		gpsMsg_t target_point = path_points_[target_index];
		//获取当前点到目标点的距离和航向
		std::pair<float, float> dis_yaw = getDisAndYaw(target_point, now_point);
	#if 1
		//循环查找满足disThreshold_的目标点
		while(dis_yaw.first < disThreshold_)
		{
			target_point = path_points_[++target_index];
			dis_yaw = getDisAndYaw(target_point, now_point);
			//ROS_INFO("path_points_:%d\t target_index:%d",path_points_.size(),target_index);
		}
		float theta = dis_yaw.second - now_point.yaw;
		if(theta > M_PI) theta -= 2*M_PI;
		else if(theta < -M_PI) theta += 2*M_PI;
	#else
		while(dis_yaw.first < disThreshold_)
		{
			target_point = path_points_[++target_index];
			dis_yaw = getDisAndYaw(target_point, now_point);
		}
		
		ROS_INFO("target_index:%d  dis:%.2f",target_index,dis_yaw.first);
		
		//预瞄航向与车辆航向夹角
		//预瞄点在右侧时,theta为正,反之为负
		float theta;
		size_t temp_target_index = target_index;
		for( ; ; )
		{
			theta = dis_yaw.second - now_point.yaw;
			if(theta > M_PI) theta -= 2*M_PI;
			else if(theta < -M_PI) theta += 2*M_PI;
			
			//车辆沿圆弧到达预瞄点时的航向与预瞄点航向的偏差
			float target_yaw_err = now_point.yaw+2*theta-target_point.yaw;
			if(target_yaw_err > M_PI)
				target_yaw_err -= 2*M_PI;
			else if(target_yaw_err < -M_PI)
				target_yaw_err += 2*M_PI;
			
//			ROS_INFO("theta:%.2f\ttarget_yaw_err:%.2f",theta/M_PI*180.0,fabs(target_yaw_err)/M_PI*180.0);
			if(fabs(target_yaw_err) < max_target_yaw_err_ ||
				(yaw_err_*theta > 0) && (fabs(yaw_err_) > 15.0/180.0*M_PI)) //航向角缩小
				break;
		
			target_point = path_points_[++temp_target_index];
			dis_yaw = getDisAndYaw(target_point, now_point);
//			ROS_INFO("target_yaw_err:%.2f",target_yaw_err*180.0/M_PI);
		}
		ROS_INFO("raw_target_index:%d  temp_target_index:%d",target_index,temp_target_index);
	#endif
	
		if(theta==0.0 || theta==M_PI) continue;
		
		float turning_radius = (-0.5 * dis_yaw.first)/sin(theta);

		float t_roadWheelAngle = generateRoadwheelAngleByRadius(turning_radius);
		
		t_roadWheelAngle = limitRoadwheelAngleBySpeed(t_roadWheelAngle,vehicle_speed);
		
		//float curvature_search_distance = disThreshold_ + 13; //曲率搜索距离
		float curvature_search_distance = vehicle_speed_ * vehicle_speed_/(2 * 1);
		float max_curvature = maxCurvatureInRange(path_points_,nearest_point_index_,curvature_search_distance);

		float max_speed = generateMaxTolarateSpeedByCurvature(max_curvature, max_side_accel_);
		max_speed = limitSpeedByParkingPoint(max_speed);
		cmd_mutex_.lock();
		cmd_.validity = true;
		cmd_.speed = (expect_speed_>max_speed) ? max_speed : expect_speed_;
		cmd_.roadWheelAngle = t_roadWheelAngle;
		cmd_mutex_.unlock();
		//ROS_INFO("speed:%.2f\t max:%.2f\t e:%.2f",cmd_.speed,max_speed,expect_speed_);
		
		publishPathTrackingState();
		publishNearestIndex();

		if((i++)%50==0)
		{
			ROS_INFO("min_r:%.3f\t max_speed:%.1f",1.0/max_curvature, max_speed);
			ROS_INFO("set_speed:%f\t speed:%f",cmd_.speed ,vehicle_speed_*3.6);
			ROS_INFO("dis2target:%.2f\t yaw_err:%.2f\t lat_err:%.2f",dis_yaw.first,yaw_err_*180.0/M_PI,lateral_err_);
			ROS_INFO("disThreshold:%f\t expect roadwheel angle:%.2f",disThreshold_,t_roadWheelAngle);
			ROS_INFO("nearest_point_index:%d",nearest_point_index_);
			publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::OK,"Running");
		}
		
		loop_rate.sleep();
	}
	
	publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::OK,"Arrived at the destination.");
	ROS_INFO("driverless completed...");
	
	cmd_mutex_.lock();
	cmd_.speed = 0.0;
	cmd_.roadWheelAngle = 0.0;
	cmd_mutex_.unlock();
	
	is_running_ = false;
}

size_t PathTracking::getNearestPointIndex()
{
	std::lock_guard<std::mutex> lock(nearest_point_index_mutex_);
	return nearest_point_index_;
}


void PathTracking::publishPathTrackingState()
{
	if(pub_tracking_state_.getNumSubscribers())
	{
		tracking_state_.header.stamp = ros::Time::now();
		tracking_state_.position_x = vehicle_pose_.x;
		tracking_state_.position_y = vehicle_pose_.y;
		tracking_state_.yaw = vehicle_pose_.yaw;
		tracking_state_.vehicle_speed =  vehicle_speed_;
		tracking_state_.roadwheel_angle = roadwheel_angle_;
		tracking_state_.lateral_error = lateral_err_;
		tracking_state_.yaw_error = yaw_err_;
	
		pub_tracking_state_.publish(tracking_state_);
	}
}

void PathTracking::publishNearestIndex()
{
	static std_msgs::UInt32 msg;
	if(pub_nearest_index_.getNumSubscribers())
	{
		msg.data = nearest_point_index_;
		pub_nearest_index_.publish(msg);
	}
}

gpsMsg_t PathTracking::pointOffset(const gpsMsg_t& point,float offset)
{
	gpsMsg_t result = point;
	result.x =  offset * cos(point.yaw) + point.x;
	result.y = -offset * sin(point.yaw) + point.y;
	return result;
}

/*@brief 当前位置到停车点的距离
*/
float PathTracking::disToParkingPoint(const parkingPoint_t& parkingPoint)
{
	if(nearest_point_index_ >=  parkingPoint.index) //当前位置已经超过停车点
		return 0;
	
	float dis1 = path_points_resolution_ * (parkingPoint.index-nearest_point_index_); //估计路程
	float dis2 = disBetweenPoints(path_points_[parkingPoint.index],path_points_[nearest_point_index_]);//直线距离
	float dis2end = dis1 > dis2 ? dis1 : dis2;
	return dis2end;
	
}

/*@brief 重载基类方法，载入停车点
*/
bool PathTracking::setParkingPoints(const std::vector<parkingPoint_t>& points)
{	
	//先调用基类函数载入停车点，然后在进行处理
	bool ok = AutoDriveBase::setParkingPoints(points);
	if(!ok) return ok;

	next_parking_index_ = points.size(); //初始化为越界索引
	for(size_t i=0; i<parking_points_.size(); ++i)
	{
		if(parking_points_[i].index > nearest_point_index_)
		{
			next_parking_index_ = i;
			break;
		}
	}
	return true;
}

/*@brief 根据停车点限制车辆速度
*/
float PathTracking::limitSpeedByParkingPoint(const float& speed,const float& acc)
{
	//初始化时parking_points_至少有一个点(终点)
	//每到达一个停车点并完成停车后,更新下一个停车点
	//程序初始化时,应将停车点由近及远排序,并将位于当前位置之后的停车点复位
	if(parking_points_.size() == 0)
	{
		ROS_ERROR("[%s] No Parking Points!",__NAME__);
		return 0.0;
	}

	//无可用停车点,已经到达终点
	//此处必须检查是否越界，防止出错
	if(next_parking_index_ >= parking_points_.size())
		return 0.0;

	parkingPoint_t& parking_point = parking_points_[next_parking_index_];
	
	if(parking_point.isParking) //正在停车中
	{
		//停车周期为0,到达终点
		if(parking_point.parkingDuration == 0.0)
			return 0.0;
		//停车超时,移除当前停车点,下次利用下一停车点限速
		if(ros::Time::now().toSec()-parking_point.parkingTime >= parking_point.parkingDuration)
		{
			ROS_INFO("parking overtime. parking point :%d",parking_point.index);
			next_parking_index_ ++;
			return speed;
		}
		//正在停车,时间未到
		return 0.0;
	}
	
	float dis2ParkingPoint = disToParkingPoint(parking_point);
	float maxSpeed = sqrt(2*acc*dis2ParkingPoint);
	//ROS_INFO("dis2end:%.2f\tmaxSpeed:%.2f",dis2end,maxSpeed);
	if(dis2ParkingPoint < 0.5)//到达停车点附近,速度置0,,防止抖动
	{
		parking_point.parkingTime = ros::Time::now().toSec();
		parking_point.isParking = true;
		ROS_INFO("[%s] start parking. point:%d",__NAME__, parking_point.index);
		return 0.0;
	}
	return speed > maxSpeed ? maxSpeed : speed;
}

/*@brief 利用转弯半径计算前轮转角
 *@param radius 转弯半径
 *@return 前轮转角
 */
inline float PathTracking::generateRoadwheelAngleByRadius(const float& radius)
{
	assert(radius!=0);
	//return asin(vehicle_.wheelbase /radius)*180/M_PI;  //the angle larger
	return atan(vehicle_.wheel_base/radius)*180/M_PI;    //correct algorithm 
}

/*@brief 获取两点间的距离以及航向
 *@param point1 终点
 *@param point2 起点
 */
std::pair<float, float> PathTracking::getDisAndYaw(const gpsMsg_t &point1, const gpsMsg_t &point2)
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

/*@brief 根据当前车速限制车辆前轮转角
 *@brief 算法已经根据路径曲率对车速进行了限制，
 *@brief 但可能出现转弯半径小于路径转弯半径,而导致侧向加速度超限的情况
 *@param angle 期望转角
 *@param speed 车速
 *@return 限制后的转角
 */
float PathTracking::limitRoadwheelAngleBySpeed(const float& angle, const float& speed)
{
	float min_steering_radius = speed*speed/max_side_accel_;
	
	float max_angle = fabs(generateRoadwheelAngleByRadius(min_steering_radius));
	if(max_angle > vehicle_.max_roadwheel_angle)
	   max_angle = vehicle_.max_roadwheel_angle;
	//ROS_INFO("max_angle:%f\t angle:%f",max_angle,angle);
	return saturationEqual(angle,max_angle);
}

/*@brief 利用路径曲率限制最大车速
 *@brief 保证车辆侧向加速度在一定范围
 *@param curvature 路径曲率
 *@param max_accel 最大允许侧向加速度
 *
 *@return 最大车速 km/h
 */
float PathTracking::generateMaxTolarateSpeedByCurvature(const float& curvature, const float& max_accel)
{
	return sqrt(1.0/fabs(curvature)*max_accel) *3.6;
}

/*@brief 利用路径曲率限制最大车速
 *@brief 保证车辆侧向加速度在一定范围
 *@param path_points        路径点
 *@param start_search_index 搜索起点
 *@param end_search_index   搜索终点
 *
 *@return 最大车速 km/h
 */
float PathTracking::generateMaxTolarateSpeedByCurvature(const std::vector<gpsMsg_t>& path_points,
											const size_t& start_search_index,
											const size_t& end_search_index,
											float max_side_accel)
{
	float max_cuvature = 0.0;
		
	for(size_t i=start_search_index; i < end_search_index; ++i)
	{
		if(fabs(path_points[i].curvature) > max_cuvature)
			max_cuvature = fabs(path_points[i].curvature);
	}
	return sqrt(1.0/max_cuvature*max_side_accel) *3.6;
}

/*@brief 利用当前前轮转角限制车速
 *@brief 保证车辆侧向加速度在一定范围
 *@param speed			期望车速
 *@param angle 			当前转角
 *
 *@return 最大车速 km/h
 */
float PathTracking::limitSpeedByCurrentRoadwheelAngle(float speed,float angle)
{
	float steering_radius = fabs(vehicle_.wheel_base/tan(angle*M_PI/180.0));
	float max_speed =  sqrt(steering_radius*max_side_accel_);
	
	return (speed>max_speed? max_speed: speed)*3.6;
}