#include "driverless/path_tracking.h"
#define __NODE__ "path_tracking"

PathTracking::PathTracking():
	nearest_point_index_(0),
	expect_speed_(10.0), //defult expect speed
	is_ready_(false)
{
	diagnostic_msg_.hardware_id = "path_tracking";
	path_points_resolution_ = 0.1;
	cmd_.speed = cmd_.roadWheelAngle = 0.0;
}

PathTracking::~PathTracking()
{
}

void PathTracking::publishDiagnostics(uint8_t level,const std::string& msg)
{
	diagnostic_msg_.level = level;
	diagnostic_msg_.message = msg;
	pub_diagnostic_.publish(diagnostic_msg_);
}

bool PathTracking::setGlobalPath(const std::vector<gpsMsg_t>& path)
{
	if(path_points_.size()!=0)
		return false;
	path_points_ = path;
	dst_index_ = path_points_.size()-1;
	if(!extendGlobalPath(20.0))
		return false;
	return true;
}

void PathTracking::setDstIndex(size_t index)
{
	dst_index_ = index;
}

bool PathTracking::setExpectSpeed(float speed)
{
	expect_speed_ = speed;
}

bool PathTracking::updateStatus(const gpsMsg_t& pose,const float& speed, const float& roadWheelAngle)
{
	if(!is_ready_) is_ready_ = true;
	std::lock_guard<std::mutex> lock(state_mutex_);
	current_point_ = pose;
	vehicle_speed_ = speed;
	roadwheel_angle_ = roadWheelAngle;
}

bool PathTracking::init(ros::NodeHandle nh,ros::NodeHandle nh_private)
{
	std::string tracking_info_topic = 
	nh_private.param<std::string>("tracking_info_topic","/tracking_state");
	nh_private.param<std::string>("parking_points_file",parking_points_file_,"");
	nh_private.param<float>("foreSightDis_speedCoefficient", foreSightDis_speedCoefficient_,1.0);
	nh_private.param<float>("foreSightDis_latErrCoefficient", foreSightDis_latErrCoefficient_,-3.0);
	nh_private.param<float>("min_foresight_distance",min_foresight_distance_,5.0);
	nh_private.param<float>("max_side_accel",max_side_accel_,1.0);
	
	
	max_target_yaw_err_ = nh_private.param<float>  ("max_target_yaw_err",50.0)*M_PI/180.0;

	pub_diagnostic_ = nh.advertise<diagnostic_msgs::DiagnosticStatus>("driverless/diagnostic",1);
	pub_tracking_state_ = nh.advertise<driverless::TrackingState>(tracking_info_topic,1);
	pub_nearest_index_  = nh.advertise<std_msgs::UInt32>("/driverless/nearest_index",1);

	if(path_points_.size()==0)
	{
		ROS_ERROR("[%s] please set global path first",__NODE__);
		return false;
	}
		
	return true;
}

//启动跟踪线程
bool PathTracking::start()
{
	is_running_ = true;
	std::thread t(&PathTracking::trackingThread,this);
	t.detach();
	return true;
}

void PathTracking::stop()
{
	is_running_ = false;
}

/*@brief 拓展全局路径,防止车辆临近终点时无法预瞄
 *@param extendDis 拓展长度，保证实际终点距离虚拟终点大于等于extendDis
 *
 */
bool PathTracking::extendGlobalPath(float extendDis)
{
	float remaindDis = 0.0; //期望终点与路径终点的路程
	
	//计算剩余路程
	for(size_t i=dst_index_; i<path_points_.size()-1; ++i)
		remaindDis += dis2Points(path_points_[i],path_points_[i+1]);
	if(remaindDis > extendDis)
		return true;
	
	//取最后一个点与倒数第n个点的连线向后插值
	//总路径点不足n个,退出
	int n = 5;
	size_t endIndex = path_points_.size()-1;
	if(endIndex < n) return false;
	
	float dx = (path_points_[endIndex].x - path_points_[endIndex-n].x)/n;
	float dy = (path_points_[endIndex].y - path_points_[endIndex-n].y)/n;
	float ds = sqrt(dx*dx+dy*dy);

	gpsMsg_t point;
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

	nearest_point_index_ = findNearestPoint(path_points_,current_point_); 
	loadParkingPoints(nearest_point_index_);//载入停车点,并根据当前位置判断停车点的有效性

	if(nearest_point_index_ > path_points_.size() - 10)
	{
		ROS_ERROR("Remaind target path is too short! nearest_point_index:%d",nearest_point_index_);
		publishDiagnostics(diagnostic_msgs::DiagnosticStatus::ERROR,"Remaind target path is too short!");
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
		now_point = current_point_;
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
		std::pair<float, float> dis_yaw = get_dis_yaw(target_point, now_point);
	#if 1
		//循环查找满足disThreshold_的目标点
		while(dis_yaw.first < disThreshold_)
		{
			target_point = path_points_[++target_index];
			dis_yaw = get_dis_yaw(target_point, now_point);
			//ROS_INFO("path_points_:%d\t target_index:%d",path_points_.size(),target_index);
		}
		float theta = dis_yaw.second - now_point.yaw;
		if(theta > M_PI) theta -= 2*M_PI;
		else if(theta < -M_PI) theta += 2*M_PI;
	#else
		while(dis_yaw.first < disThreshold_)
		{
			target_point = path_points_[++target_index];
			dis_yaw = get_dis_yaw(target_point, now_point);
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
			dis_yaw = get_dis_yaw(target_point, now_point);
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
			publishDiagnostics(diagnostic_msgs::DiagnosticStatus::OK,"Running");
		}
		
		loop_rate.sleep();
	}
	
	publishDiagnostics(diagnostic_msgs::DiagnosticStatus::OK,"Arrived at the destination.");
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

controlCmd_t PathTracking::getControlCmd() 
{
	std::lock_guard<std::mutex> lock(cmd_mutex_);
	return cmd_;
}

bool PathTracking::isRunning()
{
	return is_running_;
}

void PathTracking::publishPathTrackingState()
{
	if(pub_tracking_state_.getNumSubscribers())
	{
		tracking_state_.header.stamp = ros::Time::now();
		tracking_state_.position_x = current_point_.x;
		tracking_state_.position_y = current_point_.y;
		tracking_state_.yaw = current_point_.yaw;
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

/*@brief 从文件载入停车点
*/
bool PathTracking::loadParkingPoints(size_t vehicle_pose_index)
{
	if(path_points_.size() == 0)
	{
		ROS_ERROR("please loadPathPoints first!");
		return false;
	}
	
	
	if(!parking_points_file_.empty())
	{
		FILE *fp = fopen(parking_points_file_.c_str(),"r");
		if(fp != NULL)
		{
			int index;
			float duration;
			while(!feof(fp))
			{
				fscanf(fp,"%d\t%f\n",&index,&duration);
				parking_points_.push_back(parkingPoint_t(index,duration));
			}
		}
		fclose(fp);
	}
//手动添加停车点
//	parking_points_.push_back(parkingPoint_t(3600,10));//中途停车
//	parking_points_.push_back(parkingPoint_t(1000,3));//中途停车
	parking_points_.push_back(parkingPoint_t(dst_index_,0));//终点索引,永久停留
	
	//移除车辆位置之后的点!
	for(size_t i=0; i<parking_points_.size(); ++i)
	{
		if(parking_points_[i].index <= vehicle_pose_index)
			parking_points_.erase(parking_points_.begin()+i);
	}
	
	//停车点排序
	std::sort(parking_points_.begin(),parking_points_.end(),
		[](const parkingPoint_t&point1,const parkingPoint_t&point2)
		{return point1.index < point2.index;});
		
	for(auto &point:parking_points_)
	{
		std::cout << "stop index:" << point.index << "\t" << point.parkingDuration << std::endl;
	}
	
	ROS_INFO("loadParkingPoints ok.");
	return true;
}

/*@brief 根据停车点限制车辆速度
*/
float PathTracking::limitSpeedByParkingPoint(const float& speed,const float& acc)
{
	//初始化时parking_points_至少有一个点(终点)
	//每到达一个停车点并完成停车后,移除该点
	//程序初始化时,应将停车点由近及远排序,并将位于当前位置之后的停车点移除
	if(parking_points_.size() == 0)
		return speed;
	
	if(parking_points_[0].isParking) //正在停车中
	{
		if(parking_points_[0].parkingDuration == 0.0)
			return 0.0;
		//停车超时,移除当前停车点,下次利用下一停车点限速
		if(ros::Time::now().toSec()-parking_points_[0].parkingTime >= parking_points_[0].parkingDuration)
		{
			ROS_INFO("parking overtime. parking point :%d",parking_points_[0].index);
			parking_points_.erase(parking_points_.begin());
			return speed;
		}
		return 0;
	}
	
	float dis2ParkingPoint = disToParkingPoint(parking_points_[0]);
	float maxSpeed = sqrt(2*acc*dis2ParkingPoint);
	//ROS_INFO("dis2end:%.2f\tmaxSpeed:%.2f",dis2end,maxSpeed);
	if(dis2ParkingPoint < 0.5)//到达停车点附近,速度置0,,防止抖动
	{
		parking_points_[0].parkingTime = ros::Time::now().toSec();
		parking_points_[0].isParking = true;
		ROS_INFO("start parking. point:%d",parking_points_[0].index);
		return 0;
	}
	return speed > maxSpeed ? maxSpeed : speed;
}
