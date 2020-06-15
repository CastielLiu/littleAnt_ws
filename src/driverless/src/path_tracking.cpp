#include "driverless/path_tracking.h"
#define __NODE__ "path_tracking"

PathTracking::PathTracking():
	target_point_index_(0),
	nearest_point_index_(0),
	expect_speed_(5.0), //defult expect speed
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
	return true;
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
	nh_private.param<float>("foreSightDis_speedCoefficient", foreSightDis_speedCoefficient_,0.8);
	nh_private.param<float>("foreSightDis_latErrCoefficient", foreSightDis_latErrCoefficient_,-3.0);
	nh_private.param<float>("min_foresight_distance",min_foresight_distance_,4.0);
	nh_private.param<float>("max_side_accel",max_side_accel_,1.0);
	nh_private.param<int>  ("dst_index",dst_index_,0);

	pub_diagnostic_ = nh.advertise<diagnostic_msgs::DiagnosticStatus>("driverless/diagnostic",1);
	pub_tracking_state_ = nh.advertise<driverless::TrackingState>(tracking_info_topic,1);

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
	if(!extendGlobalPath(10))
		return false;
	is_running_ = true;
	std::thread t(&PathTracking::trackingThread,this);
	t.detach();
	return true;
}

void PathTracking::stop()
{
	is_running_ = false;
}

/*@brief 拓展全局路径，防止车辆临近终点时无法预瞄
 *@param extendDis 拓展长度，保证实际终点距离虚拟终点大于等于extendDis
 *
 */
bool PathTracking::extendGlobalPath(float extendDis)
{
	if(dst_index_ == 0) //终点索引为默认0
		dst_index_ = path_points_.size()-1;
	float remaindDis = 0.0; //期望终点与路径终点的路程
	for(size_t i=dst_index_; i<path_points_.size()-1; ++i)
		remaindDis += dis2Points(path_points_[i],path_points_[i+1]);
	if(remaindDis > extendDis)
		return true;
	
	int n = 5;
	size_t endIndex = path_points_.size()-1;
	if(endIndex < n)
		return false;
	
	//取最后一个点与倒数第n个点的连线向后插值
	float dx = (path_points_[endIndex].x - path_points_[endIndex-n].x)/n;
	float dy = (path_points_[endIndex].y - path_points_[endIndex-n].y)/n;
	float ds = sqrt(dx*dx+dy*dy);

	gpsMsg_t point;
	for(size_t i=1;;++i)
	{
		point.x = path_points_[endIndex].x + dx*i;
		point.y = path_points_[endIndex].y + dy*i;
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
	{
		usleep(500000);
	}

	state_mutex_.lock();
	nearest_point_index_ = findNearestPoint(path_points_,current_point_); 
	size_t target_index = nearest_point_index_;//跟踪目标点索引
	state_mutex_.unlock();

	if(target_index > path_points_.size() - 10)
	{
		ROS_ERROR("target index:%d\t, No target point was found !!!",target_index);
		publishDiagnostics(diagnostic_msgs::DiagnosticStatus::ERROR,"Remaind target path is too short!");
		is_running_ = false;
		return ;
	}
	
	gpsMsg_t target_point = path_points_[target_index];

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
		lateral_err_ = calculateDis2path(now_point.x, now_point.y,path_points_,nearest_point_index_,&nearest_point_index_);
		nearest_point_index_mutex_.unlock();
		
		disThreshold_ = foreSightDis_speedCoefficient_ * vehicle_speed + foreSightDis_latErrCoefficient_ * fabs(lateral_err_);
	
		if(disThreshold_ < min_foresight_distance_) 
			disThreshold_  = min_foresight_distance_;
									 
		std::pair<float, float> dis_yaw = get_dis_yaw(target_point, now_point);

		//循环查找满足disThreshold_的目标点
		while( dis_yaw.first < disThreshold_)
		{
			target_point = path_points_[++target_index];
			dis_yaw = get_dis_yaw(target_point, now_point);
		}
		
		yaw_err_ = dis_yaw.second - now_point.yaw;
		
		if(yaw_err_==0.0) continue;
		
		float turning_radius = (-0.5 * dis_yaw.first)/sin(yaw_err_);

		float t_roadWheelAngle = generateRoadwheelAngleByRadius(turning_radius);
		
		t_roadWheelAngle = limitRoadwheelAngleBySpeed(t_roadWheelAngle,vehicle_speed);
		
		float curvature_search_distance = disThreshold_ + 13; //曲率搜索距离 

		float max_curvature = maxCurvatureInRange(path_points_,nearest_point_index_,curvature_search_distance);

		float max_speed = generateMaxTolarateSpeedByCurvature(max_curvature, max_side_accel_);
		max_speed = limitSpeedByDestination(max_speed); 
		cmd_mutex_.lock();
		cmd_.validity = true;
		cmd_.speed = (expect_speed_>max_speed) ? max_speed : expect_speed_;
		cmd_.roadWheelAngle = t_roadWheelAngle;
		cmd_mutex_.unlock();
		//ROS_INFO("speed:%.2f\t max:%.2f\t e:%.2f",cmd_.speed,max_speed,expect_speed_);
		
		publishPathTrackingState();

		if((i++)%20==0)
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

gpsMsg_t PathTracking::pointOffset(const gpsMsg_t& point,float offset)
{
	gpsMsg_t result = point;
	result.x =  offset * cos(point.yaw) + point.x;
	result.y = -offset * sin(point.yaw) + point.y;
	return result;
}

float PathTracking::disToEnd()
{
	if(nearest_point_index_ >  dst_index_) //超出终点
		return 0;
	else if(nearest_point_index_ == dst_index_)
		return disBetweenPoints(path_points_[dst_index_],path_points_[nearest_point_index_]);
	else
	{
		float dis1 = path_points_resolution_ * fabs(dst_index_-nearest_point_index_); //估计路程
		float dis2 = disBetweenPoints(path_points_[dst_index_],path_points_[nearest_point_index_]);//直线距离
		float dis2end = dis1 > dis2 ? dis1 : dis2;
		return dis2end;
	}
}

float PathTracking::limitSpeedByDestination(const float& speed,const float& acc)
{
	float dis2end = disToEnd();
	float maxSpeed = sqrt(2*acc*dis2end);
	//ROS_INFO("dis2end:%.2f\tmaxSpeed:%.2f",dis2end,maxSpeed);
	if(dis2end < 0.5) //到达终点附近
		return 0;

	return speed > maxSpeed ? maxSpeed : speed;
}
