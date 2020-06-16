#include "driverless/car_following.h"

#define __NAME__ "car_following"

CarFollowing::CarFollowing()
{
	targetId_ = 0xff; //no target
	cmd_update_time_ = 0.0;
	safety_side_dis_ = 0.0+1.5+0.5;
	cmd_.validity = false;
	is_running_ = false;
	is_ready_ = false;
}

bool CarFollowing::init(ros::NodeHandle nh,ros::NodeHandle nh_private)
{
	nh_ = nh;
	nh_private_ = nh_private;
	objects_topic_     = nh_private.param<std::string>("mm_radar_objects_topic","/esr_objects");
	radar_in_base_x_   = nh_private.param<float>("radar_in_base_x",  0.0);
	radar_in_base_y_   = nh_private.param<float>("radar_in_base_y",  2.0);
	radar_in_base_yaw_ = nh_private.param<float>("radar_in_base_yaw",0.0);//deg

	//nh_private.param<float>("max_following_speed",max_following_speed_,15.0);

	pub_diagnostic_ = nh.advertise<diagnostic_msgs::DiagnosticStatus>("driverless/diagnostic",1);
	return true;
}

bool CarFollowing::setGlobalPath(const std::vector<gpsMsg_t>& path)
{
	if(path_points_.size()!=0)
		return false;
	path_points_ = path;
	return true;
}

bool CarFollowing::isRunning()
{
	return is_running_;
}

bool CarFollowing::updateStatus(const gpsMsg_t& pose,const float& speed, const size_t& nearest_point_index)
{
	if(!is_ready_) is_ready_ = true;
	std::lock_guard<std::mutex> lock(state_mutex_);
	vehicle_pose_ = pose;
	vehicle_speed_ = speed;
	nearest_point_index_ = nearest_point_index;
}

//启动跟踪线程
bool CarFollowing::start()
{
	if(path_points_.size() == 0)
	{
		ROS_ERROR("[%s]: please setGlobalPath before start!", __NAME__);
		return false;
	}
		
	is_running_ = true;
	sub_objects_  = nh_.subscribe(objects_topic_,10,&CarFollowing::object_callback,this);
	update_timer_ = nh_.createTimer(ros::Duration(0.10),&CarFollowing::updateTimer_callback,this);
	return true;
}

void CarFollowing::stop()
{
	sub_objects_.shutdown();
	update_timer_.stop();
	is_running_ = false;
}

void CarFollowing::updateTimer_callback(const ros::TimerEvent&)
{
	if(ros::Time::now().toSec() - cmd_update_time_ > 0.2)
	{
		cmd_mutex_.lock();
		cmd_.validity = false;
		cmd_mutex_.unlock();
	}
}

//1. 目标分类，障碍物/非障碍物(假定目标尺寸，计算与全局路径的距离)
//2. 障碍物由近及远排序，跟踪最近的障碍
//3. 判断最近目标ID是否与上次ID一致！
void CarFollowing::object_callback(const esr_radar::ObjectArray::ConstPtr& objects)
{
	if(!is_ready_) return;
	if(objects->objects.size()==0) return;

	state_mutex_.lock();
	float vehicle_speed = vehicle_speed_;
	gpsMsg_t vehicle_pose = vehicle_pose_;
	size_t pose_index = nearest_point_index_;
	state_mutex_.unlock();

	//跟车距离，x = v*v/(2*a) + C常亮
	follow_distance_ = vehicle_speed*vehicle_speed/(2*2.0)  + 8.0;

	std::vector<esr_radar::Object> obstacles;
	float targetDis2path=1111;;
	for(size_t i=0; i< objects->objects.size(); ++i)
	{
		const esr_radar::Object& object = objects->objects[i];
		
		//目标雷达坐标转向GPS坐标
		std::pair<float, float> base_pose = 
			coordinationConvert(radar_in_base_x_,radar_in_base_y_,radar_in_base_yaw_, object.x,object.y);
		
		//目标局部坐标转换到大地全局坐标
		std::pair<float, float> object_global_pos =  
			coordinationConvert(vehicle_pose.x,vehicle_pose.y,vehicle_pose.yaw, base_pose.first,base_pose.second);
		
		//计算目标到全局路径的距离
		float dis2path = calculateDis2path(object_global_pos.first, object_global_pos.second,path_points_,pose_index);
		//std::cout << std::fixed << std::setprecision(2) <<
		//	object.x <<"  "<<object.y <<"\t" << dis2path << std::endl;
		if(fabs(dis2path) < safety_side_dis_)
		{
			targetDis2path = dis2path;
			obstacles.push_back(object);
		}
			
	}
	if(obstacles.size() == 0) return;

	//std::sort(obstacles.begin(),obstacles.end(),[](const auto& obj1, const auto& obj2){return obj1.distance < obj2.distance});
	float minDis = 200;
	size_t nearestObstalIndex = 0;
	for(size_t i=0; i<obstacles.size(); ++i)
	{
		if(obstacles[i].distance < minDis)
		{
			minDis = obstacles[i].distance;
			nearestObstalIndex = i;
		}
	}
	const esr_radar::Object& nearestObstal = obstacles[nearestObstalIndex];
	//distanceErr>0    acceleration
	//distanceErr<0    deceleration
	float distanceErr = nearestObstal.distance -follow_distance_;

	//线控底盘加速度和减速度未知，只能模糊调整，后期设定底盘加速度为定值或区间定值
	float t_speed;  //m/s
	//加速度与减速度不同，单独计算
	if(distanceErr >= 0)
		t_speed = vehicle_speed + nearestObstal.speed + distanceErr *0.5; 
	else
		t_speed = vehicle_speed + nearestObstal.speed + distanceErr *0.3;
			
	ROS_INFO("target dis:%.2f  speed:%.2f  dis2path:%.2f  vehicle speed:%.2f  t_speed:%.2f  t_dis:%.2f   dis:%f",minDis,vehicle_speed + nearestObstal.speed,targetDis2path, vehicle_speed,t_speed,follow_distance_,nearestObstal.distance);
	
	cmd_update_time_ = ros::Time::now().toSec();
	cmd_mutex_.lock();
	cmd_.validity = true;
	cmd_.speed = t_speed;
	cmd_mutex_.unlock();
}

controlCmd_t CarFollowing::getControlCmd() 
{
	std::lock_guard<std::mutex> lock(cmd_mutex_);
	return cmd_;
}
