#include "driverless/reverse_drive.h"

#define __NAME__ "reverse_drive"
#define MAX_SPEED 5.0 //km/h

/* 倒车控制器
*/
ReverseDrive::ReverseDrive():
    AutoDriveBase(__NAME__)
{
    preview_dis_ = 2.0;
}
ReverseDrive::~ReverseDrive()
{
    this->stop();
}

/*@brief 初始化倒车控制器*/
bool ReverseDrive::init(ros::NodeHandle nh,ros::NodeHandle nh_private)
{
    if(is_initialed_) return true;
    nh_ = nh;
    nh_private_ = nh_private;

    nh_private_.param<float>("max_speed", max_speed_, 3.0);//km/h
    if(max_speed_ > MAX_SPEED)
    {
        ROS_ERROR("[%s] The max_speed is fast! Use the defaut value: %.2f.", __NAME__, MAX_SPEED);
        max_speed_ = MAX_SPEED;
    }
    exp_speed_ = max_speed_;
    //启动actionlib 服务器，监听外部请求并执行相应操作
    is_initialed_ = true;
    return true;
}

bool ReverseDrive::start()
{
    //如果有正在执行的任务，则先终止
    if(is_running_)
    	stop();

    is_running_ = true;
    std::thread t(&ReverseDrive::reverseControlThread, this);
    t.detach();
    return true;
}

/*@brief 倒车路径规划
 *@param target_pose 目标点姿态
*/
bool ReverseDrive::reversePathPlan(const Pose& target_pose)
{
    //reverse_path_.points.push_back(point)
    return true;
}

void ReverseDrive::reverseControlThread()
{
    std::lock_guard<std::mutex> lck(working_mutex_);
 
    cmd_mutex_.lock();
    cmd_.validity = true;
    cmd_mutex_.unlock();

    reverse_path_.pose_index = findNearestPoint(reverse_path_, vehicle_state_.getPose(LOCK)); 
	size_t nearest_index = reverse_path_.pose_index;
	
	ROS_INFO("[%s] Ready to reverse tracking, the nearest point index is: %lu", __NAME__, nearest_index);

	if(reverse_path_.finish()) //还未开始已经完成，表明路径无效
	{
		ROS_ERROR("[%s] Remaind target path is too short! nearest_point_index:%lu", __NAME__, nearest_index);
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::ERROR,"Remaind target path is too short!");
		is_running_ = false;
	}
	
	ROS_INFO("[%s] Current path is validity, ready to reverse tracking.", __NAME__);

	size_t i =0;
	ros::Rate loop_rate(30);
	float yaw_err, lat_err;
    size_t target_index = nearest_index;
    GpsPoint target_point = reverse_path_[target_index];

	while(ros::ok() && is_running_ && !reverse_path_.finish())
	{
		ROS_INFO("[%s] new cycle.", __NAME__);
		//创建基类数据拷贝
		VehicleState vhicle = vehicle_state_;
		const Pose&  pose   = vhicle.pose;
        float back_yaw = pose.yaw + M_PI;
		
		//横向偏差,左偏为负,右偏为正
		lat_err = calculateDis2path(pose.x, pose.y, reverse_path_, nearest_index, &nearest_index);
		reverse_path_.pose_index = nearest_index;
		
		ROS_INFO("[%s] lateral error: %.2f.", __NAME__, lat_err);
		
		//航向偏差,左偏为正,右偏为负
		yaw_err = reverse_path_[nearest_index].yaw - back_yaw;
		if(yaw_err > M_PI)       yaw_err -= 2*M_PI;
		else if(yaw_err < -M_PI) yaw_err += 2*M_PI;
		
		ROS_INFO("[%s] yaw error: %.2f.", __NAME__, yaw_err*180.0/M_PI);
		
        std::pair<float, float> dis_yaw;      //当前点到目标点的距离和航向
        while(ros::ok())
        {
            dis_yaw = getDisAndYaw(target_point, pose);
            if(dis_yaw.first >= preview_dis_)
                break;
            target_point = reverse_path_[++target_index];
        }

		float theta = dis_yaw.second - back_yaw;
		float sin_theta = sin(theta);
        if(sin_theta == 0)
        {
            cmd_mutex_.lock();
            cmd_.speed = exp_speed_;
            cmd_.roadWheelAngle = 0.0;
            cmd_mutex_.unlock();
            continue;
        }
		
		float turning_radius = (0.5 * dis_yaw.first)/sin_theta;
		float t_roadWheelAngle = generateRoadwheelAngleByRadius(vehicle_params_.wheel_base, turning_radius);
		if(t_roadWheelAngle > vehicle_params_.max_roadwheel_angle)
			t_roadWheelAngle = vehicle_params_.max_roadwheel_angle;
		else if(t_roadWheelAngle < vehicle_params_.min_roadwheel_angle)
			t_roadWheelAngle = vehicle_params_.min_roadwheel_angle;
		
		float steer_angle_err = fabs(vehicle_state_.getSteerAngle(LOCK) - t_roadWheelAngle);
		
		//limit vehicle speed by the remaind distance(dis to end)
		float t_speed = reverse_path_.remaindDis() * 1.0 + 0.5;
		if(t_speed > exp_speed_) t_speed = exp_speed_;
		
		if(steer_angle_err > 4.0)
			t_speed = 0.0;

		cmd_mutex_.lock();
		cmd_.speed = t_speed;
		cmd_.roadWheelAngle = t_roadWheelAngle;
		cmd_mutex_.unlock();
		
		ROS_INFO("[%s] expect speed: %.2f\t expect angle: %.2f", __NAME__, exp_speed_, t_roadWheelAngle);
		
        loop_rate.sleep();
    }
    cmd_mutex_.lock();
	cmd_.speed = 0.0;
	cmd_.roadWheelAngle = 0.0;
	cmd_mutex_.unlock();
	is_running_ = false;
    reverse_path_.clear(); //清空路径信息
}

/*@brief 从文件载入倒车路径
 *@param file 文件名
 *@param reverse 是否需要翻转路径
*/
bool ReverseDrive::loadReversePath(const std::string& file, bool reverse)
{
    if(!loadPathPoints(file, reverse_path_))
        return false;
    if(reverse)
    {
        Path temp_path;
        temp_path.points.reserve(reverse_path_.points.size());

        for(int i=reverse_path_.points.size()-1; i >0; --i)
        {
            GpsPoint& point = reverse_path_.points[i];
            //point.yaw += M_PI;
            //if(point.yaw > 2*M_PI) point.yaw -= 2*M_PI;
            temp_path.points.push_back(point);
        }
        reverse_path_.points.swap(temp_path.points);
    }

    return extendPath(reverse_path_, 2*preview_dis_);
}

void ReverseDrive::stop()
{
    is_running_ = false;
    //直到工作线程退出，此处才会获得锁，然后退出
    std::lock_guard<std::mutex> lck(working_mutex_); 
}