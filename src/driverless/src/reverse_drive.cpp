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

    pub_local_path_ = nh_private_.advertise<nav_msgs::Path>("/local_path",2);

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
 *@param origin_pose 车辆姿态
 *@param target_pose 目标点姿态
*/
bool ReverseDrive::reversePathPlan(const Pose& origin_pose, const Pose& target_pose)
{
    float park_rect_width = 2.7;
    float park_rect_length = 5.0;
    Pose p2; //车位入口点
    p2.x = target_pose.x + park_rect_length/2 * sin(target_pose.yaw);
    p2.y = target_pose.y + park_rect_length/2 * cos(target_pose.yaw);
    p2.yaw = target_pose.yaw;

    Point p1; //车位前方某点位置
    p1.x = target_pose.x + (park_rect_length/2+2.0) * sin(target_pose.yaw);
    p1.y = target_pose.y + (park_rect_length/2+2.0) * cos(target_pose.yaw);
    Pose p0 = origin_pose; //车辆位置

    Point p0_in_p2 = global2local(p2, p0); //将p0转换到p2坐标系
    float p2_dis2_line = p0_in_p2.x; //车辆到车库横线的距离(+/-)
    //最小转弯半径情况下，弧线到车库横线的最小距离
    float min_dis = vehicle_params_.min_radius * (1-fabs(sin(p0.yaw-p2.yaw))); 
    if(p2_dis2_line - min_dis < vehicle_params_.width)
    {
        ROS_ERROR("[%s] Can not park vehicle, Because too close!", __NAME__);
        return false;
    }

    float dx = p0.x - p2.x, dy = p0.y - p2.y;
    float distance = sqrt(dx*dx + dy*dy);
    int point_cnt = distance/0.1;

    reverse_path_.points.reserve(point_cnt+1);
    for(size_t i=0; i<point_cnt+1; ++i)
    {
        float t = 1.0*i/point_cnt;
        GpsPoint point;
        point.x = pow(1 - t, 2)*p0.x + 2*t*(1 - t)*p1.x + pow(t, 2) * p2.x;
		point.y = pow(1 - t, 2)*p0.y + 2*t*(1 - t)*p1.y + pow(t, 2) * p2.y;
        reverse_path_.points.push_back(point);
    }
    reverse_path_.final_index = reverse_path_.points.size();
    return extendPath(reverse_path_, 2*preview_dis_);
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
    int cnt = 0;
	while(ros::ok() && is_running_ && !reverse_path_.finish())
	{
		//ROS_INFO("[%s] new cycle.", __NAME__);
		//创建基类数据拷贝
		VehicleState vhicle = vehicle_state_;
		const Pose&  pose   = vhicle.pose;
        float back_yaw = pose.yaw + M_PI;
		
		//横向偏差,左偏为负,右偏为正
		lat_err = calculateDis2path(pose.x, pose.y, reverse_path_, nearest_index, &nearest_index);
		reverse_path_.pose_index = nearest_index;
		
		//航向偏差,左偏为正,右偏为负
		yaw_err = reverse_path_[nearest_index].yaw - back_yaw;
		if(yaw_err > M_PI)       yaw_err -= 2*M_PI;
		else if(yaw_err < -M_PI) yaw_err += 2*M_PI;
		
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

        if((cnt ++)%10 ==0)
        {
            this->publishLocalPath();
            ROS_INFO("[%s] lateral error: %.2f.", __NAME__, lat_err);
            ROS_INFO("[%s] yaw error: %.2f.", __NAME__, yaw_err*180.0/M_PI);
            ROS_INFO("[%s] expect speed: %.2f\t expect angle: %.2f", __NAME__, exp_speed_, t_roadWheelAngle);
        }
		
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

void ReverseDrive::publishLocalPath()
{
    nav_msgs::Path path;
    path.header.stamp=ros::Time::now();
    path.header.frame_id="base_link";
	size_t startIndex = reverse_path_.pose_index;
	size_t endIndex   = reverse_path_.final_index;
	if(endIndex <= startIndex)
		return;

	Pose origin_point = vehicle_state_.getPose(LOCK);

	path.poses.reserve(endIndex-startIndex+1);
	
	for(size_t i=startIndex; i<endIndex; ++i)
	{
		const auto& global_point = reverse_path_[i];
		std::pair<float,float> local_point = 
			global2local(origin_point.x,origin_point.y,-origin_point.yaw, global_point.x,global_point.y);
		
		geometry_msgs::PoseStamped poseStamped;
        poseStamped.pose.position.x = local_point.first;
        poseStamped.pose.position.y = local_point.second;

        poseStamped.header.frame_id="base_link";
        path.poses.push_back(poseStamped);
	}
	pub_local_path_.publish(path);
}