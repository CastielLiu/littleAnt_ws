#include "driverless/reverse_drive.h"

#define __NAME__ "reverse_drive"
#define MAX_SPEED 5.0 //km/h

/* 倒车控制，使用actionlib实现倒车路径规划，然后进行倒车控制
 * 提供两种服务 1.指定目标点进行路径规划然后倒车控制
 *            2.指定倒车路径然后进行倒车路径跟踪     
*/
ReverseDrive::ReverseDrive():
    AutoDriveBase(__NAME__),
    as_(NULL)
{
    preview_dis_ = 2.0;
}
ReverseDrive::~ReverseDrive()
{
    this->stopCurrentWork();
    if(as_)
    {
        delete as_;
        as_ = NULL;
    }
}

bool ReverseDrive::init(ros::NodeHandle nh,ros::NodeHandle nh_private)
{
    nh_ = nh;
    nh_private_ = nh_private;

    nh_private_.param<float>("max_speed", max_speed_, 3.0);//km/h
    if(max_speed_ > MAX_SPEED)
    {
        ROS_ERROR("[%s] The max_speed is fast! Use the defaut value: %.2f.", __NAME__, MAX_SPEED);
        max_speed_ = MAX_SPEED;
    }
}

bool ReverseDrive::start()
{
    as_ = new ActionlibServer(nh_private_, "do_reverse", 
                              boost::bind(&ReverseDrive::executeCallback,this, _1), false);
    as_->start();
}

void ReverseDrive::stopCurrentWork()
{
    is_running_ = false;
    //直到工作线程退出，此处才会获得锁，然后退出
    std::lock_guard<std::mutex> lck(working_mutex_); 
}

void ReverseDrive::executeCallback(const driverless::DoReverseGoalConstPtr& goal)
{
    //此处需要如此判断？
    if(as_->isPreemptRequested())  //请求抢占
    {
        this->stopCurrentWork();
        if(!as_->isNewGoalAvailable()) //无新目标，待测试
            return;
    }

    if(goal->type == goal->POSE_TYPE) //给定倒车目标点位置
    {
        Pose target_pose;
        target_pose.x = goal->target_pose.x;
        target_pose.y = goal->target_pose.y;
        target_pose.yaw = goal->target_pose.theta;
        if(!reversePathPlan(target_pose))
        {
            driverless::DoReverseResult res;
            res.success = false;

            as_->setAborted(res, "Aborting on reverse goal, because it is invalid ");
            return;
        }
    }
    else if(goal->type == goal->PATH_TYPE) //给定倒车路径
    {
        size_t len = goal->target_path.size();
        reverse_path_.points.reserve(len);

        for(const geometry_msgs::Pose2D& pose : goal->target_path)
        {
            GpsPoint point;
            point.x = pose.x;
            point.y = pose.y;
            point.yaw = pose.theta;

            reverse_path_.points.push_back(point);
        }
    }

    if(!extendPath(reverse_path_,2*preview_dis_)) //路径拓展延伸
    {
        driverless::DoReverseResult res;
        res.success = false;
        as_->setAborted(res, "Aborting on reverse path, because it is invalid ");
        return;
    }

    std::thread t(&ReverseDrive::reverseControlThread, this);
    t.detach();
    

    //as_->setSucceeded();
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
    is_running_ = true;
    std::lock_guard<std::mutex> lck(working_mutex_);
 
    cmd_mutex_.lock();
    cmd_.validity = true;
    cmd_mutex_.unlock();

    reverse_path_.pose_index = findNearestPoint(reverse_path_, vehicle_state_.getPose(LOCK)); 
	size_t nearest_index = reverse_path_.pose_index;

	if(reverse_path_.finish()) //还未开始已经完成，表明路径无效
	{
		ROS_ERROR("Remaind target path is too short! nearest_point_index:%lu", nearest_index);
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::ERROR,"Remaind target path is too short!");
		is_running_ = false;

        driverless::DoReverseResult res;
        res.success = false;
        as_->setAborted(res, "Aborting on reverse path, because it is invalid ");
		return ;
	}

	size_t i =0;
	ros::Rate loop_rate(30);
	float yaw_err, lat_err;
    size_t target_index = nearest_index;
    GpsPoint target_point = reverse_path_[target_index];

	while(ros::ok() && is_running_ && !reverse_path_.finish())
	{
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
            cmd_.speed = max_speed_;
            cmd_.roadWheelAngle = 0.0;
            cmd_mutex_.unlock();
            continue;
        }
		
		float turning_radius = (0.5 * dis_yaw.first)/sin_theta;
		float t_roadWheelAngle = generateRoadwheelAngleByRadius(vehicle_params_.wheel_base, turning_radius);

		cmd_mutex_.lock();
		cmd_.speed = max_speed_;
		cmd_.roadWheelAngle = t_roadWheelAngle;
		cmd_mutex_.unlock();

        driverless::DoReverseFeedback feedback;
        feedback.speed = max_speed_;
        feedback.steer_angle = t_roadWheelAngle;
        as_->publishFeedback(feedback);

        loop_rate.sleep();
    }
}

