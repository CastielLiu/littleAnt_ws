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
    if(as_->isPreemptRequested())
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

        //判断路径的有效性
        if(false)
        {
            driverless::DoReverseResult res;
            res.success = false;
            as_->setAborted(res, "Aborting on reverse path, because it is invalid ");
            return;
        }
    }

    std::thread t(&ReverseDrive::reverseControlThread, this);
    t.detach();
  
    as_->setSucceeded();
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
    ros::Rate loop_rate(20);

    cmd_mutex_.lock();
    cmd_.validity = true;
    cmd_mutex_.unlock();

    while(ros::ok() && is_running_)
    { 

        cmd_mutex_.lock();
        cmd_.speed = max_speed_;
        cmd_.roadWheelAngle = 0.0;
        cmd_mutex_.unlock();
    
        driverless::DoReverseFeedback feedback;
        feedback.speed = 3.0;
        feedback.steer_angle = 0.0;
        as_->publishFeedback(feedback);

        loop_rate.sleep();
    }

}

