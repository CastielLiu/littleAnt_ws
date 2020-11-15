
#include "ros/ros.h"
#include "driverless/driverless_node.h"

#define __NAME__ "driverless"

/*
	step1. actionlib服务器回调函数(as_callback)收到新目标请求.
	step2. as_callback唤醒工作线程(workingThread)开始工作，然后as_callback挂起.
	step3. workingThread任务完成后继续挂起，唤醒as_callback判断是否有新目标.
	step4. 如果有新任务，返回step2, 否则as_callback退出并再次等待step1.
	
	as_callback使用work_cv_条件变量唤醒workingThread;
	workingThread使用listen_cv_条件变量唤醒as_callback.
 */

void AutoDrive::executeDriverlessCallback(const driverless::DoDriverlessTaskGoalConstPtr& goal)
{
	if(!handleNewGoal(goal)) return;
	
	std::unique_lock<std::mutex> lock(listen_cv_mutex_);
	listen_cv_.wait(lock, [&](){return request_listen_;});
	request_listen_ = false;
	listen_cv_mutex_.unlock();
	
	while(ros::ok())
	{
		if(as_->isNewGoalAvailable())
		{
			//if we're active and a new goal is available, we'll accept it, but we won't shut anything down
			ROS_INFO("[%s] The current work was interrupted by new request!", __NAME__);
			driverless::DoDriverlessTaskGoalConstPtr new_goal = as_->acceptNewGoal();

			if(!handleNewGoal(new_goal)) return;
			
			std::unique_lock<std::mutex> lock(listen_cv_mutex_);
			listen_cv_.wait(lock, [&](){return request_listen_;});
			request_listen_ = false;
		}
	}
}

/*@brief 处理新目标，
	①有效目标 对新目标进行预处理/载入相关文件/切换系统状态/唤醒工作线程，返回true
	②无效目标 返回false
 *@param goal 目标信息
*/
bool AutoDrive::handleNewGoal(const driverless::DoDriverlessTaskGoalConstPtr& goal)
{
	std::cout << "goal->type: " << int(goal->type)  << "\t"
			      << "goal->task: " << int(goal->task)  << "\t"
				  << "goal->file: " << goal->roadnet_file <<std::endl;
	switchSystemState(State_Stop); //新请求，无论如何先停止, 暂未解决新任务文件覆盖旧文件导致的自动驾驶异常问题，
                                   //因此只能停车后开始新任务
                                   //实则，若新任务与当前任务驾驶方向一致，只需合理的切换路径文件即可！
                                   //已经预留了切换接口，尚未解决运行中清空历史文件带来的隐患 

	ROS_INFO("[%s] new task received, vehicle has speed zero now.", __NAME__);
	this->expect_speed_ = goal->expect_speed;
	if(goal->task == goal->DRIVE_TASK)  //前进任务
    {
        //给定目标点位置，调用路径规划
        if(goal->type == goal->POSE_TYPE) 
        {
			ROS_ERROR("[%s] The forward path planning function has not been developed!", __NAME__);
			as_->setSucceeded(driverless::DoDriverlessTaskResult(), 
				"Aborting on drive task, because The forward path planning function has not been developed!");
			return false;
        }
        //指定驾驶路径点集
        else if(goal->type == goal->PATH_TYPE)
        {
			if(!setDriveTaskPathPoints(goal))
			{
				ROS_ERROR("[%s] The target path is invalid!", __NAME__);
                as_->setSucceeded(driverless::DoDriverlessTaskResult(), "Aborting on drive task, because The target path is invalid!");
				return false;
			}
        }
        //指定路径文件
        else if(goal->type == goal->FILE_TYPE)
        {
            if(!loadDriveTaskFile(goal->roadnet_file))
            {
                ROS_ERROR("[%s] Load drive path file failed!", __NAME__);
                driverless::DoDriverlessTaskResult res;
                res.success = false;
                as_->setSucceeded(res, "Aborting on drive task, because load drive path file failed! ");
                return false;
            }
        }
        else
        {
            ROS_ERROR("[%s] Request type error!", __NAME__);
			as_->setAborted(driverless::DoDriverlessTaskResult(), "Aborting on unknown goal type! ");
            return false;
        }
        //切换系统状态为: 切换到前进
        switchSystemState(State_SwitchToDrive);
        std::unique_lock<std::mutex> lck(work_cv_mutex_);
        has_new_task_ = true;
		work_cv_.notify_one(); //唤醒工作线程
		return true;
    }
    else if(goal->task == goal->REVERSE_TASK)  //倒车任务
    {
        //给定目标点位置，调用路径规划
        if(goal->type == goal->POSE_TYPE) 
        {
			//目标点位置
            Pose target_pose;
            target_pose.x = goal->target_pose.x;
            target_pose.y = goal->target_pose.y;
            target_pose.yaw = goal->target_pose.theta;
			//获取车辆当前点位置
			Pose vehicle_pose = vehicle_state_.getPose(LOCK);

            if(!reverse_controler_.reversePathPlan(vehicle_pose, target_pose))
            {
                driverless::DoDriverlessTaskResult res;
                res.success = false;
                as_->setAborted(res, "Aborting on reverse goal, because it is invalid ");
                return false;
            }
            ROS_INFO("[%s] plan reverse path complete.", __NAME__);
        }
        //指定驾驶路径点集
        else if(goal->type == goal->PATH_TYPE)
        {
			//等待开发
            size_t len = goal->target_path.size();
            Path reverse_path;
            reverse_path.points.reserve(len);
            for(const geometry_msgs::Pose2D& pose : goal->target_path)
            {
                GpsPoint point;
                point.x = pose.x;
                point.y = pose.y;
                point.yaw = pose.theta;

                reverse_path.points.push_back(point);
            }
            //reverse_controler_.setPath(reverse_path);
			//?
        }
        //指定路径文件
        else if(goal->type == goal->FILE_TYPE)
        {
            if(!reverse_controler_.loadReversePath(goal->roadnet_file, goal->path_filp))
            {
                ROS_ERROR("[%s] load reverse path failed!", __NAME__);
                driverless::DoDriverlessTaskResult res;
                res.success = false;
                as_->setAborted(res, "Aborting on reverse task, because load reverse path file failed! ");
                return false;
            }
			ROS_INFO("[%s] load reverse path ok!", __NAME__);
        }
        else
        {
            ROS_ERROR("[%s] Request type error!", __NAME__);
			as_->setAborted(driverless::DoDriverlessTaskResult(), "Aborting on unknown goal type! ");
            return false;
        }
        this->expect_speed_ = goal->expect_speed;
        
        //切换系统状态为: 切换到倒车
        switchSystemState(State_SwitchToReverse);
        std::unique_lock<std::mutex> lck(work_cv_mutex_);
        has_new_task_ = true;
		work_cv_.notify_one(); //唤醒工作线程
		return true;
    }
	else
	{
		ROS_ERROR("[%s] Unknown task type!", __NAME__);
		as_->setAborted(driverless::DoDriverlessTaskResult(), "Aborting on unknown task! ");
		return false;
	}
	ROS_ERROR("[%s] Unknown error type!", __NAME__);
	as_->setAborted(driverless::DoDriverlessTaskResult(), "Aborting on unknown error! ");
	return false;
}

void AutoDrive::workingThread()
{
	is_running_ = true;
	while(ros::ok() && is_running_)
	{
		/*使用条件变量挂起工作线程，等待其他线程请求唤醒，
		 *为防止虚假唤醒(系统等原因)，带有谓词参数的wait函数，唤醒的同时判断谓词是否满足，否则继续挂起
		 *条件变量与独占指针搭配使用，首先使用独占指针加锁，wait函数内部进行解锁并等待唤醒信号，线程唤醒后再次加锁
		 *当前线程被唤醒并开始工作且任务结束前，其他线程无法获得锁，当新任务到达
		 */
		std::unique_lock<std::mutex> lock(work_cv_mutex_);
		work_cv_.wait(lock, [&](){return has_new_task_;});
		has_new_task_ = false;

		int state = system_state_;
		ROS_INFO("[%s] Current system_state: %d", __NAME__, state);
		
		if(state == State_Drive)
			doDriveWork();
		else if(state == State_Reverse)
			doReverseWork();
		else
			ROS_ERROR("[%s] Unknown task type in current state: %d.", __NAME__, state);
			
		std::unique_lock<std::mutex> lck(listen_cv_mutex_);
		request_listen_ = true;
		listen_cv_.notify_one(); //唤醒监听线程
		
		//Can not call switchSystemState(State_Stop) here !!!
		//Because the expect state has set by doDriveWork/doReverseWork
		/*此处不可切换系统状态，而需要在上述子任务函数(doDriveWork/doReverseWork)中进行切换,
		 *子任务中系统状态切换包含两种情况，
		 *1. 任务完成，切换系统为停止状态
		 *2. 被新任务打断，切换系统为指定状态
		 *因此！此处不能再切换系统状态，否则状态机制将被打破，导致任务无法正常运行！
		 */
	}
}

void AutoDrive::doDriveWork()
{
	//配置路径跟踪控制器
	tracker_.setExpectSpeed(expect_speed_);
	tracker_.start();//路径跟踪控制器
	//配置跟车控制器
	car_follower_.start(); //跟车控制器

	ros::Rate loop_rate(20);
	
	while(ros::ok() && system_state_ == State_Drive && tracker_.isRunning())
	{
		tracker_cmd_ = tracker_.getControlCmd();
		follower_cmd_= car_follower_.getControlCmd();
		
		auto cmd = this->driveDecisionMaking();

		if(as_->isActive()) //判断action server是否为活动，防止函数的非服务调用导致的错误
		{
			driverless::DoDriverlessTaskFeedback feedback;
			feedback.speed = cmd.set_speed;
			feedback.steer_angle = cmd.set_roadWheelAngle;
			as_->publishFeedback(feedback);

			if(as_->isPreemptRequested()) 
			{
				ROS_INFO("[%s] isPreemptRequested.", __NAME__);
				as_->setPreempted(); //自主触发中断请求
				break;
			}
		}

		loop_rate.sleep();
	}
	ROS_INFO("[%s] drive work  completed...", __NAME__); 
	tracker_.stop();
	car_follower_.stop();
	if(as_->isActive())
	{
		as_->setSucceeded(driverless::DoDriverlessTaskResult(), "drive work  completed");
	}
	switchSystemState(State_Stop);
}

void AutoDrive::doReverseWork()
{
	reverse_controler_.setExpectSpeed(expect_speed_);
	reverse_controler_.start();
	
	ros::Rate loop_rate(20);
	while(ros::ok() && system_state_ == State_Reverse && reverse_controler_.isRunning())
	{
		//ROS_INFO("[%s] new cycle.", __NAME__);
		reverse_cmd_ = reverse_controler_.getControlCmd();
		
		//ROS_INFO("[%s] speed: %.2f\t angle: %.2f", __NAME__, reverse_cmd_.speed, reverse_cmd_.roadWheelAngle);
		
		if(reverse_cmd_.validity)
			reverseDecisionMaking();
		
		//如果actionlib服务器处于活跃状态，则进行状态反馈并判断是否外部请求中断
		//如果actionlib服务器未active表明是其他方式请求的工作，比如测试例
		if(as_->isActive())
		{
			driverless::DoDriverlessTaskFeedback feedback;
			feedback.speed = reverse_cmd_.speed;
			feedback.steer_angle = reverse_cmd_.roadWheelAngle;
			as_->publishFeedback(feedback);
			
			if(as_->isPreemptRequested())  //外部请求中断
			{
				ROS_INFO("[%s] isPreemptRequested.", __NAME__);
				as_->setPreempted(); //自主中断当前任务
				break;
			}
		}

		loop_rate.sleep();
	}
	reverse_controler_.stop();
	ROS_INFO("[%s] reverse work complete.", __NAME__);
	if(as_->isActive())
	{
		as_->setSucceeded(driverless::DoDriverlessTaskResult(), "drive work  completed");
	}
	switchSystemState(State_Stop);
}

ant_msgs::ControlCmd2 AutoDrive::driveDecisionMaking()
{
	std::lock_guard<std::mutex> lock2(cmd2_mutex_);

	if(system_state_ == State_ForceExternControl)
		return controlCmd2_;
	
//	follower_cmd_.display("follower_cmd");

	if(extern_cmd_.validity && extern_cmd_.speed < tracker_cmd_.speed)
		controlCmd2_.set_speed = extern_cmd_.speed;
	else if(follower_cmd_.validity && follower_cmd_.speed < tracker_cmd_.speed)
		controlCmd2_.set_speed = follower_cmd_.speed;
	else
		controlCmd2_.set_speed = tracker_cmd_.speed;
	
	controlCmd2_.set_roadWheelAngle = tracker_cmd_.roadWheelAngle;
	
	std::lock_guard<std::mutex> lock1(cmd1_mutex_);
	//转向灯
	if(extern_cmd_.turnLight == 1)
		controlCmd1_.set_turnLight_L = true;
	else if(extern_cmd_.turnLight == 2)
		controlCmd1_.set_turnLight_R = true;
	else if(extern_cmd_.turnLight == 0)
	{
		controlCmd1_.set_turnLight_R = false;
		controlCmd1_.set_turnLight_L = false;
	}
	return controlCmd2_;
}

ant_msgs::ControlCmd2 AutoDrive::reverseDecisionMaking()
{
	std::lock_guard<std::mutex> lock2(cmd2_mutex_);
	if(system_state_ == State_ForceExternControl)
		return controlCmd2_;

	controlCmd2_.set_speed = reverse_cmd_.speed;
	controlCmd2_.set_roadWheelAngle = reverse_cmd_.roadWheelAngle;

	return controlCmd2_;
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "auto_drive_node");
	ros::AsyncSpinner spinner(5);
	spinner.start(); //非阻塞

	ros::NodeHandle nh, nh_private("~");
    AutoDrive auto_drive;
    if(auto_drive.init(nh, nh_private))
    	ros::waitForShutdown();
    return 0;
}  


	
