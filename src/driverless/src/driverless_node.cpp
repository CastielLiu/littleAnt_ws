
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

void AutoDrive::executeDriverlessCallback(const driverless_common::DoDriverlessTaskGoalConstPtr& goal)
{
    driverless_common::DoDriverlessTaskResult res;
    std::string request_result;

    if(!handleNewGoal(goal,request_result))
    {
        res.success = false;
        as_->setAborted(res, request_result);
        return;
    }
    else
    {
		// 此处不应设置为成功, 只是任务开始执行，执行完毕后在workThread进行更新
        // res.success = true;
        // as_->setSucceeded(res, request_result);

		//发布反馈信息以通知客户端任务被接受并开始执行
		//由于使用的是SimpleActionServer,在收到目标时已默认接受(accept),因此客户端的active_callback仅能表达请求消息已到达服务器
		//并不能代表服务开始执行了请求
		driverless_common::DoDriverlessTaskFeedback feedback;
		as_->publishFeedback(feedback);
    }
	
	std::unique_lock<std::mutex> lock(listen_cv_mutex_);
	listen_cv_.wait(lock, [&](){return request_listen_;});
	request_listen_ = false;
	listen_cv_mutex_.unlock();
	
	while(ros::ok())
	{
		if(as_->isNewGoalAvailable())
		{
			//if we're active and a new goal is available, we'll accept it, but we won't shut anything down
			ROS_ERROR("[%s] NOT ERROR. The current work was interrupted by new request!", __NAME__);
			driverless_common::DoDriverlessTaskGoalConstPtr new_goal = as_->acceptNewGoal();

            if(!handleNewGoal(new_goal, request_result))
            {
                res.success = false;
                as_->setAborted(res, request_result);
                return;
            }
            else
            {
                driverless_common::DoDriverlessTaskFeedback feedback;
				as_->publishFeedback(feedback);
            }
			
			std::unique_lock<std::mutex> lock(listen_cv_mutex_);
			listen_cv_.wait(lock, [&](){return request_listen_;});
			request_listen_ = false;
		}
	}
}

/*@brief 处理新任务目标
	①有效目标 对新目标进行预处理/载入相关文件/切换系统状态/唤醒工作线程，返回true
	②无效目标  返回false
 *@param goal 目标信息
*/
bool AutoDrive::handleNewGoal(const driverless_common::DoDriverlessTaskGoalConstPtr& goal, std::string& result)
{
	// 离线调试
	if(is_offline_debug_)
	{
		switchSystemState(State_OfflineDebug); 
		std::unique_lock<std::mutex> lck(work_cv_mutex_);
        has_new_task_ = true;
		work_cv_.notify_one(); //唤醒工作线程
        result = "Task Started.";
		return true;
	}
	
	std::string info;
	if(!vehicle_state_.validity(info))
	{
		ROS_INFO("[%s] %s",__NAME__, info.c_str());
        publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::ERROR, info);

        result = "Driverless System Not Ready!";
		return false;
	}

	std::cout << "goal->type: " << int(goal->type)  << "\t"
			      << "goal->task: " << int(goal->task)  << "\t"
				  << "goal->file: " << goal->roadnet_file << "\t" 
				  << "expect_speed: " << goal->expect_speed << "\t"
				  << "path_resolution: " << goal->path_resolution << "\t"     
				  << std::endl;
	switchSystemState(State_Stop); //新请求，无论如何先停止, 暂未解决新任务文件覆盖旧文件导致的自动驾驶异常问题，
                                   //因此只能停车后开始新任务
                                   //实则，若新任务与当前任务驾驶方向一致，只需合理的切换路径文件即可！
                                   //已经预留了切换接口，尚未解决运行中清空历史文件带来的隐患

	ROS_ERROR("[%s] NOT ERROR. new task ready, vehicle has speed zero now.", __NAME__);
	this->expect_speed_ = goal->expect_speed;
	if(goal->task == goal->DRIVE_TASK)  //前进任务
    {
        //给定目标点位置，调用路径规划
        if(goal->type == goal->POSE_TYPE) 
        {
			ROS_ERROR("[%s] The forward path planning function has not been developed!", __NAME__);
            result = "Function Not Developed!";
			return false;
        }
        //指定驾驶路径点集
        else if(goal->type == goal->PATH_TYPE)
        {
			if(!setDriveTaskPathPoints(goal))
			{
				ROS_ERROR("[%s] The target path is invalid!", __NAME__);
                result = "Target Path Invalid!";

				return false;
			}
        }
        //指定路径文件
        else if(goal->type == goal->FILE_TYPE)
        {
            if(!loadDriveTaskFile(goal->roadnet_file))
            {
                ROS_ERROR("[%s] Load drive path file failed!", __NAME__);
                result = "Load Path File Failed!";
                return false;
            }
        }
        else
        {
            ROS_ERROR("[%s] Request type error!", __NAME__);
            result = "Unknow Goal Type!";
            return false;
        }
        //切换系统状态为: 切换到前进
        bool ok = switchSystemState(State_SwitchToDrive);
		if(!ok)
		{
            result = "Switch System State failed!";
			return false;
		}

        std::unique_lock<std::mutex> lck(work_cv_mutex_);
        has_new_task_ = true;
		work_cv_.notify_one(); //唤醒工作线程
        result = "Task Started.";
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
                result = "Plan Reverse Path Failed!";
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
            ROS_ERROR("Wait Develop");
            result = "Function Not Develop!";
            return false;
        }
        //指定路径文件
        else if(goal->type == goal->FILE_TYPE)
        {
            if(!reverse_controler_.loadReversePath(goal->roadnet_file, goal->path_filp))
            {
                ROS_ERROR("[%s] load reverse path failed!", __NAME__);
                result = "Load Path File Failed!";
                return false;
            }
			ROS_INFO("[%s] load reverse path ok!", __NAME__);
        }
        else
        {
            ROS_ERROR("[%s] Request type error!", __NAME__);
            result = "Goal Type Error.";
            return false;
        }
        this->expect_speed_ = goal->expect_speed;
        
        //切换系统状态为: 切换到倒车
        bool ok = switchSystemState(State_SwitchToReverse);
		if(!ok)
		{
            result = "Switch System State failed!";
			return false;
		}

        std::unique_lock<std::mutex> lck(work_cv_mutex_);
        has_new_task_ = true;
		work_cv_.notify_one(); //唤醒工作线程
        result = "Task Started.";
		return true;
    }
	else
	{
		ROS_ERROR("[%s] Unknown task type!", __NAME__);
        result = "Unknown task type!";
		return false;
	}
	result = "Source Code Error! Please Check!";
    ROS_ERROR("[%s] Code Error! Please Check!", __NAME__);
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
		else if(state == State_OfflineDebug)
			doOfflineDebugWork();
		else
			ROS_ERROR("[%s] Unknown task type in current state: %d.", __NAME__, state);
		
		if(goal_preempt_){
			as_->setPreempted(driverless_common::DoDriverlessTaskResult(), "canceled or preempt by other goal");
			ROS_INFO("canceled or preempt by other goal");
		}
		else{
			driverless_common::DoDriverlessTaskResult result;
			result.success = true;
			as_->setSucceeded(result, "task completed");
		}
		
		goal_preempt_ = false; //复位
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
	
	// ROS_ERROR("NOT ERROR: doDriveWork-> task_running_= true");
	task_running_ = true;

	while(ros::ok() && system_state_ != State_Stop && tracker_.isRunning() && !goal_preempt_ && as_->isActive())
	{
		tracker_cmd_ = tracker_.getControlCmd();
		follower_cmd_= car_follower_.getControlCmd();
		
		auto cmd = this->driveDecisionMaking();

		driverless_common::DoDriverlessTaskFeedback feedback;
		feedback.speed = cmd.set_speed;
		feedback.steer_angle = cmd.set_roadWheelAngle;
		as_->publishFeedback(feedback);
			
		//轮询判断是否有中断请求, 也可注册中断回调函数通知系统发生了中断请求
		// if(as_->isPreemptRequested())
		// {
		// 	ROS_INFO("[%s] isPreemptRequested.", __NAME__);
		// 	goal_preempt_ = true;
		// 	break;
		// }

		loop_rate.sleep();
	}
	ROS_INFO("[%s] drive work  completed...", __NAME__); 
	tracker_.stop();
	car_follower_.stop();
	
	task_running_ = false;
	switchSystemState(State_Stop);
}

void AutoDrive::doReverseWork()
{
	reverse_controler_.setExpectSpeed(expect_speed_);
	reverse_controler_.start();
	
	ros::Rate loop_rate(20);
	
	ROS_ERROR("NOT ERROR: doReverseWork-> task_running_= true");
	task_running_ = true;
	while(ros::ok() && system_state_ != State_Stop && reverse_controler_.isRunning() && !goal_preempt_ && as_->isActive())
	{
		//ROS_INFO("[%s] new cycle.", __NAME__);
		reverse_cmd_ = reverse_controler_.getControlCmd();
		
		//ROS_INFO("[%s] speed: %.2f\t angle: %.2f", __NAME__, reverse_cmd_.speed, reverse_cmd_.roadWheelAngle);
		
		if(reverse_cmd_.validity)
			reverseDecisionMaking();
		
		driverless_common::DoDriverlessTaskFeedback feedback;
		feedback.speed = reverse_cmd_.speed;
		feedback.steer_angle = reverse_cmd_.roadWheelAngle;
		as_->publishFeedback(feedback);

		loop_rate.sleep();
	}
	reverse_controler_.stop();
	ROS_INFO("[%s] reverse work complete.", __NAME__);
	task_running_ = false;
	switchSystemState(State_Stop);
}

void AutoDrive::doOfflineDebugWork()
{
	float percentage = 0.0;
	while(ros::ok() && system_state_ != State_Stop && !goal_preempt_ && as_->isActive())
	{
		driverless_common::DoDriverlessTaskFeedback feedback;
		percentage += 0.1;
		feedback.percentage = percentage;
		as_->publishFeedback(feedback);

		if(feedback.percentage >= 100)
			break;
			
		ros::Duration(0.5).sleep();
	}
	switchSystemState(State_Idle);
}

/*@brief 前进控制指令决策
 * 指令源包括: 避障控速/跟车控速/路径跟踪控转向和速度
 * 控制指令优先级 ①外部控制指令
                ②避障速度控制
				③跟车速度控制
 */
ant_msgs::ControlCmd2 AutoDrive::driveDecisionMaking()
{
	std::lock_guard<std::mutex> lock2(cmd2_mutex_);
	//若当前状态为强制使用外部控制指令，则忽悠其他指令源
	if(system_state_ == State_ForceExternControl)
		return controlCmd2_;

	controlCmd2_.set_roadWheelAngle = tracker_cmd_.roadWheelAngle;
	controlCmd2_.set_speed = tracker_cmd_.speed; //优先使用跟踪器速度指令
	controlCmd2_.set_brake = tracker_cmd_.brake;
	
	std::lock_guard<std::mutex> lock_extern_cmd(extern_cmd_mutex_);
	if(extern_cmd_.speed_validity){     //如果外部速度指令有效,则使用外部速度
		controlCmd2_.set_speed = extern_cmd_.speed;
		controlCmd2_.set_brake = extern_cmd_.brake;
	}
	if(avoid_cmd_.speed_validity){      //如果避障速度有效，选用最小速度
		controlCmd2_.set_speed = std::min(controlCmd2_.set_speed, avoid_cmd_.speed);
		controlCmd2_.set_brake = max(controlCmd2_.set_brake, avoid_cmd_.brake);
	}
	if(follower_cmd_.speed_validity){   //如果跟车速度有效，选用最小速度
		controlCmd2_.set_speed = std::min(controlCmd2_.set_speed, follower_cmd_.speed);
		controlCmd2_.set_brake = max(controlCmd2_.set_brake, follower_cmd_.brake);
	}
/*
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
*/
	return controlCmd2_;
}

/*@brief 倒车指令决策
 */
ant_msgs::ControlCmd2 AutoDrive::reverseDecisionMaking()
{
	std::lock_guard<std::mutex> lock2(cmd2_mutex_);

	//若当前状态为强制使用外部控制指令，则忽悠其他指令源
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


	
