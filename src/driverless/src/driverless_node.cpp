
#include "ros/ros.h"
#include "driverless/driverless_node.h"

#define __NAME__ "driverless"


AutoDrive::AutoDrive():
	AutoDriveBase(__NAME__),
    avoid_offset_(0.0),
    task_running_(false),
	system_state_(State_Idle),
	last_system_state_(State_Idle),
	has_new_task_(false),
	request_listen_(false),
	as_(nullptr),
	goal_preempt_(false)
{
	this->resetVehicleCtrlCmd();
}

AutoDrive::~AutoDrive()
{
}

bool AutoDrive::init(ros::NodeHandle nh,ros::NodeHandle nh_private)
{
	if(is_initialed_) return true;  //避免重复初始化

	nh_ = nh;  nh_private_ = nh_private;

	//获取参数
	nh_private_.param<float>("max_speed",expect_speed_,10.0);//km/h
	nh_private_.param<bool>("use_car_following",use_car_following_,false);
	nh_private_.param<bool>("use_avoiding",use_avoiding_,false);
	nh_private_.param<bool>("is_offline_debug",is_offline_debug_,false);
	nh_private_.param<bool>("use_extern_controller", use_extern_controller_, true);
	nh_private_.param<bool>("use_car_follower", use_car_follower_, false);
	std::string odom_topic = nh_private_.param<std::string>("odom_topic","/ll2utm_odom");
	
	initDiagnosticPublisher(nh_,__NAME__);

	if(!loadVehicleParams())
	{
		ROS_ERROR("[%s] Load vehicle parameters failed!", __NAME__);
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::ERROR,"Load vehicle parameters failed!");
		return false;
	}

	//订阅公用传感器数据
	subscribers_.push_back(nh_.subscribe(odom_topic, 1,&AutoDrive::odom_callback,this)); //odom
	subscribers_.push_back(nh_.subscribe("/vehicleStateSet",1,&AutoDrive::vehicleStateSet_CB,this)); //stateSet
	subscribers_.push_back(nh_.subscribe("/driverless/expect_path",1,&AutoDrive::goal_callback,this));

	//发布
	pub_cmd_ = nh_.advertise<driverless_common::VehicleCtrlCmd>("/vehicleCmdSet",1);

	pub_diagnostic_ = nh_.advertise<diagnostic_msgs::DiagnosticStatus>("/driverless/diagnostic",1);
	pub_new_goal_ = nh_.advertise<driverless_common::DoDriverlessTaskActionGoal>("/do_driverless_task/goal", 1);
	pub_driverless_state_ = nh.advertise<driverless_common::SystemState>("/driverless/system_state",1);
	
	//定时器                                                                      one_shot, auto_start
	cmd_timer_ = nh_.createTimer(ros::Duration(0.01), &AutoDrive::sendCmd_CB,this, false, true);

	timers_.push_back(nh_.createTimer(ros::Duration(0.1), &AutoDrive::publishDriverlessState,this));

	/*+初始化自动驾驶请求服务器*/
	as_  = new DoDriverlessTaskServer(nh_, "do_driverless_task", 
                              boost::bind(&AutoDrive::executeDriverlessCallback,this, _1), false);
	as_->registerPreemptCallback(boost::bind(&AutoDrive::goalPreemptCallback, this)); //注册抢占回调函数
	// 也可通过轮询as_->isPreemptRequested()判断
    as_->start();
	/*-初始化自动驾驶请求服务器*/

	// 车辆状态检查，等待初始化
	while(ros::ok() && !is_offline_debug_ ) //若离线调试,无需系统检查
	{
		std::string info;
		if(!vehicle_state_.validity(info))
		{
			ROS_INFO("[%s] %s",__NAME__, info.c_str());
			publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::WARN, info);
			ros::Duration(0.5).sleep();
		}
		else
			break;
	}

    //初始化路径跟踪控制器
    if(!tracker_.init(nh_, nh_private_))
	{
		ROS_ERROR("[%s] path tracker init false!",__NAME__);
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::ERROR,"Init path tracker failed!");
		return false;
	}
    ROS_INFO("[%s] path tracker init ok",__NAME__);
    //初始化跟车行驶控制器
    if(use_car_follower_)
	{
		if(!car_follower_.init(nh_, nh_private_))
		{
			ROS_ERROR("[%s] car follower init false!",__NAME__);
			publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::ERROR,"Init car follower failed!");
			return false;
		}
		else
    		ROS_INFO("[%s] car follower init ok",__NAME__);
	}
    //初始化外部控制器
	if(use_extern_controller_)
	{
		if(extern_controler_.init(nh_, nh_private_) && extern_controler_.start())
			ROS_INFO("[%s] extern controller init and start ok",__NAME__);
		else
		{
			ROS_ERROR("[%s] Initial or Start extern controller failed!", __NAME__);
			return false;
		}
		timers_.push_back(nh_.createTimer(ros::Duration(0.05), &AutoDrive::captureExernCmd_callback, this));
	}

    //初始化倒车控制器
    if(!reverse_controler_.init(nh_, nh_private_))
	{
		ROS_ERROR("[%s] Initial reverse controller failed!", __NAME__);
		return false;
	}
    ROS_INFO("[%s] reverse controller init ok",__NAME__);

    switchSystemState(State_Idle);
	//启动工作线程，等待新任务唤醒
	std::thread t(&AutoDrive::workingThread, this);
	t.detach();

	if(nh_private_.param<bool>("reverse_test", false))   //倒车测试
	{
        if(!reverse_controler_.loadReversePath(nh_private_.param<std::string>("reverse_path_file",""),
                                               nh_private_.param<bool>("reverse_path_flip",false)))
        {
            ROS_ERROR("[%s] load reverse path failed!", __NAME__);
            return false;
        }

        if(!switchSystemState(State_Reverse))
			return false;
		has_new_task_ = true;
		work_cv_.notify_one();
	}
	else if(nh_private_.param<bool>("drive_test", false)) //前进测试
	{
        if(!loadDriveTaskFile(nh_private_.param<std::string>("drive_path_file", "")))
        {
            ROS_ERROR("[%s] Load drive path file failed!", __NAME__);
            return false;
        }

		if(!switchSystemState(State_Drive))
			return false;
		has_new_task_ = true;
		work_cv_.notify_one();
	}
	
	is_initialed_ = true;
	return true;
}

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

	// return;  //若在此处退出, 此回调函数是否会被新目标再次调用? 意外中断?
	
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

			// 阻塞在此处, 等待当前任务完成后被唤醒,继续监听
			std::unique_lock<std::mutex> lock(listen_cv_mutex_);
			listen_cv_.wait(lock, [&](){return request_listen_;});
			request_listen_ = false;
		}
		ros::Duration(0.5).sleep();
	}
}

/*@brief 处理新任务目标
	①有效目标 对新目标进行预处理/载入相关文件/切换系统状态/唤醒工作线程，返回true
	②无效目标  返回false
 *@param goal 目标信息
*/
bool AutoDrive::handleNewGoal(const driverless_common::DoDriverlessTaskGoalConstPtr& goal, std::string& result)
{
	if(system_state_ != State_Idle)
	{
		result = "Current state not idle!";
		return false;
	}

	// 离线调试
	if(is_offline_debug_)
	{
        if(!loadDriveTaskFile(goal->roadnet_file, goal->roadnet_ext_file))
        {
            ROS_ERROR("[%s] Load drive path file failed!", __NAME__);
            result = "Load Path File Failed!";
            return false;
        }
        else
            ROS_INFO("[%s] Load drive path file complete!", __NAME__);

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
            if(!loadDriveTaskFile(goal->roadnet_file, goal->roadnet_ext_file))
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
			switchSystemState(State_Idle);
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
			switchSystemState(State_Idle);
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
		
		// 任务函数退出后,系统状态有两种情况, 1是当前的任务状态(任务正常完成并退出) 2是空闲状态(任务被打断,车辆停止后状态空闲)
		if(system_state_ != State_Idle)  // 情况1
			switchSystemState(State_TaskComplete);
		
		if(goal_preempt_){
			as_->setPreempted(driverless_common::DoDriverlessTaskResult(), "canceled or preempt by other goal");
			ROS_INFO("canceled or preempt by other goal");
			goal_preempt_ = false; //复位
		}
		else{
			driverless_common::DoDriverlessTaskResult result;
			result.success = true;
			as_->setSucceeded(result, "task completed");
		}
		
		std::unique_lock<std::mutex> lck(listen_cv_mutex_);
		request_listen_ = true;
		listen_cv_.notify_one(); //唤醒监听线程

		// 此处不能再切换系统状态，否则状态机制将被打破，导致任务无法正常运行！
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

	// 循环退出条件
	// 1. ros shutdown 一般为程序异常或ctrl+C
	// 2. 跟踪控制器停止运行,一般为到达终点
	// 3. 系统状态变为空闲, 其他任意状态均应持续决策, 
	//	  当任务被外部打断时,转向和速度决策应持续输出, 系统发布控制指令之前,对速度进行限制,直到车辆停止,状态跳变为State_Idle
	while(ros::ok()  && tracker_.isRunning() && system_state_ != State_Idle)
	{
		tracker_cmd_ = tracker_.getControlCmd();
		follower_cmd_= car_follower_.getControlCmd();
		
		auto cmd = this->decisionMaking(tracker_cmd_);     

		if(as_->isActive())
		{
			driverless_common::DoDriverlessTaskFeedback feedback;
			feedback.speed = cmd.speed;
			feedback.steer_angle = cmd.roadwheel_angle;
			as_->publishFeedback(feedback);
		}
		loop_rate.sleep();
	}
	ROS_INFO("[%s] drive work  completed...", __NAME__); 
	tracker_.stop();
	car_follower_.stop();
	
	task_running_ = false;
}

void AutoDrive::doReverseWork()
{
	reverse_controler_.setExpectSpeed(expect_speed_);
	reverse_controler_.start();
	
	ros::Rate loop_rate(20);
	
	ROS_ERROR("NOT ERROR: doReverseWork-> task_running_= true");
	task_running_ = true;
	while(ros::ok() && system_state_ != State_Idle && reverse_controler_.isRunning())
	{
		//ROS_INFO("[%s] new cycle.", __NAME__);
		reverse_cmd_ = reverse_controler_.getControlCmd();

		reverse_cmd_.display("doReverseWork");
		auto cmd = this->decisionMaking(reverse_cmd_);
		
		if(as_->isActive())
		{
			driverless_common::DoDriverlessTaskFeedback feedback;
			feedback.speed = cmd.speed;
			feedback.steer_angle = cmd.roadwheel_angle;
			as_->publishFeedback(feedback);
		}
		

		loop_rate.sleep();
	}
	reverse_controler_.stop();
	ROS_INFO("[%s] reverse work complete.", __NAME__);
	task_running_ = false;
}

void AutoDrive::doOfflineDebugWork()
{
	float percentage = 0.0;
	while(ros::ok() && system_state_ != State_Idle)
	{
		if(as_->isActive())
		{
			driverless_common::DoDriverlessTaskFeedback feedback;
			percentage += 0.1;
			feedback.percentage = percentage;
			as_->publishFeedback(feedback);

			if(feedback.percentage >= 100)
				break;
		}

		ros::Duration(0.5).sleep();
	}
}

/*@brief 前进控制指令决策
 * 指令源包括: 避障控速/跟车控速/路径跟踪控转向和速度
 * 控制指令优先级 ①外部控制指令
                ②避障速度控制
				③跟车速度控制
 */
const driverless_common::VehicleCtrlCmd AutoDrive::decisionMaking(const controlCmd_t& tracker_cmd)
{
	std::unique_lock<std::mutex> lock2(cmd_msg_mutex_);
	//若当前状态为强制使用外部控制指令，则忽悠其他指令源
	if(system_state_ == State_ForceExternControl)
	{
		std::lock_guard<std::mutex> lock(extern_cmd_mutex_);
		vehicleCtrlCmd_.roadwheel_angle = extern_cmd_.roadWheelAngle;
		vehicleCtrlCmd_.speed = extern_cmd_.speed;
		vehicleCtrlCmd_.brake = extern_cmd_.brake;
		//转向灯
		if(extern_cmd_.turnLight == 1) vehicleCtrlCmd_.turnlight_l = true;
		else if(extern_cmd_.turnLight == 2) vehicleCtrlCmd_.turnlight_r = true;
		else if(extern_cmd_.turnLight == 0)	vehicleCtrlCmd_.turnlight_r = vehicleCtrlCmd_.turnlight_l = false;

		return vehicleCtrlCmd_;
	}
	vehicleCtrlCmd_.roadwheel_angle = tracker_cmd.roadWheelAngle;
	vehicleCtrlCmd_.speed = tracker_cmd.speed; //优先使用跟踪器速度指令
	vehicleCtrlCmd_.brake = tracker_cmd.brake;
	
	extern_cmd_mutex_.lock();
	if(extern_cmd_.speed_validity){     //如果外部速度指令有效,则使用外部速度
		vehicleCtrlCmd_.speed = extern_cmd_.speed;
		vehicleCtrlCmd_.brake = extern_cmd_.brake;
	}
	if(avoid_cmd_.speed_validity){      //如果避障速度有效，选用最小速度
		vehicleCtrlCmd_.speed = std::min(vehicleCtrlCmd_.speed, avoid_cmd_.speed);
		vehicleCtrlCmd_.brake = max(vehicleCtrlCmd_.brake, avoid_cmd_.brake);
	}
	if(follower_cmd_.speed_validity){   //如果跟车速度有效，选用最小速度
		vehicleCtrlCmd_.speed = std::min(vehicleCtrlCmd_.speed, follower_cmd_.speed);
		vehicleCtrlCmd_.brake = max(vehicleCtrlCmd_.brake, follower_cmd_.brake);
	}

	//转向灯
	if(extern_cmd_.turnLight == 1)
		vehicleCtrlCmd_.turnlight_l = true;
	else if(extern_cmd_.turnLight == 2)
		vehicleCtrlCmd_.turnlight_r = true;
	else if(extern_cmd_.turnLight == 0)
	{
		vehicleCtrlCmd_.turnlight_r = false;
		vehicleCtrlCmd_.turnlight_l = false;
	}
	extern_cmd_mutex_.unlock();


	// 任务中断标志 系统需控制车速直到停车, 但转角仍由当前任务决策(否则可能出现意外)
	// 若将转角置零, 然后限速,在车停下来之前将继续行驶一段距离,可能冲出车道
	if(system_state_ == State_TaskPreempt)
	{
		// 系统任务被中断, 限制车速使车辆停车
		// 当速度足够低时, 切换系统状态为空闲
		vehicleCtrlCmd_.speed = 0.0;
		vehicleCtrlCmd_.brake = max(vehicleCtrlCmd_.brake, 60); //取最大制动力

		if(vehicle_state_.speedLowEnough())
		{
			cmd_msg_mutex_.unlock();  //务必解锁，否则switchSystemState将产生死锁
			switchSystemState(State_Idle);
		}
	}

	return vehicleCtrlCmd_;
}


bool AutoDrive::isGpsPointValid(const GpsPoint& point)
{
	//std::cout << point.x  << "\t"  <<point.y << std::endl;
	if(fabs(point.x) > 100 && fabs(point.y) > 100)
		return true;
	return false;
}


/* @brief 切换系统状态
 * 根据系统状态设置档位，直到档位设置成功或超时。
*/
bool AutoDrive::switchSystemState(int state)
{
	//这里应该使用递归锁,否则将产生死锁
	RecursiveLock rlock(switchStateRMutex_);
	
	ROS_ERROR("[%s] NOT ERROR switchSystemState: %s", __NAME__, StateName[state].c_str());
	if(system_state_ == state) return true; //防止重复操作
	
	last_system_state_ = system_state_;
    system_state_ = state;

    //状态为前进，自动驾驶模式开，档位置D
	if(state == State_Drive)
	{
		cmd_msg_mutex_.lock();
		this->resetVehicleCtrlCmd(); //全部复位
		//确保指令正确，防止指令已更改状态未更新
		vehicleCtrlCmd_.gear = vehicleCtrlCmd_.GEAR_DRIVE;
		vehicleCtrlCmd_.driverless = true;
		cmd_msg_mutex_.unlock();  //需要及时解锁控制指令,以便于发送线程使用

        //启动控制指令发送
		setSendControlCmdEnable(true);

		//等待档位切换成功或超时
        return waitGearOk(driverless_common::VehicleState::GEAR_DRIVE);
	}
    //状态为后退，自动驾驶模式开，档位置R
	else if(state == State_Reverse) 
	{
		cmd_msg_mutex_.lock();
		this->resetVehicleCtrlCmd();
		vehicleCtrlCmd_.gear = vehicleCtrlCmd_.GEAR_REVERSE;
		vehicleCtrlCmd_.driverless = true;
		cmd_msg_mutex_.unlock();

        //启动控制指令发送
		setSendControlCmdEnable(true);
		//等待档位切换成功或超时
    	return waitGearOk(driverless_common::VehicleState::GEAR_REVERSE);
	}
    //状态为空闲，控制指令不能停止,否则复位指令将无法发送
	else if(state == State_Idle)  //空闲
	{
		cmd_msg_mutex_.lock();
		this->resetVehicleCtrlCmd();
		vehicleCtrlCmd_.driverless = true;
		vehicleCtrlCmd_.hand_brake = true; //系统空闲, 拉上手刹,防止溜车
		cmd_msg_mutex_.unlock();
		setSendControlCmdEnable(true);
		return true;
	}
	else if(state == State_Stop)  // 停车
	{
		// 上次状态为空闲, 当前为停止,则不进行处理
		if(last_system_state_ == State_Idle)
		{
			system_state_ = last_system_state_;
			return true;
		}

		cmd_msg_mutex_.lock();
		this->resetVehicleCtrlCmd();
		vehicleCtrlCmd_.driverless = true;
		vehicleCtrlCmd_.speed = 0.0;
		vehicleCtrlCmd_.brake = 60.0;
		cmd_msg_mutex_.unlock();

		setSendControlCmdEnable(true);

		waitSpeedZero(); //等待汽车速度为0
        return switchSystemState(State_Idle); //递归调用， 状态置为空闲
	}
	else if(state == State_TaskComplete)  // 任务完成
	{
		return switchSystemState(State_Idle);
	}
	else if(state == State_TaskPreempt)  //任务被中断
	{
		
	}
    //准备切换到前进状态
    else if(state == State_SwitchToDrive)
    {
		setSendControlCmdEnable(true);  //启动控制指令发送

		if(!vehicle_state_.isDriveGear()) //当前不是前进档, 则需要进行制动等待车速为0
		{
			cmd_msg_mutex_.lock();
			vehicleCtrlCmd_.driverless = true;
			vehicleCtrlCmd_.speed = 0;
			vehicleCtrlCmd_.brake = 60.0;
			cmd_msg_mutex_.unlock();
			waitSpeedZero();
		}

        return switchSystemState(State_Drive); //递归调用，状态置为前进
    }
    //切换到倒车状态
    else if(state == State_SwitchToReverse)
    {
		setSendControlCmdEnable(true);  //启动控制指令发送

		if(!vehicle_state_.isReverseGear()) //当前不是后退档, 则需要进行制动等待车速为0
		{
			cmd_msg_mutex_.lock();
			vehicleCtrlCmd_.driverless = true;
			vehicleCtrlCmd_.speed = 0;
			vehicleCtrlCmd_.brake = 60.0;
			cmd_msg_mutex_.unlock();
			waitSpeedZero();
		}

        return switchSystemState(State_Reverse); //递归调用，状态置为倒车
    }
	else if(state == State_ForceExternControl)
	{
		//No operation
	}
	return true;
}

void AutoDrive::setSendControlCmdEnable(bool flag)
{
	static bool last_flag = false;
	if(flag == last_flag)
		return;
	last_flag = flag;

	if(flag)
		cmd_timer_.start();
	else
		cmd_timer_.stop();
}


void AutoDrive::sendCmd_CB(const ros::TimerEvent&)
{
	std::lock_guard<std::mutex> lock(cmd_msg_mutex_);
	pub_cmd_.publish(vehicleCtrlCmd_);
}


/*@brief 定时捕获外部控制指令,  
  -当系统正在执行任务时，由decisionMaking更新终端控制指令
  -而当系统处于其他状态时由captureExernCmd_callback定时更新终端控制指令,以确保外部控制器随时生效

 - 当外部控制指令有效时，将系统状态置为强制使用外部指令，并将指令更新到终端控制指令
 - 当外部控制指令失效时，将系统状态切换到原来的状态
 */
void AutoDrive::captureExernCmd_callback(const ros::TimerEvent&)
{
	static bool last_validity = false;
	std::lock_guard<std::mutex> _lock(extern_cmd_mutex_);
	extern_cmd_ = extern_controler_.getControlCmd();
	//std::cout << "extern_cmd_.validity: " << extern_cmd_.validity << std::endl;
	if(extern_cmd_.validity) //当前外部指令有效
 	{
		setSendControlCmdEnable(true); //使能控制指令发送
		if(system_state_ != State_ForceExternControl)
			switchSystemState(State_ForceExternControl);
		//extern_cmd_.display("Extern Cmd ");

		std::lock_guard<std::mutex> lock(cmd_msg_mutex_);
		vehicleCtrlCmd_.hand_brake = extern_cmd_.hand_brake;
		vehicleCtrlCmd_.speed = extern_cmd_.speed;
		vehicleCtrlCmd_.brake = extern_cmd_.brake;
		vehicleCtrlCmd_.roadwheel_angle = extern_cmd_.roadWheelAngle;
		vehicleCtrlCmd_.gear = extern_cmd_.gear;
	}
	else
	{
		 //当前无效，上次有效，切换为历史状态
		if(last_validity)
			switchSystemState(last_system_state_);
	}

	last_validity = extern_cmd_.validity;
	extern_cmd_mutex_.unlock();
}

void AutoDrive::odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	Pose pose;
	pose.x =  msg->pose.pose.position.x;
	pose.y =  msg->pose.pose.position.y;
	pose.yaw = msg->pose.covariance[0];
	
	vehicle_state_.setPose(pose);
}

void AutoDrive::goal_callback(const pathplaning_msgs::expected_path::ConstPtr& msg)
{
	driverless_common::DoDriverlessTaskActionGoal::Ptr  actionGoal = 
		driverless_common::DoDriverlessTaskActionGoal::Ptr(new driverless_common::DoDriverlessTaskActionGoal);
	actionGoal->header.stamp = ros::Time::now();

	driverless_common::DoDriverlessTaskGoal& goal = actionGoal->goal;
	if(msg->direction == msg->DIRECTION_DRIVE)
		goal.task = goal.DRIVE_TASK;
	else if(msg->direction == msg->DIRECTION_REVERSE)
		goal.task = goal.REVERSE_TASK;
	goal.type = goal.PATH_TYPE;
	goal.target_path = msg->points;
	goal.expect_speed = msg->expect_speed;
	goal.path_resolution = msg->path_resolution;
	pub_new_goal_.publish(actionGoal);

	ROS_INFO("[%s] Receive expect path from extern path planner.", __NAME__);
}

void AutoDrive::vehicleStateSet_CB(const driverless_common::VehicleState::ConstPtr& msg)
{
	WriteLock writeLock(vehicle_state_.wr_mutex);
	vehicle_state_.speed = msg->speed;
	vehicle_state_.steer_angle = msg->roadwheel_angle;
	vehicle_state_.speed_validity = msg->speed_validity;
	vehicle_state_.steer_validity = msg->roadwheel_angle_validity;

	vehicle_state_.gear = msg->gear;
	vehicle_state_.driverless_mode = msg->driverless;
}


bool AutoDrive::loadVehicleParams()
{
	bool ok = true;
	vehicle_params_.max_roadwheel_angle = nh_private_.param<float>("vehicle/max_roadwheel_angle",0.0);
	vehicle_params_.min_roadwheel_angle = nh_private_.param<float>("vehicle/min_roadwheel_angle",0.0);
	vehicle_params_.min_radius          = nh_private_.param<float>("vehicle/min_radius",0.0);
	vehicle_params_.max_speed = nh_private_.param<float>("vehicle/max_speed",0.0);
	vehicle_params_.wheel_base = nh_private_.param<float>("vehicle/wheel_base",0.0);
	vehicle_params_.wheel_track = nh_private_.param<float>("vehicle/wheel_track",0.0);
	vehicle_params_.width = nh_private_.param<float>("vehicle/width",0.0);
	vehicle_params_.length = nh_private_.param<float>("vehicle/length",0.0);
	std::string node = ros::this_node::getName();
	if(vehicle_params_.max_roadwheel_angle == 0.0)
	{
		ROS_ERROR("[%s] No parameter %s/vehicle/max_roadwheel_angle.",__NAME__,node.c_str());
		ok = false;
	}
	if(vehicle_params_.min_roadwheel_angle == 0.0)
	{
		ROS_ERROR("[%s] No parameter %s/vehicle/min_roadwheel_angle.",__NAME__,node.c_str());
		ok = false;
	}
	if(vehicle_params_.min_radius == 0.0)
	{
		ROS_ERROR("[%s] No parameter %s/vehicle/min_radius.",__NAME__,node.c_str());
		ok = false;
	}
	if(vehicle_params_.max_speed == 0.0)
	{
		ROS_ERROR("[%s] No parameter %s/vehicle/max_speed.",__NAME__,node.c_str());
		ok = false;
	}
	if(vehicle_params_.wheel_base == 0.0)
	{
		ROS_ERROR("[%s] No parameter %s/vehicle/wheel_base.",__NAME__,node.c_str());
		ok = false;
	}
	if(vehicle_params_.wheel_track == 0.0)
	{
		ROS_ERROR("[%s] No parameter %s/vehicle/wheel_track.",__NAME__,node.c_str());
		ok = false;
	}
	if(vehicle_params_.width == 0.0)
	{
		ROS_ERROR("[%s] No parameter %s/vehicle/width.",__NAME__,node.c_str());
		ok = false;
	}
	if(vehicle_params_.length == 0.0)
	{
		ROS_ERROR("[%s] No parameter %s/vehicle/length.",__NAME__,node.c_str());
		ok = false;
	}
	if(ok) vehicle_params_.validity = true;
	return ok;
}

/*@brief 载入前进任务文件，路径点位信息/停车点信息/拓展路径
		 若路径附加信息文件不存在，不返回错误，以应对非常规路线
*/
bool AutoDrive::loadDriveTaskFile(const std::string& points_file, const std::string& extend_file)
{
	//载入路网文件
    if(! loadPathPoints(points_file, global_path_))
	{
	    ROS_ERROR("[%s] Load path file failed!",__NAME__);
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::ERROR,"Load path file failed!");
		return false;
	}

	//载入路径附加信息
    if(extend_file!="" && !loadPathAppendInfos(extend_file, global_path_, __NAME__))
	{
		ROS_ERROR("[%s] Load path infomation failed!",__NAME__);
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::ERROR,"Load path infomation failed!");
		return false;
	}
	return extendPath(global_path_, 20.0); //路径拓展延伸
}

/*@brief 设置前进任务目标路径点集， 自行计算路径曲率信息
 */
bool AutoDrive::setDriveTaskPathPoints(const driverless_common::DoDriverlessTaskGoalConstPtr& goal)
{
	size_t len = goal->target_path.size();
	if(len == 0) 
		return false;
		
	global_path_.clear();
	global_path_.points.resize(len);
	for(size_t i=0; i<len; ++i)
	{
		const geometry_msgs::Pose2D& pose = goal->target_path[i];
		GpsPoint& point = global_path_.points[i];
		point.x = pose.x;
		point.y = pose.y;
		point.yaw = pose.theta;
	}
	calPathCurvature(global_path_); //计算路径曲率
    global_path_.resolution = goal->path_resolution;
	global_path_.final_index = global_path_.points.size() - 1 ;  //设置终点索引为最后一个点
	//算法根据停车点距离控制车速，若没有附加路径信息将导致到达终点前无法减速停车！
	//因此，此处将终点设为一个永久停车点，
	global_path_.park_points.points.emplace_back(global_path_.final_index, 0.0); 
	return true;
}

/*@brief 等待车速减为0
*/
void AutoDrive::waitSpeedZero()
{
    while(ros::ok() && vehicle_state_.getSpeed(LOCK)!=0.0)
		ros::Duration(0.2).sleep();
}

/*@brief 等待档位切换成功
 */
bool AutoDrive::waitGearOk(int gear)
{
	float waitTime = 0.0;
    while(ros::ok() && vehicle_state_.getGear() != gear && system_state_!= State_Idle)
    {
		ros::Duration(0.2).sleep();
		waitTime += 0.2;
		ROS_INFO("[%s] wait for gear: %d", __NAME__, gear);
		if(waitTime > 5.0)
		{
			ROS_ERROR("[%s] wait for gear: %d timeout!", __NAME__, gear);
			return false;
		}
	}
	return true;
}

/*@brief 发布自动驾驶状态信息
*/
void AutoDrive::publishDriverlessState(const ros::TimerEvent&)
{
	if(pub_driverless_state_.getNumSubscribers())
	{
		driverless_common::SystemState driverless_state;
		const Pose pose = vehicle_state_.getPose(LOCK);
		const float speed = vehicle_state_.getSpeed(LOCK);
		driverless_state.header.stamp = ros::Time::now();
		driverless_state.location_source = "gps";

		driverless_state.position_x = pose.x;
		driverless_state.position_y = pose.y;
		driverless_state.yaw = pose.yaw;
		driverless_state.vehicle_speed =  speed;
		driverless_state.roadwheel_angle = vehicle_state_.getSteerAngle(LOCK);
		// driverless_state.lateral_error = g_lateral_err_;
		// driverless_state.yaw_error = g_yaw_err_;
		driverless_state.task_state = StateName[system_state_];
		driverless_state.command_speed = vehicleCtrlCmd_.speed;

		pub_driverless_state_.publish(driverless_state);
	}
}

void AutoDrive::resetVehicleCtrlCmd()
{
	//调用前手动加锁
	//std::lock_guard<std::mutex> _lock(cmd_msg_mutex_);
	vehicleCtrlCmd_.gear = vehicleCtrlCmd_.GEAR_NEUTRAL;
	vehicleCtrlCmd_.driverless = false;
	vehicleCtrlCmd_.hand_brake = false;
	vehicleCtrlCmd_.speed = 0.0;
	vehicleCtrlCmd_.brake = 0.0;
	vehicleCtrlCmd_.roadwheel_angle = 0.0;
	vehicleCtrlCmd_.emergency_brake = false;
	vehicleCtrlCmd_.accelerate = 0.0;
	vehicleCtrlCmd_.turnlight_l = false;
	vehicleCtrlCmd_.turnlight_r = false;
	vehicleCtrlCmd_.brake_light = false;
	vehicleCtrlCmd_.horn = false;
}

void AutoDrive::goalPreemptCallback()
{
	if(as_->isActive())
	{
		goal_preempt_=true;
		switchSystemState(State_TaskPreempt);
	}
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