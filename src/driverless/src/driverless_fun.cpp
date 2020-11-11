#include "ros/ros.h"
#include "driverless/driverless_node.h"
#define __NAME__ "driverless"

/*@function 在此文件中定义AutoDrive初始化成员函数、功能函数等
 *          在driverless_node.cpp对关键函数进行定义，分布存储以提高可读性

*/

AutoDrive::AutoDrive():
	AutoDriveBase(__NAME__),
    avoid_offset_(0.0),
	system_state_(State_Idle),
	has_new_task_(false),
	as_(nullptr)
{
	controlCmd1_.set_driverlessMode = true;
	controlCmd1_.set_handBrake = false;
	controlCmd2_.set_gear = controlCmd2_.GEAR_DRIVE;
	controlCmd2_.set_speed = 0.0;
	controlCmd2_.set_brake = 0.0;
	controlCmd2_.set_roadWheelAngle = 0.0;
}

AutoDrive::~AutoDrive()
{

}

bool AutoDrive::isGpsPointValid(const GpsPoint& point)
{
	//std::cout << point.x  << "\t"  <<point.y << std::endl;
	if(fabs(point.x) > 100 && fabs(point.y) > 100)
		return true;
	return false;
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
	std::string odom_topic = nh_private_.param<std::string>("odom_topic","/ll2utm_odom");
	
	initDiagnosticPublisher(nh_,__NAME__);

	if(!loadVehicleParams())
	{
		ROS_ERROR("[%s] Load vehicle parameters failed!", __NAME__);
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::ERROR,"Load vehicle parameters failed!");
		return false;
	}

	//订阅公用传感器数据
	sub_odom_ = nh_.subscribe(odom_topic, 1,&AutoDrive::odom_callback,this);
	sub_vehicleState1_ = nh_.subscribe("/vehicleState1",1,&AutoDrive::vehicleState1_callback,this);
	sub_vehicleState2_ = nh_.subscribe("/vehicleState2",1,&AutoDrive::vehicleSpeed_callback,this);
	sub_vehicleState4_ = nh_.subscribe("/vehicleState4",1,&AutoDrive::vehicleState4_callback,this);

	//发布
	pub_cmd1_ = nh_.advertise<ant_msgs::ControlCmd1>("/controlCmd1",1);
	pub_cmd2_ = nh_.advertise<ant_msgs::ControlCmd2>("/controlCmd2",1);
	pub_diagnostic_ = nh_.advertise<diagnostic_msgs::DiagnosticStatus>("driverless/diagnostic",1);

	//定时器                                                                           one_shot, auto_start
	cmd1_timer_ = nh_.createTimer(ros::Duration(0.02), &AutoDrive::sendCmd1_callback,this, false, false);
	cmd2_timer_ = nh_.createTimer(ros::Duration(0.01), &AutoDrive::sendCmd2_callback,this, false, false);
	
		
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

	/*+初始化自动驾驶请求服务器*/
	as_  = new DoDriverlessTaskServer(nh_private_, "do_driverless_task", 
                              boost::bind(&AutoDrive::executeDriverlessCallback,this, _1), false);
    as_->start();
	/*-初始化自动驾驶请求服务器*/

    //初始化路径跟踪控制器
    if(!tracker_.init(nh_, nh_private_))
	{
		ROS_ERROR("[%s] path tracker init false!",__NAME__);
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::ERROR,"Init path tracker failed!");
		return false;
	}
    ROS_INFO("[%s] path tracker init ok",__NAME__);
    //初始化跟车行驶控制器
    if(!car_follower_.init(nh_, nh_private_))
	{
		ROS_ERROR("[%s] car follower init false!",__NAME__);
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::ERROR,"Init car follower failed!");
		return false;
	}
    ROS_INFO("[%s] car follower init ok",__NAME__);
    //初始化外部控制器
    extern_controler_.init(nh_, nh_private_);
    ROS_INFO("[%s] extern controller init ok",__NAME__);
    //初始化倒车控制器
    if(!reverse_controler_.init(nh_, nh_private_))
	{
		ROS_ERROR("[%s] Initial reverse controller failed!", __NAME__);
		return false;
	}
    ROS_INFO("[%s] reverse controller init ok",__NAME__);

    switchSystemState(State_Idle);

	if(nh_private_.param<bool>("reverse_test", false))   //倒车测试
	{
        if(!reverse_controler_.loadReversePath(nh_private_.param<std::string>("reverse_path_file",""),
                                               nh_private_.param<bool>("reverse_path_flip",false)))
        {
            ROS_ERROR("[%s] load reverse path failed!", __NAME__);
            return false;
        }

        switchSystemState(State_Reverse);
		has_new_task_ = true;
	}
	else if(nh_private_.param<bool>("drive_test", false)) //前进测试
	{
        if(!loadDriveTaskFile(nh_private_.param<std::string>("drive_path_file", "")))
        {
            ROS_ERROR("[%s] Load drive path file failed!", __NAME__);
            return false;
        }

        switchSystemState(State_Drive);
		has_new_task_ = true;
	}
	
	std::thread t(&AutoDrive::workingThread, this);
	t.detach();

	is_initialed_ = true;
	return true;
}

void AutoDrive::executeDriverlessCallback(const driverless::DoDriverlessTaskGoalConstPtr& goal)
{
	ROS_INFO("[%s] executeDriverlessCallback!", __NAME__);
	std::cout << "goal->type: " << int(goal->type)  << "\t"
	          << "goal->task: " << int(goal->task)  << "\t"
			  << "goal->file: " << goal->roadnet_file <<std::endl;

	handleNewGoal(goal);    
}

void AutoDrive::handleNewGoal(const driverless::DoDriverlessTaskGoalConstPtr& goal)
{
	switchSystemState(State_Stop); //新请求，无论如何先停止, 暂未解决现任务文件覆盖旧文件导致的自动驾驶异常问题，
                                   //因此只能停车后开始新任务
                                   //实则，若新任务与当前任务驾驶方向一致，只需合理的切换路径文件即可！
                                   //已经预留了切换接口，尚未解决运行中清空历史文件带来的隐患 

	if(goal->task == goal->DRIVE_TASK)  //前进任务
    {
        //给定目标点位置，调用路径规划
        if(goal->type == goal->POSE_TYPE) 
        {
        }
        //指定驾驶路径
        else if(goal->type == goal->PATH_TYPE)
        {/*
        global_path_.clear(); //此处清空全局路径，是否会导致其他使用该路径的进程出错？
                              //任务切换时，应暂停其他所有任务
        size_t len = goal->target_path.size();
        global_path_.points.reserve(len);

        for(const geometry_msgs::Pose2D& pose : goal->target_path)
        {
            GpsPoint point;
            point.x = pose.x;
            point.y = pose.y;
            point.yaw = pose.theta;

            global_path_.points.push_back(point);
        }
        global_path_.resolution = goal->path_resolution;*/
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
                return ;
            }
        }
        else
        {
            ROS_ERROR("[%s] Request type error!", __NAME__);
			as_->setAborted(driverless::DoDriverlessTaskResult(), "Aborting on unknown goal type! ");
            return ;
        }
        this->expect_speed_ = goal->expect_speed;
        has_new_task_ = true;
        //切换系统状态为: 切换到前进
        switchSystemState(State_SwitchToDrive);
		return;
    }
    else if(goal->task == goal->REVERSE_TASK)  //倒车任务
    {
        //给定目标点位置，调用路径规划
        if(goal->type == goal->POSE_TYPE) 
        {
            Pose target_pose;
            target_pose.x = goal->target_pose.x;
            target_pose.y = goal->target_pose.y;
            target_pose.yaw = goal->target_pose.theta;
            if(!reverse_controler_.reversePathPlan(target_pose))
            {
                driverless::DoDriverlessTaskResult res;
                res.success = false;
                as_->setAborted(res, "Aborting on reverse goal, because it is invalid ");
                return;
            }
        }
        //指定驾驶路径
        else if(goal->type == goal->PATH_TYPE)
        {
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
                return ;
            }
			ROS_INFO("[%s] load reverse path ok!", __NAME__);
        }
        else
        {
            ROS_ERROR("[%s] Request type error!", __NAME__);
			as_->setAborted(driverless::DoDriverlessTaskResult(), "Aborting on unknown goal type! ");
            return ;
        }
        this->expect_speed_ = goal->expect_speed;
        has_new_task_ = true;
        //切换系统状态为: 切换到倒车
        switchSystemState(State_SwitchToReverse);
		return;
    }
	else
	{
		ROS_ERROR("[%s] Unknown task type!", __NAME__);
		as_->setAborted(driverless::DoDriverlessTaskResult(), "Aborting on unknown task! ");
		return;
	}
	as_->setAborted(driverless::DoDriverlessTaskResult(), "Aborting on unknown error! ");
}


/* @brief 切换系统状态
 * 根据系统状态设置档位，直到档位设置成功。
*/
void AutoDrive::switchSystemState(int state)
{
    system_state_ = state;

    //状态为前进，自动驾驶模式开，档位置D
	if(state == State_Drive) 
	{
		if(isDriveGear()) return;

		cmd1_mutex_.lock();
		controlCmd1_.set_driverlessMode = true;
		controlCmd1_.set_handBrake = false;
		cmd1_mutex_.unlock();

		cmd2_mutex_.lock();
		controlCmd2_.set_gear = controlCmd2_.GEAR_DRIVE;
		controlCmd2_.set_speed = 0.0;
		controlCmd2_.set_brake = 0.0;
		controlCmd2_.set_roadWheelAngle = 0.0;
		controlCmd2_.set_emergencyBrake = false;
		cmd2_mutex_.unlock();

        //启动控制指令发送
		setSendControlCmdEnable(true);

		//等待档位切换成功
        waitGearOk(ant_msgs::State1::GEAR_DRIVE);
	}
    //状态为后退，自动驾驶模式开，档位置R
	else if(state == State_Reverse) 
	{
		if(isReverseGear()) return;

		cmd1_mutex_.lock();
		controlCmd1_.set_driverlessMode = true;
		controlCmd1_.set_handBrake = false;
		cmd1_mutex_.unlock();

		cmd2_mutex_.lock();
		controlCmd2_.set_gear = controlCmd2_.GEAR_REVERSE;
		controlCmd2_.set_speed = 0.0;
		controlCmd2_.set_brake = 0.0;
		controlCmd2_.set_roadWheelAngle = 0.0;
		controlCmd2_.set_emergencyBrake = false;
		cmd2_mutex_.unlock();

        //启动控制指令发送
		setSendControlCmdEnable(true);

		//等待档位切换成功
        waitGearOk(ant_msgs::State1::GEAR_REVERSE);
	}
    //状态为空闲，停止发送控制指令
	else if(state == State_Idle)  //空闲
	{
		setSendControlCmdEnable(false);
	}
    //状态为停止，自动驾驶模式开, 速度置零，拉手刹
    //车辆停止后，切换为空挡
	else if(state == State_Stop)  
	{
		cmd1_mutex_.lock();
		controlCmd1_.set_driverlessMode = true;
		controlCmd1_.set_handBrake = true; //拉手刹
		cmd1_mutex_.unlock();

		cmd2_mutex_.lock();
		//controlCmd2_.set_gear = controlCmd2_.GEAR_NEUTRAL;
		controlCmd2_.set_speed = 0.0;
		controlCmd2_.set_brake = 0.0;
		controlCmd2_.set_roadWheelAngle = 0.0;
		controlCmd2_.set_emergencyBrake = false;
		cmd2_mutex_.unlock();
		setSendControlCmdEnable(true);

		waitSpeedZero(); //等待汽车速度为0
		
        cmd2_mutex_.lock(); //指令预设为N档
		controlCmd2_.set_gear = controlCmd2_.GEAR_NEUTRAL;
        cmd2_mutex_.unlock();

        switchSystemState(State_Idle); //递归调用， 状态置为空闲
	}
    //准备切换到前进状态
    else if(state == State_SwitchToDrive)
    {
        //已经D档，直接退出
        if(isDriveGear()) return;
        
        //非D档，速度置0，然后切D档
        cmd1_mutex_.lock();
		controlCmd1_.set_driverlessMode = true;
		controlCmd1_.set_handBrake = false;
		cmd1_mutex_.unlock();

		cmd2_mutex_.lock();
		controlCmd2_.set_speed = 0.0;
		controlCmd2_.set_brake = 0.0;
		cmd2_mutex_.unlock();

		setSendControlCmdEnable(true);  //启动控制指令发送
        waitSpeedZero();                //等待速度为0
        switchSystemState(State_Drive); //递归调用，状态置为前进
    }
    //切换到倒车状态
    else if(state == State_SwitchToReverse)
    {
        //已经为R档，直接返回
        if(isReverseGear()) return;
        
        //非R档，速度置0，然后切R档
        cmd1_mutex_.lock();
		controlCmd1_.set_driverlessMode = true;
		controlCmd1_.set_handBrake = false;
		cmd1_mutex_.unlock();

		cmd2_mutex_.lock();
		controlCmd2_.set_speed = 0.0;
		controlCmd2_.set_brake = 0.0;
		cmd2_mutex_.unlock();

		setSendControlCmdEnable(true);  //启动控制指令发送
        waitSpeedZero();                //等待速度为0
        switchSystemState(State_Reverse); //递归调用，状态置为倒车
    }
}

void AutoDrive::setSendControlCmdEnable(bool flag)
{
	static bool last_flag = false;
	if(flag == last_flag)
		return;
	last_flag = flag;

	if(flag)
	{
		cmd1_timer_.start();
		cmd2_timer_.start();
	}
	else
	{
		cmd1_timer_.stop();
		cmd2_timer_.stop();
	}
}

void AutoDrive::sendCmd1_callback(const ros::TimerEvent&)
{
	std::lock_guard<std::mutex> lock(cmd1_mutex_);
	pub_cmd1_.publish(controlCmd1_);
}

void AutoDrive::sendCmd2_callback(const ros::TimerEvent&)
{
	std::lock_guard<std::mutex> lock(cmd2_mutex_);
	pub_cmd2_.publish(controlCmd2_);
}

void AutoDrive::odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	Pose pose;
	pose.x =  msg->pose.pose.position.x;
	pose.y =  msg->pose.pose.position.y;
	pose.yaw = msg->pose.covariance[0];
	
	vehicle_state_.setPose(pose);
}

void AutoDrive::vehicleSpeed_callback(const ant_msgs::State2::ConstPtr& msg)
{
	if(msg->vehicle_speed >20.0)
	{
		vehicle_state_.speed_validity = false;
		return;
	}
		
	vehicle_state_.setSpeed(msg->vehicle_speed); //  m/s
	vehicle_state_.speed_validity = true;
}

void AutoDrive::vehicleState4_callback(const ant_msgs::State4::ConstPtr& msg)
{
	vehicle_state_.setSteerAngle(msg->roadwheelAngle);
	vehicle_state_.steer_validity = true;
}

void AutoDrive::vehicleState1_callback(const ant_msgs::State1::ConstPtr& msg)
{
	vehicle_state_.setGear(msg->act_gear);
}

bool AutoDrive::isReverseGear()
{
	return vehicle_state_.getGear() == ant_msgs::State1::GEAR_REVERSE;
}

bool AutoDrive::isDriveGear()
{
	return vehicle_state_.getGear() == ant_msgs::State1::GEAR_DRIVE;
}

bool AutoDrive::isNeutralGear()
{
	return vehicle_state_.getGear() == ant_msgs::State1::GEAR_NEUTRAL;
}

bool AutoDrive::loadVehicleParams()
{
	bool ok = true;
	vehicle_params_.max_roadwheel_angle = nh_private_.param<float>("vehicle/max_roadwheel_angle",0.0);
	vehicle_params_.min_roadwheel_angle = nh_private_.param<float>("vehicle/min_roadwheel_angle",0.0);
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

/*@brief 载入任务文件
*/
bool AutoDrive::loadDriveTaskFile(const std::string& file)
{
	//载入路网文件
	if(! loadPathPoints(file, global_path_))
	{
	    ROS_ERROR("[%s] Load path file failed!",__NAME__);
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::ERROR,"Load path file failed!");
		return false;
	}

	//载入路径信息
	std::string path_infos_file = file.substr(0,file.find_last_of(".")) + "_info.xml";
	if(!loadPathInfos(path_infos_file, global_path_, __NAME__))
	{
		ROS_ERROR("[%s] Load path infomation failed!",__NAME__);
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::ERROR,"Load path infomation failed!");
		return false;
	}
	return extendPath(global_path_, 20.0); //路径拓展延伸
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
void AutoDrive::waitGearOk(int gear)
{
    while(ros::ok() && vehicle_state_.getGear() != gear)
			ros::Duration(0.2).sleep();
}