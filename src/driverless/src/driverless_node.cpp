
#include "ros/ros.h"
#include "driverless/driverless_node.h"

#define __NAME__ "driverless"

AutoDrive::AutoDrive():
	AutoDriveBase(__NAME__),
    nh_private_("~"),
    avoid_offset_(0.0)
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

bool AutoDrive::init()
{
	//获取参数
	nh_private_.param<float>("max_speed",max_speed_,10.0);//km/h
	nh_private_.param<bool>("use_car_following",use_car_following_,false);
	nh_private_.param<bool>("use_avoiding",use_avoiding_,false);
	nh_private_.param<bool>("is_offline_debug",is_offline_debug_,false);
	
	std::string odom_topic = nh_private_.param<std::string>("odom_topic","/ll2utm_odom");
	std::string path_points_file = nh_private_.param<std::string>("path_points_file","");
	
	initDiagnosticPublisher(nh_,__NAME__);

	if(!loadVehicleParams())
	{
		ROS_ERROR("[%s] Load vehicle parameters failed!", __NAME__);
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::ERROR,"Load vehicle parameters failed!");
		return false;
	}

	if(path_points_file.empty())
	{
		ROS_ERROR("[%s] No input path points file !!", __NAME__);
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::ERROR,"No input path file!");
		return false;
	}

	//载入路网文件
	if(! loadPathPoints(path_points_file, global_path_))
	{
	    ROS_ERROR("[%s] Load path file failed!",__NAME__);
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::ERROR,"Load path file failed!");
		return false;
	}
	if(!extendPath(global_path_, 20.0)) return false; //路径拓展延伸

	//载入路径信息
	std::string path_infos_file = path_points_file.substr(0,path_points_file.find_last_of(".")) + "_info.xml";
	if(!loadPathInfos(path_infos_file, global_path_, parking_points_, __NAME__))
	{
		ROS_ERROR("[%s] Load path infomation failed!",__NAME__);
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::ERROR,"Load path infomation failed!");
		return false;
	}

	//订阅公用传感器数据
	sub_odom_ = nh_.subscribe(odom_topic, 1,&AutoDrive::odom_callback,this);
	sub_vehicleState2_ = nh_.subscribe("/vehicleState2",1,&AutoDrive::vehicleSpeed_callback,this);
	sub_vehicleState4_ = nh_.subscribe("/vehicleState4",1,&AutoDrive::vehicleState4_callback,this);

	//发布
	pub_cmd1_ = nh_.advertise<ant_msgs::ControlCmd1>("/controlCmd1",1);
	pub_cmd2_ = nh_.advertise<ant_msgs::ControlCmd2>("/controlCmd2",1);
	pub_diagnostic_ = nh_.advertise<diagnostic_msgs::DiagnosticStatus>("driverless/diagnostic",1);

	//定时器
	cmd1_timer_ = nh_.createTimer(ros::Duration(0.02), &AutoDrive::sendCmd1_callback,this);
	cmd2_timer_ = nh_.createTimer(ros::Duration(0.01), &AutoDrive::sendCmd2_callback,this);
	
	//离线调试,无需系统检查
	if(is_offline_debug_)
		return true;
		
	// 车辆状态检查，等待初始化
	while(ros::ok())
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
	return true;
}

bool AutoDrive::loadVehicleParams()
{
	bool ok = true;
	vehicle_params_.max_roadwheel_angle = nh_private_.param<float>("vehicle/max_roadwheel_angle",0.0);
	vehicle_params_.max_speed = nh_private_.param<float>("vehicle/max_speed",0.0);
	vehicle_params_.wheel_base = nh_private_.param<float>("vehicle/wheel_base",0.0);
	vehicle_params_.wheel_track = nh_private_.param<float>("vehicle/wheel_track",0.0);
	vehicle_params_.width = nh_private_.param<float>("vehicle/width",0.0);
	vehicle_params_.length = nh_private_.param<float>("vehicle/length",0.0);
	std::string node = ros::this_node::getName();
	if(vehicle_params_.max_roadwheel_angle == 0.0)
	{
		ROS_ERROR("[%s] No parameter %s/vehicle/max_roadwheel_angle.",__NAME__,node);
		ok = false;
	}
	if(vehicle_params_.max_speed == 0.0)
	{
		ROS_ERROR("[%s] No parameter %s/vehicle/max_speed.",__NAME__,node);
		ok = false;
	}
	if(vehicle_params_.wheel_base == 0.0)
	{
		ROS_ERROR("[%s] No parameter %s/vehicle/wheel_base.",__NAME__,node);
		ok = false;
	}
	if(vehicle_params_.wheel_track == 0.0)
	{
		ROS_ERROR("[%s] No parameter %s/vehicle/wheel_track.",__NAME__,node);
		ok = false;
	}
	if(vehicle_params_.width == 0.0)
	{
		ROS_ERROR("[%s] No parameter %s/vehicle/width.",__NAME__,node);
		ok = false;
	}
	if(vehicle_params_.length == 0.0)
	{
		ROS_ERROR("[%s] No parameter %s/vehicle/length.",__NAME__,node);
		ok = false;
	}
	if(ok) vehicle_params_.validity = true;
	return ok;
}

void AutoDrive::run()
{
	//配置路径跟踪控制器
    if(!tracker_.init(nh_, nh_private_))
	{
		ROS_ERROR("[%s] path tracker init false!",__NAME__);
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::ERROR,"Init path tracker failed!");
		return;
	}
	tracker_.setExpectSpeed(max_speed_);
	tracker_.start();//路径跟踪控制器
	ROS_INFO("[%s] path tracker init ok",__NAME__);
	
	//配置跟车控制器
	if(!car_follower_.init(nh_, nh_private_))
	{
		ROS_ERROR("[%s] car follower init false!",__NAME__);
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::ERROR,"Init car follower failed!");
		return;
	}
	car_follower_.start(); //跟车控制器
	ROS_INFO("[%s] car follower init ok",__NAME__);
	
	//配置外部控制器
	extern_controler_.init(nh_, nh_private_);
	extern_controler_.start();

	ros::Rate loop_rate(20);
	
	while(ros::ok())
	{
		tracker_cmd_ = tracker_.getControlCmd();
		follower_cmd_= car_follower_.getControlCmd();
		extern_cmd_ = extern_controler_.getControlCmd();
		
		decisionMaking();
		loop_rate.sleep();
	}
	
	ROS_INFO("driverless completed..."); 
	tracker_.stop();
	car_follower_.stop();
	extern_controler_.stop();
}

void AutoDrive::decisionMaking()
{
	std::lock_guard<std::mutex> lock(command_mutex_);
//	showCmd(extern_cmd_,"extern_cmd");
//	showCmd(follower_cmd_,"follower_cmd");
//	showCmd(tracker_cmd_,"tracker_cmd");
	
	if(extern_cmd_.validity && extern_cmd_.speed < tracker_cmd_.speed)
		controlCmd2_.set_speed = extern_cmd_.speed;
	else if(follower_cmd_.validity && follower_cmd_.speed < tracker_cmd_.speed)
		controlCmd2_.set_speed = follower_cmd_.speed;
	else
		controlCmd2_.set_speed = tracker_cmd_.speed;
	
	controlCmd2_.set_roadWheelAngle = tracker_cmd_.roadWheelAngle;
	
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
}

void AutoDrive::sendCmd1_callback(const ros::TimerEvent&)
{
	pub_cmd1_.publish(controlCmd1_);
}

void AutoDrive::sendCmd2_callback(const ros::TimerEvent&)
{
	std::lock_guard<std::mutex> lock(command_mutex_);
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
	if(vehicle_speed_ >20.0)
		return;

	vehicle_state_.setSpeed(msg->vehicle_speed); //  m/s
	vehicle_state_.speed_validity = true;
}

void AutoDrive::vehicleState4_callback(const ant_msgs::State4::ConstPtr& msg)
{
	vehicle_state_.setSteerAngle(msg->roadwheelAngle);
	vehicle_state_.steer_validity = true;
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "auto_drive_node");
	ros::AsyncSpinner spinner(5);
	spinner.start(); //非阻塞

    AutoDrive auto_drive;
    if(auto_drive.init())
		auto_drive.run();
    //ros::waitForShutdown();
    return 0;
}  
