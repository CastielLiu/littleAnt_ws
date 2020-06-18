
#include "ros/ros.h"
#include "driverless/driverless.h"

AutoDrive::AutoDrive():
    nh_private_("~"),
    avoid_offset_(0.0)
{
	diagnostic_msg_.hardware_id = "driverless";
	vehicle_speed_status_ = false;
	controlCmd1_.set_driverlessMode = true;
	controlCmd1_.set_handBrake = false;
	controlCmd2_.set_gear = 1;
	controlCmd2_.set_speed = 0.0;
	controlCmd2_.set_brake = 0.0;
	controlCmd2_.set_roadWheelAngle = 0.0;
}

AutoDrive::~AutoDrive()
{

}

void AutoDrive::publishDiagnostics(uint8_t level,const std::string& msg)
{
	diagnostic_msg_.level = level;
	diagnostic_msg_.message = msg;
	pub_diagnostic_.publish(diagnostic_msg_);
}

bool AutoDrive::is_gps_data_valid(const gpsMsg_t& point)
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
	nh_private_.param<std::string>("path_points_file",path_points_file_,"");
	nh_private_.param<bool>("use_car_following",use_car_following_,false);
	nh_private_.param<bool>("use_avoiding",use_avoiding_,false);
	nh_private_.param<bool>("is_offline_debug",is_offline_debug_,false);
	
	std::string odom_topic = nh_private_.param<std::string>("odom_topic","/ll2utm_odom");
	

	if(path_points_file_.empty())
	{
		ROS_ERROR("no input path points file !!");
		publishDiagnostics(diagnostic_msgs::DiagnosticStatus::ERROR,"No input path file!");
		return false;
	}

	//载入路网文件
	if(!loadPathPoints(path_points_file_, path_points_))
	{
		publishDiagnostics(diagnostic_msgs::DiagnosticStatus::ERROR,"Path file error!");
		return false;
	}

	//订阅公用传感器数据
	sub_odom_ = nh_.subscribe(odom_topic, 1,&AutoDrive::odom_callback,this);
	sub_vehicleState2_ = nh_.subscribe("/vehicleState2",1,&AutoDrive::vehicleSpeed_callback,this);
	sub_vehicleState4_ = nh_.subscribe("/vehicleState4",1,&AutoDrive::vehicleState4_callback,this);

	//发布
	pub_cmd1_ = nh_.advertise<little_ant_msgs::ControlCmd1>("/controlCmd1",1);
	pub_cmd2_ = nh_.advertise<little_ant_msgs::ControlCmd2>("/controlCmd2",1);
	pub_diagnostic_ = nh_.advertise<diagnostic_msgs::DiagnosticStatus>("driverless/diagnostic",1);

	//定时器
	cmd1_timer_ = nh_.createTimer(ros::Duration(0.02), &AutoDrive::sendCmd1_callback,this);
	cmd2_timer_ = nh_.createTimer(ros::Duration(0.01), &AutoDrive::sendCmd2_callback,this);
	
	//离线调试,无需系统检查
	if(is_offline_debug_) 
		return true;
		
	// 系统状态检查，等待初始化
	while(ros::ok() && !is_gps_data_valid(vehicle_pose_))
	{
		ROS_INFO("gps data is invalid, please check the gps topic or waiting...");
		publishDiagnostics(diagnostic_msgs::DiagnosticStatus::WARN,"gps data is invalid");
		sleep(1);
	}
	while(!vehicle_speed_status_ && ros::ok())
	{
		ROS_INFO("waiting for vehicle speed data ok ...");
		publishDiagnostics(diagnostic_msgs::DiagnosticStatus::WARN,"Vehicle speed is invalid");
		usleep(500000);
	}
	return true;
}

void AutoDrive::run()
{
	//配置路径跟踪控制器
	tracker_.setExpectSpeed(max_speed_);
	tracker_.setGlobalPath(path_points_);
    if(!tracker_.init(nh_, nh_private_))
	{
		publishDiagnostics(diagnostic_msgs::DiagnosticStatus::ERROR,"Init path tracker failed!");
		return;
	}
	//tracker_.start();//路径跟踪控制器
	
	//配置跟车控制器
	car_follower_.setGlobalPath(path_points_);
	if(!car_follower_.init(nh_, nh_private_))
	{
		publishDiagnostics(diagnostic_msgs::DiagnosticStatus::ERROR,"Init car follower failed!");
		return;
	}
	car_follower_.start(); //跟车控制器
	
	//配置外部控制器
	extern_controler_.init(nh_, nh_private_);
	extern_controler_.start();

	ros::Rate loop_rate(20);
	
	while(ros::ok())
	{
		tracker_.updateStatus(vehicle_pose_, vehicle_speed_, roadwheel_angle_);
		size_t nearest_point_index = tracker_.getNearestPointIndex();
		car_follower_.updateStatus(vehicle_pose_,vehicle_speed_,nearest_point_index);
		extern_controler_.updateStatus(vehicle_pose_,vehicle_speed_);

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
	vehicle_pose_.x = msg->pose.pose.position.x;
	vehicle_pose_.y = msg->pose.pose.position.y;
	vehicle_pose_.yaw = msg->pose.covariance[0];
	
	//vehicle_pose_.longitude = msg->pose.covariance[1];
	//vehicle_pose_.latitude = msg->pose.covariance[2];
}

void AutoDrive::vehicleSpeed_callback(const little_ant_msgs::State2::ConstPtr& msg)
{
	if(vehicle_speed_ >20.0)
		return;
	vehicle_speed_status_ = true;
	vehicle_speed_ = msg->vehicle_speed; //  m/s
}

void AutoDrive::vehicleState4_callback(const little_ant_msgs::State4::ConstPtr& msg)
{
	roadwheel_angle_ = msg->roadwheelAngle;
}



int main(int argc, char *argv[])
{
    ros::init(argc, argv, "auto_drive_node");
	ros::AsyncSpinner spinner(4);
	spinner.start(); //非阻塞

    AutoDrive auto_drive;
    if(auto_drive.init())
		auto_drive.run();
    //ros::waitForShutdown();
    return 0;
}  
