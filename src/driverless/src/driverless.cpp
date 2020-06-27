
#include "ros/ros.h"
#include "driverless/driverless.h"

#define __NAME__ "driverless"

AutoDrive::AutoDrive():
	AutoDriveBase(__NAME__),
    nh_private_("~"),
    avoid_offset_(0.0)
{
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
		ROS_ERROR("no input path points file !!");
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::ERROR,"No input path file!");
		return false;
	}

	//载入路网文件
	path_points_resolution_ = loadPathPoints(path_points_file, path_points_);
	if(path_points_resolution_ == 0.0)
	{
	    ROS_ERROR("[%s] Load path file failed!",__NAME__);
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::ERROR,"Load path file failed!");
		return false;
	}

	//载入路径信息
	std::string path_infos_file = path_points_file.substr(0,path_points_file.find_last_of(".")) + "_info.xml";
	if(!loadPathInfos(path_infos_file))
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
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::WARN,"gps data is invalid");
		sleep(1);
	}
	while(!vehicle_speed_status_ && ros::ok())
	{
		ROS_INFO("waiting for vehicle speed data ok ...");
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::WARN,"Vehicle speed is invalid");
		usleep(500000);
	}
	return true;
}

bool AutoDrive::loadVehicleParams()
{
	bool ok = true;
	vehicle_.max_roadwheel_angle = nh_private_.param<float>("vehicle/max_roadwheel_angle",0.0);
	vehicle_.max_speed = nh_private_.param<float>("vehicle/max_speed",0.0);
	vehicle_.wheel_base = nh_private_.param<float>("vehicle/wheel_base",0.0);
	vehicle_.wheel_track = nh_private_.param<float>("vehicle/wheel_track",0.0);
	vehicle_.width = nh_private_.param<float>("vehicle/width",0.0);
	vehicle_.length = nh_private_.param<float>("vehicle/length",0.0);
	std::string node = ros::this_node::getName();
	if(vehicle_.max_roadwheel_angle == 0.0)
	{
		ROS_ERROR("[%s] No parameter %s/vehicle/max_roadwheel_angle.",__NAME__,node);
		ok = false;
	}
	if(vehicle_.max_speed == 0.0)
	{
		ROS_ERROR("[%s] No parameter %s/vehicle/max_speed.",__NAME__,node);
		ok = false;
	}
	if(vehicle_.wheel_base == 0.0)
	{
		ROS_ERROR("[%s] No parameter %s/vehicle/wheel_base.",__NAME__,node);
		ok = false;
	}
	if(vehicle_.wheel_track == 0.0)
	{
		ROS_ERROR("[%s] No parameter %s/vehicle/wheel_track.",__NAME__,node);
		ok = false;
	}
	if(vehicle_.width == 0.0)
	{
		ROS_ERROR("[%s] No parameter %s/vehicle/width.",__NAME__,node);
		ok = false;
	}
	if(vehicle_.length == 0.0)
	{
		ROS_ERROR("[%s] No parameter %s/vehicle/length.",__NAME__,node);
		ok = false;
	}
	if(ok) vehicle_.validity = true;
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
	if(!tracker_.setVehicleParams(vehicle_)) return;
	tracker_.setExpectSpeed(max_speed_);
	tracker_.setGlobalPath(path_points_,path_points_resolution_);
	tracker_.setParkingPoints(parking_points_);
	tracker_.start();//路径跟踪控制器
	ROS_INFO("[%s] path tracker init ok",__NAME__);
	
	//配置跟车控制器
	if(!car_follower_.init(nh_, nh_private_))
	{
		ROS_ERROR("[%s] car follower init false!",__NAME__);
		publishDiagnosticMsg(diagnostic_msgs::DiagnosticStatus::ERROR,"Init car follower failed!");
		return;
	}
	if(!car_follower_.setVehicleParams(vehicle_)) return;
	car_follower_.setGlobalPath(path_points_,path_points_resolution_);
	car_follower_.setParkingPoints(parking_points_);
	car_follower_.start(); //跟车控制器
	ROS_INFO("[%s] car follower init ok",__NAME__);
	
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
	if(!vehicle_speed_status_)
		vehicle_speed_status_ = true;
	vehicle_speed_ = msg->vehicle_speed; //  m/s
}

void AutoDrive::vehicleState4_callback(const little_ant_msgs::State4::ConstPtr& msg)
{
	roadwheel_angle_ = msg->roadwheelAngle;
}

/*@brief 从xml文件载入路径信息
 *@1. 停车点-若文件不包含终点信息，手动添加√
 *@2. 转向区间-控制转向灯　
 *@3. 
*/
#include <tinyxml2.h>
bool AutoDrive::loadPathInfos(const std::string& file)
{
	if(path_points_.size() == 0)
	{
		ROS_ERROR("[%s] please load Path Points first!",__NAME__);
		return false;
	}
	
	using namespace tinyxml2;
	XMLDocument Doc;  
	XMLError res = Doc.LoadFile(file.c_str());
	
	if(XML_ERROR_FILE_NOT_FOUND == res)
	{
		ROS_ERROR_STREAM("[" << __NAME__ <<"] " << "path infomation file: "<< file << " not exist!");
		return false;
	}
	else if(XML_SUCCESS != res)
	{
		ROS_ERROR_STREAM("[" << __NAME__ <<"] " << "path infomation file: "<< file << " parse error!");
		return false;
	}
	tinyxml2::XMLElement *pRoot=Doc.RootElement();//根节点
	if(pRoot == nullptr)
	{
		ROS_ERROR_STREAM("[" << __NAME__ <<"] " << "path infomation file: "<< file << " no root node!");
		return false;
	}
	tinyxml2::XMLElement *pParkingPoints = pRoot->FirstChildElement("ParkingPoints"); //一级子节点
	if(pParkingPoints)
	{
		bool has_dst_parking_point = false;//是否有终点
		tinyxml2::XMLElement *pParkingPoint = pParkingPoints->FirstChildElement("ParkingPoint"); //二级子节点
		while (pParkingPoint)
		{
			uint32_t id    = pParkingPoint->Unsigned64Attribute("id");
			uint32_t index = pParkingPoint->Unsigned64Attribute("index");
			float duration = pParkingPoint->FloatAttribute("duration");
			parking_points_.push_back(parkingPoint_t(index,duration));
			//std::cout << id << "\t" << index << "\t" << duration << std::endl;
			if(duration == 0)
				has_dst_parking_point = true;
			//转到下一子节点
			pParkingPoint = pParkingPoint->NextSiblingElement("ParkingPoint");  
		}

		//如果路径信息中不包含终点停车点，手动添加路径终点为停车点
		if(!has_dst_parking_point)
			parking_points_.push_back(parkingPoint_t(path_points_.size()-1,0.0)); 
		
		//停车点从小到达排序
		std::sort(parking_points_.begin(),parking_points_.end(),
			[](const parkingPoint_t&point1,const parkingPoint_t&point2)
			{return point1.index < point2.index;});
			
		for(auto &point:parking_points_)
			ROS_INFO("[%s] parking point index: %d  duration: %.1f",__NAME__,point.index,point.parkingDuration);
		
		ROS_INFO("[%s] load Parking Points ok.",__NAME__);
	}
	else
		ROS_INFO("[%s] No Parking Points in path info file!",__NAME__);

	tinyxml2::XMLElement *pTurnRanges = pRoot->FirstChildElement("TurnRanges"); //一级子节点
	if(pTurnRanges)
	{
		tinyxml2::XMLElement *pTurnRange = pTurnRanges->FirstChildElement("TurnRange"); //二级子节点
		while (pTurnRange)
		{
			int    type  = pTurnRange->IntAttribute("type");
			size_t start = pParkingPoint->Unsigned64Attribute("start");
			size_t end   = pParkingPoint->Unsigned64Attribute("end");
			turn_ranges_.push_back(turnRange_t(type,start,end));
			//std::cout << type << "\t" << start << "\t" << end << std::endl;
			
			//转到下一子节点
			pTurnRange = pTurnRange->NextSiblingElement("TurnRange"); 
		}
		for(auto &range : turn_ranges_)
			ROS_INFO("[%s] turn range: type:%d  start:%d  end:%d",__NAME__,range.type,range.start,range.end);
		
		ROS_INFO("[%s] load turn ranges ok.",__NAME__);
	}
	else
		ROS_INFO("[%s] No tutn ranges in path info file!",__NAME__);


	return true;
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
