
#include "ros/ros.h"
#include "driverless/driverless.h"

AutoDrive::AutoDrive():
    nh_private_("~"),
    avoid_offset_(0.0),
    tracker_(nh_, nh_private_),
{
    diagnostic_msg_.hardware_id = "driverless";
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

bool AutoDrive::init()
{
    nh_private_.param<float>("max_speed",max_speed_,20.0);//km/h
    nh_private_.param<std::string>("path_points_file",path_points_file_,"");

	if(path_points_file_.empty())
	{
		ROS_ERROR("no input path points file !!");
		publishDiagnostics(diagnostic_msgs::DiagnosticStatus::ERROR,"No input path file!");
		return false;
	}
	if(!loadPathPoints(path_points_file_, path_points_))
	{
		publishDiagnostics(diagnostic_msgs::DiagnosticStatus::ERROR,"Path file error!");
		return false;
	}

	//订阅公用传感器数据
	sub_odom_ = nh_.subscribe("/ll2utm",1,&AutoDrive::odom_callback,this);
	sub_vehicleState2_ = nh.subscribe("/vehicleState2",1,&AutoDrive::vehicleSpeed_callback,this);
	sub_vehicleState4_ = nh.subscribe("/vehicleState4",1,&AutoDrive::vehicleState4_callback,this);

	// wait for system ok
	while(ros::ok() && !is_gps_data_valid(current_point_))
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

    pub_cmd_ = nh_.advertise<driverless_msgs::ControlCmd>("/cmd",1);
	pub_diagnostic_ = nh.advertise<diagnostic_msgs::DiagnosticStatus>("driverless/diagnostic",1);

	


	if(!loadPathPoints(path_points_file_, path_points_))
	{
		publishDiagnostics(diagnostic_msgs::DiagnosticStatus::ERROR,"Path file error!");
		return false;
	}
}


void AutoDrive::autoDriveThread(const fs::path& file, float speed)
{
	if(speed > max_speed_)
		speed = max_speed_;
		
	std::string file_name = fs::system_complete(file).string();
	if(status_ == CurveTracking)
	{
		path_.clear();
		loadPath(file_name, path_);
		ROS_INFO("CurveTracking__path points size:%lu",path_.points.size());
	}
	else if(status_ == VertexTracking)
	{
		path_t vertex_path;
		loadPath(file_name,vertex_path);
		generatePathByVertexes(vertex_path, path_, 0.1);
		ROS_INFO("VertexTracking__path points size:%lu",path_.points.size());
	}
	
	ROS_INFO("path points size:%lu",path_.points.size());
	
	//配置路径跟踪控制器
	this->tracker_.setPath(path_);
    if(!tracker_.init(vehicle_pose_))
        return;
	//配置避障控制器
	avoider_.setPath(path_);
	if(!avoider_.init())
		return;
	
	ros::Rate loop_rate(20);
	
	while(ros::ok())
	{
		if(status_ == TrackerSuspend)
			continue;
		else if(status_ == TrackerStop)
			break;
		bool update_state = tracker_.update(vehicle_speed_, roadwheel_angle_, vehicle_pose_, avoid_offset_);
        if(update_state == 0)
        {
            usleep(0.01);
            continue;
        }
        else if(update_state == 2)
            break;
        tracker_.getTrackingCmd(cmd_.set_speed, cmd_.set_roadWheelAngle);
	}
	callOtherService("auto_drive_complete");
	
	ROS_INFO("driverless completed..."); //send msg to screen ??????/

	this->status_ = TrackerStop;
	avoider_.shutDown();
	
}

void AutoDrive::timer_callback(const ros::TimerEvent&)
{
	if(this->status_ == TrackerIdle)
	{
		cmd_.driverless_mode = false;
	}
	else if(this->status_ == TrackerSuspend)
	{
		cmd_.driverless_mode = true;
		cmd_.set_brake = 1;
	}
	else if(this->status_ == TrackerStop)
	{
		static bool is_first = false;
		static ros::Time first_time;
		if(!is_first)
		{
			is_first = true;
			first_time = ros::Time::now();
		}
		else if((ros::Time::now() - first_time).toSec() > 10.0)
		{
			this->status_ = TrackerIdle;
			is_first = false;
		}
		cmd_.driverless_mode = true;
		cmd_.set_brake = 1;
	}
	else
	{
		cmd_.driverless_mode = true;
		cmd_.set_brake = 0;
	}
	pub_cmd_.publish(cmd_);
}

void AutoDrive::odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	vehicle_pose_.x = msg->pose.pose.position.x;
	vehicle_pose_.y = msg->pose.pose.position.y;
	vehicle_pose_.yaw = msg->pose.covariance[0];
	
	vehicle_pose_.longitude = msg->pose.covariance[1];
	vehicle_pose_.latitude = msg->pose.covariance[2];
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "auto_drive_node");
	ros::AsyncSpinner spinner(4);
	spinner.start(); //非阻塞

    AutoDrive auto_drive;
    if(auto_drive.init())
    	ros::waitForShutdown();
    return 0;
}  