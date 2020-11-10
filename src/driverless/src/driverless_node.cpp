
#include "ros/ros.h"
#include "driverless/driverless_node.h"

#define __NAME__ "driverless"

void AutoDrive::workingThread()
{
	ros::Rate loop_rate1(10);
	is_running_ = true;
	while(ros::ok() && is_running_)
	{
		if(!has_new_task_)
		{
			loop_rate1.sleep();
			continue;
		}
		has_new_task_ = false;

		if(system_state_ == State_Drive)
			doDriveWork();
		else if(system_state_ == State_Reverse)
			doReverseWork();
		
		switchSystemState(State_Stop);
	}
}

void AutoDrive::doDriveWork()
{
	//配置路径跟踪控制器
	tracker_.setExpectSpeed(expect_speed_);
	tracker_.start();//路径跟踪控制器
	//配置跟车控制器
	car_follower_.start(); //跟车控制器
	//配置外部控制器
	extern_controler_.start();

	ros::Rate loop_rate(20);
	
	while(ros::ok() && system_state_ == State_Drive)
	{
		tracker_cmd_ = tracker_.getControlCmd();
		follower_cmd_= car_follower_.getControlCmd();
		extern_cmd_ = extern_controler_.getControlCmd();
		
		auto cmd = this->decisionMaking();

		if(as_->isActive())
		{
			driverless::DoDriverlessTaskFeedback feedback;
			feedback.speed = cmd.set_speed;
			feedback.steer_angle = cmd.set_roadWheelAngle;
			as_->publishFeedback(feedback);
		}

		loop_rate.sleep();
	}

	ROS_INFO("[%s] driverless completed...", __NAME__); 
	tracker_.stop();
	car_follower_.stop();
	extern_controler_.stop();
}

void AutoDrive::doReverseWork()
{
	reverse_controler_.start();
	
	ros::Rate loop_rate(20);
	while(ros::ok() && system_state_ == State_Reverse && reverse_controler_.isRunning())
	{
		ROS_INFO("[%s] new cycle.", __NAME__);
		reverse_cmd_ = reverse_controler_.getControlCmd();
		
		ROS_INFO("[%s] speed: %.2f\t angle: %.2f", __NAME__, reverse_cmd_.speed, reverse_cmd_.roadWheelAngle);
		
		if(reverse_cmd_.validity)
		{
			std::lock_guard<std::mutex> lock2(cmd2_mutex_);
			controlCmd2_.set_speed = reverse_cmd_.speed;
			controlCmd2_.set_roadWheelAngle = reverse_cmd_.roadWheelAngle;
		}

		if(as_->isActive())
		{
			driverless::DoDriverlessTaskFeedback feedback;
			feedback.speed = reverse_cmd_.speed;
			feedback.steer_angle = reverse_cmd_.roadWheelAngle;
			as_->publishFeedback(feedback);
		}

		loop_rate.sleep();
	}
	reverse_controler_.stop();
	ROS_INFO("[%s] reverse work complete.", __NAME__);
}

ant_msgs::ControlCmd2 AutoDrive::decisionMaking()
{
	std::lock_guard<std::mutex> lock2(cmd2_mutex_);
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


	