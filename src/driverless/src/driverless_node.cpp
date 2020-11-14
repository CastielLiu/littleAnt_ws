
#include "ros/ros.h"
#include "driverless/driverless_node.h"

#define __NAME__ "driverless"

void AutoDrive::workingThread()
{
	ros::Rate loop_rate1(10);
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
		//ROS_INFO("[%s] Current system_state: %d", __NAME__, state);
		
		if(state == State_Drive)
			doDriveWork();
		else if(state == State_Reverse)
			doReverseWork();
		else
			ROS_ERROR("[%s] Unknown task type in current state: %d.", __NAME__, state);
		
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
				//在执行当前目标的同时，判断是否有新目标到达
				if(as_->isNewGoalAvailable())
				{
					//if we're active and a new goal is available, we'll accept it, but we won't shut anything down
					driverless::DoDriverlessTaskGoalConstPtr new_goal = as_->acceptNewGoal();
					handleNewGoal(new_goal);
					break;
				}
				else
				{
					as_->setPreempted(); //自主触发中断请求
					break;
				}
			}
		}

		loop_rate.sleep();
	}
	switchSystemState(State_Stop);
	ROS_INFO("[%s] drive work  completed...", __NAME__); 
	tracker_.stop();
	car_follower_.stop();
	if(as_->isActive())
	{
		as_->setSucceeded(driverless::DoDriverlessTaskResult(), "drive work  completed");
	}
	
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
		{
			reverseDecisionMaking();
		}

		if(as_->isActive())
		{
			driverless::DoDriverlessTaskFeedback feedback;
			feedback.speed = reverse_cmd_.speed;
			feedback.steer_angle = reverse_cmd_.roadWheelAngle;
			as_->publishFeedback(feedback);
			
			if(as_->isPreemptRequested()) 
			{
				ROS_INFO("[%s] isPreemptRequested.", __NAME__);
				//在执行当前目标的同时，判断是否有新目标到达
				if(as_->isNewGoalAvailable())
				{
					//if we're active and a new goal is available, we'll accept it, but we won't shut anything down
					driverless::DoDriverlessTaskGoalConstPtr new_goal = as_->acceptNewGoal();
					handleNewGoal(new_goal);
					break;
				}
				else
				{
					as_->setPreempted(); //自主触发中断请求
					break;
				}
			}
		}

		loop_rate.sleep();
	}
	if(as_->isActive())
	{
		
	}
	switchSystemState(State_Stop);
	reverse_controler_.stop();
	ROS_INFO("[%s] reverse work complete.", __NAME__);
	if(as_->isActive())
	{
		as_->setSucceeded(driverless::DoDriverlessTaskResult(), "drive work  completed");
	}
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


	
