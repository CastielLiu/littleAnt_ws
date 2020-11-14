#include "driverless/extern_control/extern_control.h"

#define __NAME__ "extern_control"

ExternControl::ExternControl():
	AutoDriveBase(__NAME__),
	controller_(nullptr)
{
	
}

ExternControl::~ExternControl()
{
	if(controller_!=nullptr)
	{
		delete controller_;
		controller_ = nullptr;
	}
}

bool ExternControl::init(ros::NodeHandle nh,ros::NodeHandle nh_private)
{
	std::string which = nh_private.param<std::string>("which_extern_controller", "");
	if(which == "lan_controller")
		controller_ = new LanExternControl();
	else if(which == "wan_controller")
		controller_ = new WanExternControl();
	else
	{
		ROS_ERROR("[%s] Unknown extern controller: %s!", __NAME__, which.c_str());
		return false;
	}
	is_initialed_ = controller_->init(nh, nh_private);
	return is_initialed_;
}


bool ExternControl::start()
{
	if(!is_initialed_)
	{
		ROS_ERROR("[%s] Please init extern controller before start!", __NAME__);
		return false;
	}
	return controller_->start();
}

void ExternControl::stop()
{
	if(is_initialed_)
		controller_->stop();
}

bool ExternControl::isRunning()
{
	return controller_->isRunning();
}

controlCmd_t ExternControl::getControlCmd() 
{
	return controller_->getControlCmd();
}