#ifndef __EXTERN_CONTROL_H_
#define __EXTERN_CONTROL_H_
#include "driverless/structs.h"

/*自动驾驶外部控制器基类，子类继承该基类实现外部控制策略
 *为避免自动驾驶过程中出现意外情况，而设计的一种外部干预机制
*/

class ExternControlBase
{
public:
    ExternControlBase() = delete;
	ExternControlBase(const ExternControlBase& ) = delete;

	explicit ExternControlBase(const std::string& name)
	{
		is_initialed_ = false;
		is_running_ = false;
		name_ = name;
		cmd_.speed = cmd_.roadWheelAngle = 0.0;
		cmd_.validity = false;
	}
	virtual ~ExternControlBase()
	{

	}

	/*@brief 获取控制指令*/
	virtual controlCmd_t getControlCmd()
	{
		std::lock_guard<std::mutex> lock(cmd_mutex_);
		return cmd_;
	}

    virtual bool init(ros::NodeHandle nh,ros::NodeHandle nh_private) = 0;
	virtual bool start() {is_running_ = true;}
	virtual void stop() {is_running_ = false;}
	virtual bool isRunning() {return is_running_;}


protected: //子类可以访问
	std::mutex cmd_mutex_;
	controlCmd_t cmd_;

	bool is_running_;
	bool is_initialed_;
	std::string name_;

};



#endif