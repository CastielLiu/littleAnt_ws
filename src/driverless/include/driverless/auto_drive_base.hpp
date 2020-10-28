#ifndef AUTO_DRIVE_BASE_CPP
#define AUTO_DRIVE_BASE_CPP

#include <thread>
#include <mutex>
#include <ros/ros.h>
#include "structs.h"
#include "utils.hpp"
#include <diagnostic_msgs/DiagnosticStatus.h>



/*@brief 自动驾驶子模块基类
 */
class AutoDriveBase
{
/*params*/
protected://子类可访问,实例不可访问
	static Path global_path_ ;            //全局路径
	static Path local_path_;              //局部路径
	static VehicleState vehicle_state_;   //汽车状态
	static VehicleParams vehicle_params_;

	std::mutex cmd_mutex_;
	controlCmd_t cmd_;

	ros::Publisher pub_diagnostic_;
	diagnostic_msgs::DiagnosticStatus diagnostic_msg_;
	bool is_ready_;
	bool is_running_;
	bool is_initialed_;

private:
	bool diagnostic_inited_;
	std::string child_name_;

/*methods*/
public:
	AutoDriveBase() = delete;
	AutoDriveBase(const AutoDriveBase& ) = delete;

	explicit AutoDriveBase(const std::string& child_name)
	{
		is_ready_ = false;
		is_initialed_ = false;
		is_running_ = false;
		diagnostic_inited_ = false;
		child_name_ = child_name;
		cmd_.speed = cmd_.roadWheelAngle = 0.0;
		cmd_.validity = false;
	}
	virtual ~AutoDriveBase()
	{

	}

	/*@brief 获取控制指令
	*/
	virtual controlCmd_t getControlCmd()
	{
		std::lock_guard<std::mutex> lock(cmd_mutex_);
		return cmd_;
	}

	virtual controlCmd_t setControlCmd(const controlCmd_t& cmd)
	{
		std::lock_guard<std::mutex> lock(cmd_mutex_);
		cmd_ = cmd;
	}


	virtual void showCmd(const std::string& name)
	{
		std::lock_guard<std::mutex> lock(cmd_mutex_);
		std::cout << name << "\t" << cmd_.validity << "\t" << cmd_.speed <<
				"\t" << cmd_.roadWheelAngle << std::endl; 
	}

	//virtual bool init();
	virtual bool start() {is_running_ = true;}
	virtual void stop() {is_running_=false;}
	virtual bool isRunning() {return is_running_;}

protected:
	/*@brief 初始化故障诊断发布器
	*/
	virtual void initDiagnosticPublisher(ros::NodeHandle& nh,const std::string& module_id)
	{
		diagnostic_msg_.hardware_id = module_id;
		pub_diagnostic_ = nh.advertise<diagnostic_msgs::DiagnosticStatus>("driverless/diagnostic",1);
		diagnostic_inited_ = true;
	}
	/*@brief 发布故障诊断消息
	*/
	virtual void publishDiagnosticMsg(uint8_t level,const std::string& msg)
	{
		if(!diagnostic_inited_)
		{
			ROS_ERROR("[%s] please initial diagnostic publisher before use it!",child_name_.c_str());
			return;
		}
		diagnostic_msg_.level = level;
		diagnostic_msg_.message = msg;
		pub_diagnostic_.publish(diagnostic_msg_);
	}
};

Path AutoDriveBase::global_path_ ;            //全局路径
Path AutoDriveBase::local_path_;              //局部路径
VehicleState AutoDriveBase::vehicle_state_;   //汽车状态
VehicleParams AutoDriveBase::vehicle_params_; //汽车参数

#endif
