#ifndef AUTO_DRIVE_BASE_CPP
#define AUTO_DRIVE_BASE_CPP

#include <thread>
#include <mutex>
#include <ros/ros.h>
#include "structs.h"
#include <ant_math/ant_math.h>
#include <diagnostic_msgs/DiagnosticStatus.h>


/*@brief 自动驾驶子模块基类
 */
class AutoDriveBase
{
protected://子类可访问,实例不可访问
	std::vector<gpsMsg_t>  path_points_;//路径点
    std::vector<parkingPoint_t> parking_points_;//停车点索引
	size_t next_parking_index_; //下一个停车点索引
	float path_points_resolution_;

	std::vector<turnRange_t> turn_ranges_; //转向区间

	ros::Publisher pub_diagnostic_;
	diagnostic_msgs::DiagnosticStatus diagnostic_msg_;
	bool is_ready_;
	bool is_running_;
	bool is_initialed_;

	bool vehicle_speed_status_;
	float vehicle_speed_;
    gpsMsg_t vehicle_pose_;
	float roadwheel_angle_;

	vehicleParams_t vehicle_;

	std::mutex cmd_mutex_;
	controlCmd_t cmd_;

private:
	bool diagnostic_inited_;
	std::string child_name_;

public:
	AutoDriveBase() = delete;
	AutoDriveBase(const AutoDriveBase& ) = delete;

	explicit AutoDriveBase(const std::string& child_name)
	{
		is_ready_ = false;
		is_initialed_ = false;
		is_running_ = false;
		diagnostic_inited_ = false;
		vehicle_speed_status_ = false;
		child_name_ = child_name;
		cmd_.speed = cmd_.roadWheelAngle = 0.0;
		cmd_.validity = false;
                next_parking_index_ = 0;
	}
	virtual ~AutoDriveBase()
	{

	}

	virtual bool setVehicleParams(const vehicleParams_t& params)
	{
		if(false == params.validity)
		{
			ROS_ERROR("[%s] Vehicle parameters is invalid, please load them correctly.",child_name_.c_str());
			return false;
		}
		vehicle_ = params;
		return true;
	}

	/*@brief 设置全局路径
	*/
	virtual bool setGlobalPath(const std::vector<gpsMsg_t>& path, float resolution)
	{
		if(path_points_.size()!=0)
		{
			ROS_ERROR("[%s] global path points is not empty, set new points failed!",child_name_.c_str());
			return false;
		}
			
		path_points_ = path;
		path_points_resolution_ = resolution;
		return true;
	}

	/*@brief 设置停车点
	*/
	virtual bool setParkingPoints(const std::vector<parkingPoint_t>& points)
	{
		if(parking_points_.size()!=0)
		{
			ROS_ERROR("[%s] parking points is not empty, set new points failed!",child_name_.c_str());
			return false;
		}
		parking_points_ = points;
		return true;
	}

	/*@brief 设置转向区间信息
	*/
	virtual bool setTurnRanges(const std::vector<turnRange_t>& ranges)
	{
		if(turn_ranges_.size()!=0)
		{
			ROS_ERROR("[%s] turn ranges is not empty, set new ranges failed!",child_name_.c_str());
			return false;
		}
		turn_ranges_ = ranges;
	}

	/*@brief 获取控制指令
	*/
	virtual controlCmd_t getControlCmd()
	{
		std::lock_guard<std::mutex> lock(cmd_mutex_);
		return cmd_;
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

#endif
