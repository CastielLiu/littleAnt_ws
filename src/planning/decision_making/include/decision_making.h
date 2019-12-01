#ifndef DECISION_MAKING_H_
#define DECISION_MAKING_H_
#include<ros/ros.h>
#include<little_ant_msgs/ControlCmd.h>
#include<little_ant_msgs/ControlCmd1.h>
#include<little_ant_msgs/ControlCmd2.h>
#include<little_ant_msgs/DetectedObjectArray.h>
#include<std_msgs/Float32.h>
#include<ant_math/ant_math.h>

#define SENSOR_NUM little_ant_msgs::ControlCmd::SENSOR_NUM

#define _LIDAR        little_ant_msgs::ControlCmd::_LIDAR

#define _ESR_RADAR    little_ant_msgs::ControlCmd::_ESR_RADAR

#define _GPS          little_ant_msgs::ControlCmd::_GPS


class DecisionMaking
{
public:
	DecisionMaking();
	~DecisionMaking();
	
	bool init(ros::NodeHandle nh,ros::NodeHandle nh_private);

private:
	void sensor_decision_callback(const little_ant_msgs::ControlCmd::ConstPtr& msg);

	void sendCmd1_callback(const ros::TimerEvent&);

	void sendCmd2_callback(const ros::TimerEvent&);
	
	void updateCmdStatus_callback(const ros::TimerEvent&);
	
	void maxTolerateSpeed_callback(const std_msgs::Float32::ConstPtr& msg);

private:
	ros::Subscriber sub_sensors_decision_;
	ros::Subscriber sub_traffic_mark_;
	ros::Subscriber sub_max_tolerate_speed_;
	
	ros::Publisher pub_final_decision1_;
	ros::Publisher pub_final_decision2_;
	
	ros::Timer sendCmd1Timer_20ms_;
	ros::Timer sendCmd2Timer_10ms_;
	ros::Timer updateCmdStatus_300ms_;
	
	std::string sensors_decision_topic_;
	std::string final_decision_topic1_;
	std::string final_decision_topic2_;
	
	little_ant_msgs::ControlCmd1 cmd1_;
	little_ant_msgs::ControlCmd2 cmd2_;
	
	struct cmd_msg_t
	{
		bool status;
		double time;
		little_ant_msgs::ControlCmd cmd;
	}cmdMsg_[SENSOR_NUM];
	 
	float max_tolerate_speed_;
	
};






#endif
