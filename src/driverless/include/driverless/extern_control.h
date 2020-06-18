#ifndef EXTERN_CONTROL_H_
#define EXTERN_CONTROL_H_

#include <ros/ros.h>
#include <ant_math/ant_math.h>
#include <std_msgs/UInt8.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <vector>
#include <thread>
#include <mutex>
#include "structs.h"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define EXTERN_CONTORL_CMD_HEADER1 0x66
#define EXTERN_CONTORL_CMD_HEADER2 0xcc

#ifndef PACK
#define PACK( __Declaration__ ) __Declaration__ __attribute__((__packed__))
#endif

enum ExternCmdType
{
	ExternCmdType_Speed = 0,
	ExternCmdType_Disable = 1, //禁用外部指令
	ExternCmdType_turnLight=2, //转向灯
	
};

PACK(
typedef struct ExternCmd
{
	uint8_t header[2];
	uint8_t type; 
	uint8_t len;
	uint8_t data[];
	
}) externCmd_t;


class ExternControl
{
public:
	ExternControl();
	~ExternControl(){};
	
	bool setGlobalPath(const std::vector<gpsMsg_t>& path);
	bool updateStatus(const gpsMsg_t& pose,const float& speed);
	bool init(ros::NodeHandle nh,ros::NodeHandle nh_private);
	bool start();
	void stop();
	bool isRunning();
	controlCmd_t getControlCmd();

private:
	bool initSocket();
	void externControlThread();
	
private:
	ros::NodeHandle nh_;
	ros::NodeHandle nh_private_;
	ros::Publisher  pub_diagnostic_;

	ros::Timer      update_timer_;

	std::vector<gpsMsg_t> path_points_;
	float path_points_resolution_;

	std::mutex cmd_mutex_;
	controlCmd_t cmd_;

	//state
	std::mutex state_mutex_;
	float      vehicle_speed_;
	float      roadwheel_angle_;
	bool       is_ready_; //是否准备就绪
	bool       is_running_;

	diagnostic_msgs::DiagnosticStatus diagnostic_msg_;
	
	int udp_fd_;
	struct sockaddr_in sockaddr_;
	std::string local_ip_;
	int         local_port_;
	
};

#endif





