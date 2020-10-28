#ifndef EXTERN_CONTROL_H_
#define EXTERN_CONTROL_H_

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <vector>
#include <thread>
#include <mutex>
#include "structs.h"
#include "utils.hpp"
#include "driverless/auto_drive_base.h"

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
	ExternCmdType_Speed = 0,   //速度指令
	ExternCmdType_Disable = 1, //禁用外部指令
	ExternCmdType_turnLight=2, //转向灯指令
	
};

PACK(
typedef struct ExternCmd
{
	uint8_t header[2];
	uint8_t type; 
	uint8_t len;
	uint8_t data[];
	
}) externCmd_t;


class ExternControl : public AutoDriveBase
{
public:
	ExternControl();
	~ExternControl(){};
	
	virtual bool init(ros::NodeHandle nh,ros::NodeHandle nh_private) override;
	bool start();
	void stop();
	bool isRunning();

private:
	bool initSocket();
	void externControlThread();
	
private:
	ros::Timer      update_timer_;
	bool       is_ready_; //是否准备就绪
	bool       is_running_;
	
	int udp_fd_;
	struct sockaddr_in sockaddr_;
	std::string local_ip_;
	int         local_port_;
	
};

#endif





