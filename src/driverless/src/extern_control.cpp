#include "driverless/extern_control.h"

#define __NAME__ "extern_control"

ExternControl::ExternControl():
	AutoDriveBase(__NAME__)
{
	cmd_.validity = false;
	is_running_ = false;
	is_ready_ = false;
}

bool ExternControl::init(ros::NodeHandle nh,ros::NodeHandle nh_private)
{
	nh_ = nh;
	nh_private_ = nh_private;
	local_ip_     = nh_private.param<std::string>("local_ip","0.0.0.0");
	local_port_   = nh_private.param<int>("local_port", 5000);

	pub_diagnostic_ = nh.advertise<diagnostic_msgs::DiagnosticStatus>("driverless/diagnostic",1);
	return initSocket();
}

bool ExternControl::isRunning()
{
	return is_running_;
}

bool ExternControl::initSocket()
{
	struct sockaddr_in local_addr;
	bzero(&local_addr,sizeof(local_addr));//init 0

	local_addr.sin_port = htons(local_port_);
	local_addr.sin_family = AF_INET;
	int convert_ret = inet_pton(AF_INET, local_ip_.c_str(), &local_addr.sin_addr);
	if(convert_ret !=1)
	{
		ROS_ERROR("convert socket ip failed, please check the format!");
		return false;
	}
	else
		ROS_INFO("convert socket ip complete .");
	
	//UDP
	udp_fd_ = socket(PF_INET,SOCK_DGRAM , 0);
	if(udp_fd_ < 0)
	{
		ROS_ERROR("build socket error");
		return false;
	}
	else
		ROS_INFO("build socket ok .");
	
	//设置端口复用
	int udo_opt = 1;
	setsockopt(udp_fd_, SOL_SOCKET, SO_REUSEADDR, &udo_opt, sizeof(udo_opt));
	
	// 设置超时
//	struct timeval timeout;
//	timeout.tv_sec = 0;  timeout.tv_usec = 0;
//	setsockopt(udp_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
	
	int ret = bind(udp_fd_, (struct sockaddr*)&local_addr, sizeof(local_addr));
	if(ret < 0)
	{
		ROS_ERROR("udp bind ip: %s port: %d failed!",local_ip_.c_str(),local_port_);
		return false;
	}
	
	return true;
}


//启动外部控制线程
bool ExternControl::start()
{
	is_running_ = true;
	std::thread t(&ExternControl::externControlThread,this);
	t.detach();
	return true;
}

void ExternControl::externControlThread()
{
	const int BufLen = 100;
	uint8_t *recvbuf = new uint8_t [BufLen+1];
	struct sockaddr_in client_addr;
	socklen_t clientLen = sizeof(client_addr);
	int len = 0;

	while(is_running_)
	{
		len = recvfrom(udp_fd_, recvbuf, BufLen,0,(struct sockaddr*)&client_addr, &clientLen);
		if(len <=0) continue;
		
		for(int i=0; i<len; ++i)
			std::cout << std::hex << int(recvbuf[i]) << "\t";
		std::cout << std::endl;
		
		if(recvbuf[0] != EXTERN_CONTORL_CMD_HEADER1 ||
		   recvbuf[1] != EXTERN_CONTORL_CMD_HEADER2 )
		   continue;
		   
		std::string answer;
		externCmd_t *extern_cmd = (externCmd_t*)recvbuf;
		if(extern_cmd->type == ExternCmdType_Speed)
		{
			cmd_mutex_.lock();
			cmd_.validity = true;
			cmd_.speed = extern_cmd->data[0];
			cmd_mutex_.unlock();
			answer = std::string("speed:") + std::to_string(extern_cmd->data[0]) + std::string("km/h");
		}
		else if(extern_cmd->type == ExternCmdType_Disable)
		{
			cmd_mutex_.lock();
			cmd_.validity = false;
			cmd_mutex_.unlock();
			answer = "disabled";
		}
		else if(extern_cmd->type == ExternCmdType_turnLight)
		{
			cmd_mutex_.lock();
			cmd_.turnLight = extern_cmd->data[0];
			cmd_mutex_.unlock();
			if(extern_cmd->data[0] == 0)
				answer = "close all light";
			else if(extern_cmd->data[0] == 1)
				answer = std::string("left light");
			else if(extern_cmd->data[0] == 2)
				answer = std::string("right light");
		}
		
		sendto(udp_fd_, answer.c_str(), answer.length(),0, 
						 (struct sockaddr*)&client_addr, sizeof(client_addr));
	}
	is_running_ = false;
}

void ExternControl::stop()
{
	is_running_ = false;
	
	cmd_mutex_.lock();
	cmd_.validity = false;
	cmd_mutex_.unlock();
}