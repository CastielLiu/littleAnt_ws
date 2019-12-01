#ifndef ESR_RADAR_H_
#define ESR_RADAR_H_
#include "can2serial/can2serial.h"
#include <ros/ros.h>
#include <string>
#include<little_ant_msgs/State2.h>
#include <esr_radar_msgs/Objects.h>
#include <esr_radar_msgs/Object.h>
#include<jsk_recognition_msgs/BoundingBox.h>
#include<jsk_recognition_msgs/BoundingBoxArray.h>
#include<cmath>
 
#define RECEIVE__ 0
#define SEND__  1


class ESR_RADAR 
{
	public:
		 ESR_RADAR(int argc,char ** argv);
		~ESR_RADAR();
		
		bool init();
		
		void run();
		void handleCanMsg();
		void start_publishBoundingBoxArray_thread();
		
		
	private:
		Can2serial *in_can2serial;
		Can2serial *out_can2serial;
		
		void vehicleSpeed_callback(const little_ant_msgs::State2::ConstPtr& msg);
		void send_vehicleMsg_callback(const ros::TimerEvent&);
		void parse_msg(CanMsg_t &can_msg);
		void pubBoundingBoxArray();
		
		ros::Publisher esr_pub;
		ros::Publisher boundingBox_pub;
		
		ros::Subscriber sub_vehicleSpeed_;
		
		std::string in_port_name_;
		std::string out_port_name_;
		
		bool is_pubBoundingBox_;
		bool is_sendMsgToEsr_;
		
		ros::Timer timer_50ms;
		
		uint16_t lastMsgId;
		
		int argc_;
		char **argv_;
		
		esr_radar_msgs::Objects objects;
		esr_radar_msgs::Objects last_frame_objects;
		jsk_recognition_msgs::BoundingBoxArray  boxes;
		boost::mutex mutex_;
		
		float vehicleSpeed_;
		

};

#endif
