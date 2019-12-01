#include "ESR_radar.h"
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>

using std::cout;
using std::endl;
using std::hex;


ESR_RADAR::ESR_RADAR(int argc,char ** argv):
	argc_(argc),
	argv_(argv) 
{
	in_can2serial =new Can2serial;
	out_can2serial = new Can2serial;
	objects.header.frame_id = std::string("esr_radar");
	lastMsgId = 0x000;
	
	vehicleSpeed_ = 0.0;
}

ESR_RADAR::~ESR_RADAR()
{
	delete out_can2serial;
	out_can2serial=NULL;
	
	delete in_can2serial;
	out_can2serial=NULL;
}


bool ESR_RADAR::init()
{
	ros::init(argc_,argv_,"esr_radar");
	ros::NodeHandle nh_private("~");
	ros::NodeHandle nh;
	nh_private.param<std::string>("in_port_name", in_port_name_, "/dev/ttyUSB1");
	nh_private.param<std::string>("out_port_name", out_port_name_, "/dev/ttyUSB2");
	
	nh_private.param<bool>("is_pubBoundingBox",is_pubBoundingBox_,false);
	
	nh_private.param<bool>("is_sendMsgToEsr",is_sendMsgToEsr_,false);
	
	ROS_INFO("in_port_name:%s",in_port_name_.c_str());
	ROS_INFO("out_port_name:%s",out_port_name_.c_str());
	ROS_INFO("is_pubBoundingBox:%d",is_pubBoundingBox_);
	ROS_INFO("is_sendMsgToEsr:%d",is_sendMsgToEsr_ );
	
	esr_pub = nh.advertise<esr_radar_msgs::Objects>("/esr_radar",10);
	
	sub_vehicleSpeed_ = nh.subscribe<little_ant_msgs::State2>("/vehicleState2",2,&ESR_RADAR::vehicleSpeed_callback,this);
	
	if(is_sendMsgToEsr_ )
	{
		if(!out_can2serial->configure_port(out_port_name_.c_str()))
		{
			ROS_ERROR("open esr_out port %s failed!",out_port_name_.c_str());
			return 0;
		}
	
		out_can2serial->clearCanFilter();
		
		out_can2serial->setCanFilter(1,0x00,0x7ff);
		
		out_can2serial->configBaudrate(500);
		ROS_INFO("send to esr initialization complete");
		
	}
	
	if(!in_can2serial->configure_port(in_port_name_.c_str()))
	{
		ROS_ERROR("open esr port %s failed!",in_port_name_.c_str());
		return 0;
	}
	
	//in_can2serial->clearCanFilter(); usleep(10000);
	//in_can2serial->clearCanFilter(); usleep(10000);

	
	//in_can2serial->setCanFilter_alone(0x01,0x4E0); usleep(1000);
	in_can2serial->setCanFilter_alone(0x01,0x4E0); usleep(1000);
	in_can2serial->setCanFilter(0x02,0x500,0x7c0); //500-53f
	//in_can2serial->setCanFilter(0x03,0x500,0x7C0);
	
	//in_can2serial->configBaudrate(500);
		
	usleep(10000);
	in_can2serial->StartReading();
	
	for(uint8_t i=0;i<13;i++)
	{
		in_can2serial->inquireFilter(i); 
		usleep(10000);
	}
	
	ROS_INFO("esr radar initialization complete");

	return 1;
}



void ESR_RADAR::run()
{
	boost::thread parse_thread(boost::bind(&ESR_RADAR::handleCanMsg,this));
	
	if(is_pubBoundingBox_)
		this->start_publishBoundingBoxArray_thread();
	
	if(is_sendMsgToEsr_)
	{
		ros::NodeHandle nh;
		timer_50ms = nh.createTimer(ros::Duration(0.05),&ESR_RADAR::send_vehicleMsg_callback,this);
	}
		
	ros::spin();
}

void ESR_RADAR::start_publishBoundingBoxArray_thread()
{
	boost::thread pubBoundingBoxThread(boost::bind(&ESR_RADAR::pubBoundingBoxArray,this));
}

void ESR_RADAR::pubBoundingBoxArray()
{
	ros::NodeHandle nh;
	boundingBox_pub = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("/esr_box",2);
	this->boxes.header.frame_id = "esr_radar";
	
	jsk_recognition_msgs::BoundingBox box;
	box.header.frame_id = "esr_radar";
	box.dimensions.x = 1.0;
	box.dimensions.y =1.0;
	box.dimensions.z = 1.0;
	
	ros::Rate r(20);
	
	while(ros::ok())
	{
		boxes.boxes.clear();
		mutex_.lock();
		for(size_t i=0;i<last_frame_objects.size;i++)
		{
			box.pose.position.x = last_frame_objects.objects[i].x;
			box.pose.position.y = last_frame_objects.objects[i].y;
			box.pose.position.z = 0.0;  //install_height
			boxes.boxes.push_back(box);
		}
		mutex_.unlock();
		if(last_frame_objects.size) 
			boundingBox_pub.publish(boxes);
		r.sleep();
	}
}

void ESR_RADAR::handleCanMsg()
{
	CanMsg_t can_msg;
	while(ros::ok())
	{
		usleep(1000);
		
		if(!in_can2serial->getCanMsg(can_msg)) 
		{
			//ROS_ERROR("nothing");
			continue;
		}
		parse_msg(can_msg);
		
	}
}


void ESR_RADAR::parse_msg(CanMsg_t &can_msg)
{
	static uint16_t scan_index;
//	cout << "ID:" << hex << can_msg.ID <<endl;
/*	
	for(size_t i=0;i<can_msg.len;i++)
	 printf("%x\t",can_msg.data[i]);
	 printf("\n");
*/	
	
	if(can_msg.ID == 0x4E0 || can_msg.ID < lastMsgId)
	{
		if(can_msg.ID == 0x4E0)
			scan_index = can_msg.data[3]*256 + can_msg.data[4];
		else
			scan_index +=1;
			
		objects.sequence = scan_index;
		objects.size = objects.objects.size();
		if(objects.size)
			esr_pub.publish(objects);
		if(is_pubBoundingBox_)
			last_frame_objects = objects;
		
		objects.objects.clear();
	}
	
	if(can_msg.ID <= 0x53f && can_msg.ID >=0x500)
	{
		if(objects.objects.size() > 63)
		{
			objects.objects.clear();
			return ;
		}
		
		uint8_t measurementStatus = (can_msg.data[1]&0xE0) >> 5;
		
		//if(measurementStatus !=1 && measurementStatus !=3 && measurementStatus !=4)
		//	return;
		
		uint16_t u16_tempAngle = (can_msg.data[1] &0x1F);
		u16_tempAngle <<=5;
		u16_tempAngle += ((can_msg.data[2] &0xf8)>>3) ;
		
		int16_t s16_angle = (int16_t)u16_tempAngle;
		
		if((can_msg.data[1]&0x10) !=0 )
			s16_angle -=1024;
		
		uint16_t u16_distance = (can_msg.data[2] &0x07);
		u16_distance <<=8;
		u16_distance += can_msg.data[3];
		
		uint16_t u16_tempTargetSpeed = can_msg.data[6] & 0x1f; 
		u16_tempTargetSpeed <<= 8;
		u16_tempTargetSpeed += can_msg.data[7];
		
		int16_t s16_targetSpeed =(int16_t)u16_tempTargetSpeed;
		
		if((can_msg.data[6]&0x20) !=0)
			s16_targetSpeed -= 8192;
		
		esr_radar_msgs::Object object;
		
		object.azimuth = s16_angle*0.1 - 1.2;  //left is negative(-)  //subtract the offset

		object.distance = u16_distance*0.1;
		
		if(object.distance>100.0)
			return;
		
		object.x = object.distance*sin(object.azimuth*M_PI/180.0);
		object.y = object.distance*cos(object.azimuth*M_PI/180.0);
		
		if(!object.x && !object.y) return;

		object.speed = s16_targetSpeed*0.01;  //m/s
		
		object.status = measurementStatus;
		
		object.id = can_msg.ID;
		
		if(object.distance > 0.5)
			objects.objects.push_back(object);
		
		/*
		switch(measurementStatus)
		{	
			case 1: //new target
				cout <<"ID :" << hex << can_msg.ID << "   new     target ----> angle:" <<object.azimuth << "\t range:" 
					 <<object.distance <<"\tspeed:" <<object.speed << endl;
				break;
			case 3: //update target
				cout <<"ID :" << hex << can_msg.ID << "   update  target ----> angle:" <<object.azimuth << "\t range:" 
					 <<object.distance <<"\tspeed:" <<object.speed << endl;
				break;
			case 4: //coasted target
				cout <<"ID :" << hex << can_msg.ID << "   coasted target ----> angle:" <<object.azimuth << "\t range:" 
					 <<object.distance <<"\tspeed:" <<object.speed << endl;
				break;
					
			default:
				break;
		}*/
	}
	lastMsgId = can_msg.ID;
}

void ESR_RADAR::send_vehicleMsg_callback(const ros::TimerEvent&)
{
	CanMsg_t canMsg5F2 = {0x5F2,8};
	canMsg5F2.type = 0x03; //std can msg
	uint8_t installHeight = 20;
	 
	canMsg5F2.data[4] &=0x80; //clear low 7bits
	canMsg5F2.data[4] |= installHeight & 0x7f;//install_height
	
	out_can2serial->sendCanMsg(canMsg5F2);
	
	CanMsg_t canMsg4F0 = {0x4F0,8};
	canMsg4F0.type = 0x03;//std can msg
	
	uint16_t speed = vehicleSpeed_/0.0625 ; 
	canMsg4F0.data[0] = speed>>3;
	canMsg4F0.data[1] |= (speed%8)<<5 ;
	
	canMsg4F0.data[1] &= 0xef;  // speed direction 0=forward
	out_can2serial->sendCanMsg(canMsg4F0);
	
	CanMsg_t canMsg4F1 = {0x4F1,8};
	canMsg4F1.type = 0x03;//std can msg
	
	canMsg4F1.data[7] |= 0x20 ; //speed valid
	out_can2serial->sendCanMsg(canMsg4F1);
	
}

void ESR_RADAR::vehicleSpeed_callback(const little_ant_msgs::State2::ConstPtr& msg)
{
	static int i=0;
	vehicleSpeed_ = (msg->wheel_speed_FL + msg->wheel_speed_RR)/2*5.0/18; //m/s

	i++;
	if(i%20==0)
		ROS_INFO("callback speed:%f ",vehicleSpeed_);
}


int main(int argc,char **argv)
{
	
	ESR_RADAR radar(argc,argv);
	
	if(!radar.init())
		return 1;
		
	radar.run();
	
	return 0;
}












 

