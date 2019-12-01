#include "lane_keeping/lane_keeping.h"
#include<assert.h>
#include<cmath>

LaneKeeping::LaneKeeping():
	current_lane_msg_index(0),
	system_delay_(50),
	mean_vehicleSpeed_(0),
	gps_status_(0x00),
	system_status_(KeepLine_status)
{
	cmd_.origin = little_ant_msgs::ControlCmd::_LANE_KEEPING;
	cmd_.status = true;
	cmd_.cmd2.set_gear = 1;
	cmd_.cmd1.set_driverlessMode = true;
}

LaneKeeping::~LaneKeeping()
{

}

bool LaneKeeping::init()
{
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	
	nh_private.param<float>("foresight_distance",foresight_distance_,10.0);
	nh_private.param<float>("lane_keeping_speed",lane_keeping_speed_,10.0);
	nh_private.param<float>("changeLane_speed",changeLane_speed_,5.0);
	
	sub_laneMsg_ = nh.subscribe("/lane",1,&LaneKeeping::laneDetect_callback,this);
	
	sub_vehicleSpeed_ = nh.subscribe("/State2",1,&LaneKeeping::vehicleSpeed_callback,this);
	
	pub_controlCmd_ = nh.advertise<little_ant_msgs::ControlCmd>("/sensor_decision",2);
	
	pub_cmd_20ms_ = nh.createTimer(ros::Duration(0.02), &LaneKeeping::pub_cmd_callback,this);
	
	sub_cartesian_gps_ = nh.subscribe("/gps_odom",1,&LaneKeeping::cartesian_gps_callback,this);
	
	sub_polar_gps_ = nh.subscribe("/gps",1,&LaneKeeping::gps_callback,this);
	
	sub_start_change_lane_ = nh.subscribe("/is_change_lane",1,&LaneKeeping::change_lane_callback,this);
	
	
	
	while(ros::ok() && gps_status_!= 0x03)
	{
		ros::spinOnce();
		ROS_INFO("gps status is %x ,waiting for gps initial .....",gps_status_);
		usleep(100000);
	}
}

void LaneKeeping::run()
{
	boost::shared_ptr<boost::thread> new_thread_ptr = 
		boost::shared_ptr<boost::thread >(new boost::thread(boost::bind(&LaneKeeping::generate_cmd_thread, this)));
		
	ros::spin();
}

void LaneKeeping::generate_cmd_thread()
{
	ros::Rate loop_rate(20);
	
	while(ros::ok())
	{
		if(system_status_ == ChangeLine_Left_status)
		{
			changeLane(-1,widthOfLane_);
			ROS_INFO("changeLane ok , start to keepLane");
			system_status_ = KeepLine_status;
		}
		else if(system_status_ == ChangeLine_Right_status)
		{
			changeLane(1,widthOfLane_);
			ROS_INFO("changeLane ok , start to keepLane");
			system_status_ = KeepLine_status;
		}
		else if(system_status_ == KeepLine_status)
			keepLane();
			
		loop_rate.sleep();
	}
}


void LaneKeeping::pub_cmd_callback(const ros::TimerEvent&)
{
	if(system_delay_ >0)
		return;
	
	if(system_status_ != None_status)
		pub_controlCmd_.publish(cmd_);
}

void LaneKeeping::keepLane()
{
	//float alpha = asin(lane_msg_.distance_from_center / foresight_distance_) ;//distance_from_center =OA
	
	//distance_from_center =OB
	float alpha = asin(lane_msg_.err *cos(lane_msg_.theta) / foresight_distance_) ;
	
	float delta = fabs(lane_msg_.theta) - fabs(alpha);
	
	//ROS_INFO("alpha:%f\t theta:%f \t delta:%f",alpha*180.0/M_PI,lane_msg_.included_angle*180.0/M_PI,delta*180.0/M_PI);
	
	float steering_radius = 0.5*foresight_distance_ / sin(delta) ;
	
	float angle = saturationEqual(generateRoadwheelAngleByRadius(steering_radius),15.0) ;
		
	cmd_.cmd2.set_roadWheelAngle = fabs(angle) * get_steeringDir(lane_msg_.err, lane_msg_.theta,alpha);
	
	cmd_.cmd2.set_speed = lane_keeping_speed_;
	
	/*
	ROS_INFO("vehicle_speed:%f\t steerAngle:%f\t included_angle:%f\t err:%f",
				mean_vehicleSpeed_,
				cmd_.cmd2.set_steeringAngle,
				lane_msg_.included_angle*180.0/M_PI,
				lane_msg_.distance_from_center);
	*/

}


void LaneKeeping::laneDetect_callback(const little_ant_msgs::Lane::ConstPtr& msg)
{
	lane_msg_ = *msg;
	
	history_status_msgs_[current_lane_msg_index].lane_msg = *msg;
	history_status_msgs_[current_lane_msg_index].vehicle_speed = vehicle_speed_;
	
	if(system_delay_ > 0)
		system_delay_ --;
	else
	{
		int lastIndex = (current_lane_msg_index + 1 == Max_history_size) ? 0 : current_lane_msg_index+1;
		float time_interval = history_status_msgs_[current_lane_msg_index].lane_msg.header.stamp.toSec() -
							  history_status_msgs_[lastIndex].lane_msg.header.stamp.toSec();
							   
		float diff_of_lateralErr = history_status_msgs_[current_lane_msg_index].lane_msg.err - 
										history_status_msgs_[lastIndex].lane_msg.err;
		
		mean_vehicleSpeed_ = 0.0;
		for(size_t i=0; i<Max_history_size; i++)
			mean_vehicleSpeed_ += history_status_msgs_[i].vehicle_speed;
			
		mean_vehicleSpeed_ /= Max_history_size;
			
		//mean_vehicleSpeed_ += (history_status_msgs_[current_lane_msg_index].vehicle_speed - 
		//					  history_status_msgs_[lastIndex].vehicle_speed)/(Max_history_size-1);
		
		//ROS_INFO("lastIndex:%d\t currentIndex:%d",lastIndex,current_lane_msg_index);				  
		//for(int i=0;i<Max_history_size;i++)
		//{
		//	std::cout << history_status_msgs_[i].vehicle_speed <<"\t";
		//}
		//std::cout <<std::endl;
		
		float distance_between_twoTime ;
		
		if( mean_vehicleSpeed_ >0.0)
		{
			distance_between_twoTime = mean_vehicleSpeed_ * time_interval;
			
			//lane_msg_.included_angle = asin(diff_of_lateralErr/distance_between_twoTime) ; //distance_from_center = OA
			 lane_msg_.theta = atan(diff_of_lateralErr/distance_between_twoTime) ;  //distance_from_center = OB
		}
	}
	
	current_lane_msg_index = (current_lane_msg_index+1 == Max_history_size) ? 0: current_lane_msg_index+1;
	
	//ROS_INFO("distance_from_center:%f\t angle1:%f\t angle2:%f",
	//		lane_msg_.err,msg->theta*180.0/M_PI,lane_msg_.theta*180.0/M_PI);
	
	lane_msg_.theta = msg->theta;/////////////
	
	
	//generate_laneChange_points(-1,3.0);		

}

void LaneKeeping::vehicleSpeed_callback(const little_ant_msgs::State2::ConstPtr& msg)
{
	vehicle_speed_ = msg->vehicle_speed;
}

//left -1 ; right 1
int LaneKeeping::get_steeringDir(float err,float theta,float alpha)
{
	if(err==0 && theta==0)
		return 0;
	else if(err<=0 && theta<=0)
		return -1;
	else if(err>=0 && theta>=0)
		return 1;
	else if(err<0)
		return fabs(theta) > fabs(alpha) ? 1:-1;
	else if(err>0)
		return fabs(theta) > fabs(alpha) ? -1:1;
	else
		return 0;
}

void LaneKeeping::generate_laneChange_points(int dir,float widthOfLane)
{
	//车辆离目标车道中心点的距离
	float dis2targetLane = dir * lane_msg_.err*cos(lane_msg_.theta) + widthOfLane;
		
	//车道的方向角
	float yawOfLane=0.0;
	float sum = 0.0;
	for(size_t i=0;i<20;i++)
	{
		yawOfLane = current_point_.yaw + lane_msg_.theta;
		sum += yawOfLane;
		std::cout << "yawOfLane: "<< yawOfLane*180.0/M_PI << std::endl;
		usleep(10000);
	}
	yawOfLane = sum / 20;
	std::cout << "average yawOfLane: "<< yawOfLane*180.0/M_PI << std::endl;
	
	
	//目标车道中心点相对车辆的坐标
	//float x0 = dir * dis2targetLane * cos(lane_msg_.included_angle);
	//float y0 = dir * dis2targetLane * sin(lane_msg_.included_angle);
	
	float x0 = dir*dis2targetLane;
	float y0 = 0.0;
	
	
	
	ROS_INFO("target lane center x0:%lf\t y0:%lf",x0,y0);
	ROS_INFO("X:%f\t Y:%f",current_point_.x,current_point_.y);
	//目标车道中心点绝对坐标
	float x =  x0 * cos(current_point_.yaw) + y0 * sin(current_point_.yaw) + current_point_.x;
	float y = -x0 * sin(current_point_.yaw) + y0 * cos(current_point_.yaw) + current_point_.y;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	printf("x:%f \t y:%f\n",x,y);
	
	temp_targetArray_.clear();
	
	gpsMsg_t target ;
	
	for(int i=0; i<50;i++)
	{
		target.x = x + (foresight_distance_ + i*1.0)*sin(yawOfLane);
		target.y = y + (foresight_distance_ + i*1.0)*cos(yawOfLane);
		temp_targetArray_.push_back(target);
		
		printf("%f\t%f\n",target.x-current_point_.x,target.y-current_point_.y);
		
	}
}

void LaneKeeping::changeLane(int dir,float widthOfLane)
{
	generate_laneChange_points(dir ,widthOfLane);
	
	float x,y,distance,t_yaw,yaw_err,turning_radius,t_roadWheelAngle;
	
	for(int i=0;i<temp_targetArray_.size(); )
	{  //path_tracking 
		x = temp_targetArray_[i].x - current_point_.x;
		y = temp_targetArray_[i].y - current_point_.y;
		distance = sqrt(x*x+y*y);
		
		if(distance < foresight_distance_)
		{
			i++;
			continue;
		}
		
		t_yaw = atan2(x,y);
		if(t_yaw <0)
			t_yaw += 2*M_PI;
		
		yaw_err = t_yaw - current_point_.yaw;
		if(yaw_err==0.0) continue;
		
		turning_radius = (0.5 * distance)/sin(yaw_err);
		
		t_roadWheelAngle = generateRoadwheelAngleByRadius(turning_radius);
		
		t_roadWheelAngle = saturationEqual(t_roadWheelAngle, 20.0);
		
		cmd_.cmd2.set_speed = changeLane_speed_;
		cmd_.cmd2.set_roadWheelAngle = -t_roadWheelAngle;
		//ROS_INFO("lane changing... distance:%f  yaw_err:%f ",distance,yaw_err*180.0/M_PI);
	}
}

void LaneKeeping::gps_callback(const gps_msgs::Inspvax::ConstPtr &msg)
{
	gps_status_ |= 0x01;
	
	//current_point_.longitude = msg->longitude;
	//current_point_.latitude = msg->latitude;
	current_point_.yaw = deg2rad(msg->azimuth);
	//std::cout << "call_back ID: "<< std::this_thread::get_id()  << std::endl;
}

void LaneKeeping::cartesian_gps_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	gps_status_ |= 0x02;
	
	current_point_.x = msg->pose.pose.position.x;
	current_point_.y = msg->pose.pose.position.y;
	/*
	tf::Quaternion quat;
	tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
	
	double roll,pitch;
	
	tf::Matrix3x3(quat).getRPY(roll, pitch, current_point_.yaw);
*/
}

void LaneKeeping::change_lane_callback(const little_ant_msgs::ChangLane::ConstPtr& msg)
{
	if(msg->valid ==0)
		return ;
		
	widthOfLane_ = msg->widthOfLane;
	if(msg->dir == -1)
		system_status_ = ChangeLine_Left_status;
	else
		system_status_ = ChangeLine_Right_status;
}


int main(int argc, char** argv)
{
	ros::init(argc,argv,"lane_keeping_node");
	
	LaneKeeping lane_keeping;
	
	if(lane_keeping.init())
		lane_keeping.run();
	
	
	return 0;

}




