#include "acc/acc_esr.h"


Acc_esr::Acc_esr(ros::NodeHandle nh,ros::NodeHandle nh_private) :
	nh_(nh),
	nh_private_(nh_private),
	is_car_following_(false),
	max_target_search_distance_(30.0)
{
	cmd_.cmd2.set_gear =1;
	cmd_.cmd1.set_driverlessMode =true;
	cmd_.origin = little_ant_msgs::ControlCmd::_ESR_RADAR;
	cmd_.status = false;
	cmd_.cmd2.set_brake = 0.0;
	
	vehicleSpeed_ = 0.0;
	is_acc_ = false;
	
	acc_targetId_ = 0xff; //no target
	lastTime_of_seekTarget_ = 0.0;
	tracking_distance_ = 10.0;
}

bool Acc_esr::init()
{
	sub_esrObjects_ = nh_.subscribe("/esr_radar",2,&Acc_esr::object_callback,this);
	sub_vehicleSpeed_ = nh_.subscribe("/vehicleState2",2,&Acc_esr::vehicleSpeed_callback,this);
	sub_start_acc_ = nh_.subscribe("/is_acc",2,&Acc_esr::is_acc_callback,this);
	
	sub_carFollow_request_ = nh_.subscribe("/carFollow_request",1,&Acc_esr::carFollowRequest_callback,this);
	sub_current_scene_ = nh_.subscribe("/current_scene",1,&Acc_esr::current_scene_callback,this);
	
	pub_car_follow_response_ = nh_.advertise<std_msgs::Bool>("/carFollow_response",1);
	
	pub_cmd_ = nh_.advertise<little_ant_msgs::ControlCmd>("/sensor_decision",2);
	
	nh_private_.param<float>("trackTargetAngle_range",trackTargetAngle_range_,1.0);
	nh_private_.param<float>("max_following_speed",max_following_speed_,15.0);
	
	updateTargetStatus_timer_ = nh_.createTimer(ros::Duration(0.10),&Acc_esr::updateTargetStatus_callback,this);
	
//	car_follow_thread_ptr_ = boost::shared_ptr<boost::thread >(new boost::thread(boost::bind(&Acc_esr::carFollowThread, this)));
}

void Acc_esr::run()
{
	ros::spin();
}

void Acc_esr::carFollowRequest_callback(const esr_radar_msgs::Objects::ConstPtr& msg)
{
	max_target_search_distance_ = 30.0;
	for(size_t i=0; i< msg->objects.size(); i++)
	{
		max_target_search_distance_ = 
				msg->objects[i].y > max_target_search_distance_ ? msg->objects[i].y : max_target_search_distance_;
	}
	is_acc_ = true;
}

void Acc_esr::carFollowThread()
{
	ros::Rate loop_rate(1); // nothing
	while(ros::ok())
	{
		if(is_car_following_)
		{
		
		}
		loop_rate.sleep();
	}
}

void Acc_esr::vehicleSpeed_callback(const little_ant_msgs::State2::ConstPtr& msg)
{
	/*static int i=0;
	i++;
	if(i%20==0)
		ROS_INFO("vehicle speed:%f ",vehicleSpeed_);*/
	
	vehicleSpeed_ = msg->vehicle_speed; //m/s
		
	tracking_distance_ = 0.5* vehicleSpeed_ * vehicleSpeed_ /5.0 * 3  + 8.0;
}


void Acc_esr::object_callback(const esr_radar_msgs::Objects::ConstPtr& objects)
{
	if(!is_acc_ ) return;
	if(current_scene_ != TrafficSign_BusStop && current_scene_ != TrafficSign_CarFollow )
		return;
	
	if(acc_targetId_==0xff) //no track target
	{
		float min_distance=500.0; //a big num
		
		publishCarFollowingStats(false);
		
		//ROS_INFO("finding target........ max_target_search_distance_:%f",max_target_search_distance_);
		
		potentialTarget_num_=0; 
		
		for(size_t i=0;i< objects->size;i++)
		{
			//ROS_INFO("angle:%f  anglerange:%f",objects->objects[i].azimuth,trackTargetAngle_range_);
			if((objects->objects[i].azimuth <trackTargetAngle_range_) &&
				(objects->objects[i].azimuth >-trackTargetAngle_range_)&&
				objects->objects[i].distance <= max_target_search_distance_ &&
				objects->objects[i].speed + vehicleSpeed_ > 0.5 )
			{
				potentialTarget_index_[potentialTarget_num_++] = i; 
			}
		}
		//ROS_INFO("objects->size:%d \tpotentialTarget_num_:%d",objects->size,potentialTarget_num_);
		for(size_t i=0;i<potentialTarget_num_;i++)
		{
			if(objects->objects[potentialTarget_index_[i]].distance < min_distance)
			{
				min_distance = objects->objects[potentialTarget_index_[i]].distance;
				acc_targetId_ = objects->objects[potentialTarget_index_[i]].id;
			}
		}
		//ROS_INFO("target locked distance:%f  azimuth:%f",
		//		objects->objects[potentialTarget_index_[i]].distance,
		//		objects->objects[potentialTarget_index_[i]].azimuth);
	}
	else
	{
		cmd_.status = false;
		cmd_.cmd2.set_brake = 0.0;
		for(size_t i=0;i<objects->size;i++)
		{
			if(objects->objects[i].id == acc_targetId_ && objects->objects[i].status != 1)  //!=newTarget
			{
				trackTargetMsg_ = objects->objects[i];
				//ROS_INFO("target Id:%x  angle:%f  distance:%f speed:%f",
				//			trackTargetMsg_.id,trackTargetMsg_.azimuth,trackTargetMsg_.distance,trackTargetMsg_.speed);
				
				lastTime_of_seekTarget_ = ros::Time::now().toSec();
				publishCarFollowingStats(true);
				cmd_.status = true;
				break;
			}
			else if(objects->objects[i].id == acc_targetId_ && objects->objects[i].status == 1)
			{
				this->setTrackTargetLost();
				break;
			}
		}
	}
	if(cmd_.status == true)
	{
		//distanceErr>0    acceleration
		//distanceErr<0    deceleration
		float distanceErr = trackTargetMsg_.distance -tracking_distance_;
		float t_speed;  //m/s
		if(distanceErr >= 0)
			t_speed = vehicleSpeed_ + trackTargetMsg_.speed + (trackTargetMsg_.distance -tracking_distance_) *0.5;  //wait debug
		else
			t_speed = vehicleSpeed_ + trackTargetMsg_.speed + (trackTargetMsg_.distance -tracking_distance_) *0.3;
			
		ROS_INFO("target speed:%f \t vehicle speed:%f \t t_speed:%f\t t_dis:%f\t dis:%f",
						vehicleSpeed_ + trackTargetMsg_.speed,vehicleSpeed_,t_speed,tracking_distance_,trackTargetMsg_.distance);
		
		if(t_speed > max_following_speed_/3.6)
			cmd_.cmd2.set_speed = max_following_speed_ * 3.6; //km/h
		else if(t_speed >0.0)
			cmd_.cmd2.set_speed = t_speed * 3.6; //km/h
		else
		{
			//ROS_INFO("here...............................");
			cmd_.cmd2.set_speed = 0.0;   //emergencyBrake!!
			cmd_.cmd2.set_brake = 35.0;
		}
	}
	pub_cmd_.publish(cmd_);
	
}

//extern acc command
void Acc_esr::is_acc_callback(const std_msgs::Bool::ConstPtr& state)
{
	if(is_acc_ && !state->data)
	{
		ROS_INFO("Acc mode exited...");
		cmd_.status = false;
		lastTime_of_seekTarget_ = 0.0;
	}
	else if(!is_acc_ && state->data)	
	{
		ROS_INFO("Acc mode started...");
		cmd_.status = true;
	}
	is_acc_ = state->data;
}

void Acc_esr::updateTargetStatus_callback(const ros::TimerEvent&)
{
	if(is_acc_ && (acc_targetId_!=0xff) && 
		(ros::Time::now().toSec()-lastTime_of_seekTarget_)>0.2 && 
		lastTime_of_seekTarget_ >10.0) //overtime
	{
		this->setTrackTargetLost();
	}
}

inline void Acc_esr::setTrackTargetLost()
{
	is_acc_ = false;
	acc_targetId_ = 0xff;
	ROS_ERROR("acc target lost! acc has to exited !!");
	lastTime_of_seekTarget_ = 0.0;
	publishCarFollowingStats(false);
	cmd_.status = false;
}

inline void Acc_esr::publishCarFollowingStats(bool status)
{
	std_msgs::Bool is_can_follow;
	is_can_follow.data = status;
	pub_car_follow_response_.publish(is_can_follow);
	
}

void Acc_esr::current_scene_callback(const std_msgs::UInt8::ConstPtr& msg)
{
	current_scene_ = msg->data;
}


int main(int argc,char ** argv)
{
	ros::init(argc,argv,"acc_node");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	
	Acc_esr acc(nh,nh_private);
	if(acc.init())
		acc.run();
	
	return 0;
}


