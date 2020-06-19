#include<iostream>
#include<cstdio>
#include<ros/ros.h>
#include<sensor_msgs/Imu.h>
#include<gps_msgs/Inspvax.h>

using namespace std;

class Recorder
{
private:
	void imu_callback(const sensor_msgs::Imu::ConstPtr& imu);
	void gps_callback(const gps_msgs::Inspvax::ConstPtr& gps);
	void timer_callback(const ros::TimerEvent&);
	std::string getDate();
	
	std::string file_path_;
	FILE *fp_;
	
	bool imu_ok_,gps_ok_;
	sensor_msgs::Imu::ConstPtr imu_msg_;
	gps_msgs::Inspvax::ConstPtr gps_msg_;
	
	ros::Subscriber sub_gps_;
	ros::Subscriber sub_ins_;
	ros::Timer      timer_;
	
public:
	Recorder();
	~Recorder();
	bool init();
};

Recorder::Recorder()
{
	imu_ok_ = false;
	gps_ok_ = false;
}

Recorder::~Recorder()
{
	if(fp_ != NULL)
		fclose(fp_);
}

void Recorder::imu_callback(const sensor_msgs::Imu::ConstPtr& imu)
{
	if(!imu_ok_)
		imu_ok_ = true;
	imu_msg_ = imu;
}

void Recorder::gps_callback(const gps_msgs::Inspvax::ConstPtr& gps)
{
	if(!gps_ok_)
		gps_ok_ = true;
	gps_msg_ = gps;
}

bool Recorder::init()
{
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	
	private_nh.param<std::string>("file_path",file_path_,"");
	if(file_path_.empty() || file_path_.empty())
	{
		ROS_ERROR("please input record file path and file name in launch file!!!");
		return false;
	}
	
	std::string gps_topic = private_nh.param<std::string>("gps_topic","/gps");
	std::string ins_topic = private_nh.param<std::string>("ins_topic","/imu");
	
	sub_gps_ = nh.subscribe(gps_topic ,1,&Recorder::gps_callback,this);
	sub_ins_ = nh.subscribe(ins_topic ,1,&Recorder::imu_callback,this);
	timer_   = nh.createTimer(ros::Duration(0.1), &Recorder::timer_callback, this);

	fp_ = fopen(file_path_.c_str(),"w");

	if(fp_ == NULL)
	{
		ROS_ERROR("open record data file %s failed !",file_path_.c_str());
		return false;
	}
	ROS_INFO("record gps ins initial ok.");
	return true;
}

std::string Recorder::getDate()
{
	std::stringstream date;
	double time = ros::Time::now().toSec();
	std::time_t time_s = time;
	int time_ms = (time - time_s)*100;
	
	date << std::put_time(std::localtime(&time_s),"%Y-%m-%d-%H:%M:%S.") << std::setw(2) << std::setfill('0') << time_ms;
	return date.str();
}

void Recorder::timer_callback(const ros::TimerEvent&)
{
	//等待所有数据就绪
	static int delay = 30;
	if(delay > 0)
	{
		--delay;
		return;
	}
	
	if(!gps_ok_ || !imu_ok_)
		return;
	
	float x_speed = gps_msg_->north_velocity * cos(gps_msg_->azimuth) + gps_msg_->east_velocity * sin(gps_msg_->azimuth) *3.6;
	
	fprintf(fp_,"%s\t%.7f\t%.7f\t%.3f\t%.2f\t%.3f\t%.3f\t%.2f\t%.2f\t%.2f\r\n",getDate().c_str(), gps_msg_->longitude, gps_msg_->latitude, gps_msg_->height,x_speed,
			imu_msg_->linear_acceleration.x, imu_msg_->linear_acceleration.y,gps_msg_->azimuth, gps_msg_->roll, gps_msg_->pitch);

	fflush(fp_);
}
	


int main(int argc,char **argv)
{
	ros::init(argc,argv,"record_gps_ins_node");
	
	Recorder recorder;
	
	if(recorder.init())
		ros::spin();
	
	

	return 0;
}

