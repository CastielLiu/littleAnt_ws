#include<iostream>
#include<cstdio>
#include<ros/ros.h>
#include<sensor_msgs/Imu.h>

using namespace std;

FILE * fp = NULL;

void imu_callback(const sensor_msgs::Imu::ConstPtr& imu)
{
	fprintf(fp,"%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\n", imu->header.stamp.toSec(),
			imu->angular_velocity.x,imu->angular_velocity.y,imu->angular_velocity.z,
			imu->linear_acceleration.x,imu->linear_acceleration.y,imu->linear_acceleration.z);
	
	fflush(fp);
}

int main(int argc,char **argv)
{
	ros::init(argc,argv,"imu_record_node");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	
	std::string imu_topic = nh_private.param<std::string>("imu_topic","/imu");
	ros::Subscriber sub_imu= nh.subscribe(imu_topic,1,&imu_callback);
	
	std::string file_path = nh_private.param<std::string>("file_path","");
	std::string file_name = nh_private.param<std::string>("file_name","");
	
	if(file_path.empty() || file_name.empty())
	{
		ROS_ERROR("please input file path and file name in launch file!!!");
		return false;
	}
	
	std::string full_file_name = file_path + file_name;
	
	fp = fopen(full_file_name.c_str(),"w");
	if(fp==NULL)
	{
		ROS_ERROR("open %s failed",full_file_name.c_str());
		return 0;
	}
	else
		ROS_INFO("open %s ok",full_file_name.c_str());
	
	fprintf(fp,"stamp    angle_speed_x_y_z \t line_acc_x_y_z\n");
	
	ROS_INFO("imu_record_node initial ok.");
	
	ros::spin();

	return 0;
}

