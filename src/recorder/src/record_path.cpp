#include<ros/ros.h>
#include "gps_msgs/Inspvax.h"
#include <unistd.h>
#include<cmath>
#include<nav_msgs/Odometry.h> 
#include<ant_math/ant_math.h>

class Record
{
	private:
		void cartesian_gps_callback(const nav_msgs::Odometry::ConstPtr& msg);
		std::string file_path_;
		std::string file_name_;
		
		FILE *fp;
		FILE *fp_wgs84;
		
		bool record_wgs84_; 
		
		gpsMsg_t last_point , current_point;
		bool is_generate_curvature_;
		float sample_distance_;
		ros::Subscriber sub_gps_;
		ros::Subscriber sub_cartesian_gps_ ;
		
	public:
		Record();
		~Record();
		bool init();
		void run();
		void recordToFile();
};

Record::Record()
{
	last_point = {0.0,0.0,0.0,0.0,0.0};
	current_point = last_point;
}

Record::~Record()
{
	if(fp != NULL)
		fclose(fp);
	if(fp_wgs84 != NULL)
		fclose(fp_wgs84);
}

bool Record::init()
{
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	
	private_nh.param<std::string>("file_path",file_path_,"");
	private_nh.param<std::string>("file_name",file_name_,"");
	private_nh.param<bool>("curvature_gen",is_generate_curvature_,false);
	private_nh.param<bool>("record_wgs84",record_wgs84_,true);
	
	if(file_path_.empty() || file_name_.empty())
	{
		ROS_ERROR("please input record file path and file name in launch file!!!");
		return false;
	}
	
	private_nh.param<float>("sample_distance",sample_distance_,0.1);
	std::string utm_topic = private_nh.param<std::string>("utm_topic","/ll2utm_odom");
	
	sub_cartesian_gps_ = nh.subscribe(utm_topic ,1,&Record::cartesian_gps_callback,this);

	fp = fopen((file_path_+file_name_).c_str(),"w");
	
	if(record_wgs84_)
	{
		fp_wgs84 = fopen((file_path_+"wgs84"+file_name_).c_str(),"w");
		if(fp_wgs84 == NULL)
		{
			ROS_ERROR("open record data file %s failed !!!",(file_path_+"wgs84"+file_name_).c_str());
			return false;
		}
	}
		
	if(fp == NULL)
	{
		ROS_ERROR("open record data file %s failed !!!",(file_path_+file_name_).c_str());
		return false;
	}
	
	return true;
}

void Record::run()
{
	ros::spin();
	if(!is_generate_curvature_)
		return;
	
	// generate curvature
	std::string package_path;
	ros::param::get("~pkg_path",package_path);
	std::string tool_file = package_path + "/scripts/generate_curvature.py";
	std::string raw_file = package_path + "/data/path_data/raw/" + file_name_;
	std::string result_file = package_path + "/data/path_data/result/" + file_name_;
	std::string cmd = std::string("python ") + tool_file + " " + raw_file + " " + result_file;
	std::cout << "\n=====================================================\n";
	std::cout << "start to generate curvature... "<< std::endl;
	system(cmd.c_str());
	std::cout << "generate curvature complete..." << std::endl;
	std::cout << "=====================================================\n";
	
}

void Record::cartesian_gps_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	static size_t  row_num = 0;
	current_point.x = msg->pose.pose.position.x;
	current_point.y = msg->pose.pose.position.y;
	current_point.yaw = msg->pose.covariance[0];
	
	if(sample_distance_*sample_distance_ <= dis2Points(current_point,last_point,false))
	{
		fprintf(fp,"%.3f\t%.3f\t%.4f\n",current_point.x,current_point.y,current_point.yaw);
		fflush(fp);
		
		if(record_wgs84_)
		{
			fprintf(fp_wgs84,"%.7f,%.7f\r\n",msg->pose.covariance[2],msg->pose.covariance[1]);
			fflush(fp_wgs84);
		}

		ROS_INFO("row:%d\t%.3f\t%.3f\t%.3f",row_num++,current_point.x,current_point.y,current_point.yaw);
		last_point = current_point;
	}
}

int main(int argc,char**argv)
{
	ros::init(argc,argv,"record_data_node");
	
	Record recorder;
	
	if(!recorder.init())
		return 1;
	recorder.run();
	
	return 0;
}


