#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <gps_msgs/Inspvax.h>
#include <tf/tf.h>
#include <cmath>

typedef struct
{
	double longitude;
	double latitude;
	float yaw;
}gpsMsg_t;

gpsMsg_t first_point,current_point;

bool first_time_flag = 1;

void gps_callback(const gps_msgs::Inspvax::ConstPtr &msg)
{
	current_point.latitude = msg->latitude;
	current_point.longitude =msg->longitude;
	current_point.yaw = msg->azimuth;
	
	if(first_time_flag ==1)
	{
		first_point = current_point;
		first_time_flag =0;
	}
	
	current_point.longitude -= first_point.longitude;
	current_point.latitude -= first_point.latitude;
	current_point.longitude *= 100000;
	current_point.latitude *= 100000;
}

int main (int argc, char **argv)
{
    ros::init (argc, argv, "showpath");

    ros::NodeHandle nh;
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("trajectory",1, true);
    ros::Subscriber gps_sub = nh.subscribe("/gps",1,&gps_callback);

    ros::Time current_time;
    current_time = ros::Time::now();

    nav_msgs::Path path;
    //nav_msgs::Path path;
    path.header.stamp=current_time;
    path.header.frame_id="world";

    ros::Rate loop_rate(20);
    
    while (ros::ok())
    {

        current_time = ros::Time::now();

        geometry_msgs::PoseStamped current_pose_stamped;
        current_pose_stamped.pose.position.x = current_point.longitude;
        current_pose_stamped.pose.position.y = current_point.latitude;

        geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(current_point.yaw *M_PI/180.);
        current_pose_stamped.pose.orientation.x = goal_quat.x;
        current_pose_stamped.pose.orientation.y = goal_quat.y;
        current_pose_stamped.pose.orientation.z = goal_quat.z;
        current_pose_stamped.pose.orientation.w = goal_quat.w;

        current_pose_stamped.header.stamp=current_time;
        current_pose_stamped.header.frame_id="world";
        path.poses.push_back(current_pose_stamped);

        path_pub.publish(path);
        ros::spinOnce();               // check for incoming messages

        loop_rate.sleep();
    }

    return 0;
}

