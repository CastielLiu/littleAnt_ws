#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "robot_tf_publisher");
    ros::NodeHandle nh;
    ros::Rate r(10);
    tf::TransformBroadcaster transform_broadcaster;
    while (ros::ok())
    {
       transform_broadcaster.sendTransform( tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.1, 0.0, 0.2)), ros::Time::now(),"world", "lidar"));

transform_broadcaster.sendTransform( tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-0.1, 0.0, -0.2)), ros::Time::now(),"world", "camera"));


        r.sleep();
    }
    
    return 0;
}