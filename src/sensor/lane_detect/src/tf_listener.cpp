#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
 geometry_msgs::PointStamped base_point;
 geometry_msgs::PointStamped lidar_point;


   tf::StampedTransform world_lidar_tf;
    tf::StampedTransform world_camera_tf;
     tf::StampedTransform lidar_camera_tf;
/*tf::StampedTransform PixelCloudFusion::FindTransform(const std::string &target_frame, const std::string source_frame)
{
    tf::StampedTransform transform;
    try
    {
      
        transform_listener.lookupTransform(target_frame, source_frame, ros::Time(0), transform);
        camera_lidar_tf_ok_ = true;
        ROS_INFO("tf obtained");
    }
    catch (tf::TransformException ex)
    {
        ROS_INFO("joint_pixel_pointcloud : %s", ex.what());
    }
    return transform;
}
*/

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "robot_tf_listener");

    ros::NodeHandle nh;

    tf::TransformListener transform_listener;
    ros::Rate rate(10);

    while (ros::ok())
    {
       
            base_point.point.x=0;
            base_point.point.y=0;
            base_point.point.z=0;

            lidar_point.point.x=0;
            lidar_point.point.y=0;
            lidar_point.point.z=0;
        try
        {
            transform_listener.lookupTransform("camera","world",ros::Time(0),world_camera_tf);
            transform_listener.lookupTransform("lidar","world",ros::Time(0),world_lidar_tf);
           transform_listener.lookupTransform("lidar","camera",ros::Time(0),lidar_camera_tf);
        }
        catch (tf::TransformException ex)
        {
            ROS_INFO("joint_pixel_pointcloud : %s", ex.what());
        }

        tf::Vector3 tf_point(base_point.point.x, base_point.point.y, base_point.point.z);
        tf::Vector3 tf_point_transformed = lidar_camera_tf * tf_point;
        
        lidar_point.point.x = tf_point_transformed.x();
        lidar_point.point.y = tf_point_transformed.y();
        lidar_point.point.z = tf_point_transformed.z();
        
ROS_INFO("base: (%.2f, %.2f. %.2f) -----> : (%.2f, %.2f, %.2f) ",
        base_point.point.x, base_point.point.y, base_point.point.z,
        lidar_point.point.x, lidar_point.point.y, lidar_point.point.z);

        rate.sleep();
    }
    

 


    
    return 0;
}