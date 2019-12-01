#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>


class test_core
{
public:
    test_core(ros::NodeHandle &nh);
   ~test_core();

private:
    ros::Subscriber test_sub ;
    ros::Publisher test_pub ;
    void subCallback(const std_msgs::String::ConstPtr &msg);
    
};