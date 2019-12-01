#include "test_core.h"


test_core::test_core(ros::NodeHandle &nh)
{
    test_sub = nh.subscribe("/test_sub", 5, &test_core::subCallback,this);

 
    
    test_pub = nh.advertise<std_msgs::String>("/test_pub", 1000);

    ros::spin();
}

test_core::~test_core(){};

void test_core::subCallback(const std_msgs::String::ConstPtr &msg)
{
    
    ROS_INFO("i heard %s", msg->data.c_str() );

}
