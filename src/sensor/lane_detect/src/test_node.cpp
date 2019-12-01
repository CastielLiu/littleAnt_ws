#include "test_core.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "test");
    ros::NodeHandle nh;

    test_core core( nh );

    return 0;
}