
#include "ros/ros.h"
#include "path_tracking.h"

#include<little_ant_msgs/ControlCmd.h>
#include<little_ant_msgs/State2.h>  //speed
#include<little_ant_msgs/State4.h>  //steerAngle
#include<diagnostic_msgs/DiagnosticStatus.h>

class AutoDrive 
{
public:
    AutoDrive();
    ~AutoDrive();
    bool init();
    void run();

    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
    void timer_callback(const ros::TimerEvent&);

private:
	void publishDiagnostics(uint8_t level,const std::string& msg);
	void publishPathTrackingState();

private: 
    std::vector<gpsMsg_t>  path_points_;


    float max_speed_;
	float vehicle_speed_;
	float roadwheel_angle_;
	float max_roadwheelAngle_;
    gpsMsg_t vehicle_pose_;
    std::string path_points_file_;

	ros::NodeHandle nh_, nh_private_;
	ros::Timer timer_;
    ros::Subscriber sub_odom_;
	ros::Subscriber sub_vehicleState2_;
	ros::Subscriber sub_vehicleState4_;

	ros::Publisher pub_diagnostic_;

    
	little_ant_msgs::ControlCmd1 controlCmd1_;
	little_ant_msgs::ControlCmd2 controlCmd2_;
	diagnostic_msgs::DiagnosticStatus diagnostic_msg_;
    
    float avoid_offset_;
    PathTracking tracker_;

};