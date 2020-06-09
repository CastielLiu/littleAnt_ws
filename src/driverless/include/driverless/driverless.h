
#include "ros/ros.h"
#include "path_tracking.h"

#include<little_ant_msgs/ControlCmd1.h>
#include<little_ant_msgs/ControlCmd2.h>
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

private:
	void publishDiagnostics(uint8_t level,const std::string& msg);
	void publishPathTrackingState();
    bool is_gps_data_valid(const gpsMsg_t& point);
    void vehicleSpeed_callback(const little_ant_msgs::State2::ConstPtr& msg);
    void vehicleState4_callback(const little_ant_msgs::State4::ConstPtr& msg);
    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
    void sendCmd1_callback(const ros::TimerEvent&);
	void sendCmd2_callback(const ros::TimerEvent&);
    void decisionMaking();

private: 
    std::vector<gpsMsg_t>  path_points_;

    float max_speed_;
    float max_roadwheelAngle_;

    bool vehicle_speed_status_;
	float vehicle_speed_;
    gpsMsg_t vehicle_pose_;
	float roadwheel_angle_;
    
    std::string path_points_file_;

	ros::NodeHandle nh_, nh_private_;
	ros::Timer cmd1_timer_, cmd2_timer_;
    ros::Subscriber sub_odom_;
	ros::Subscriber sub_vehicleState2_;
	ros::Subscriber sub_vehicleState4_;

	ros::Publisher pub_diagnostic_;
    ros::Publisher pub_cmd1_, pub_cmd2_;
    
    std::mutex command_mutex_;
	little_ant_msgs::ControlCmd1 controlCmd1_;
	little_ant_msgs::ControlCmd2 controlCmd2_;
	diagnostic_msgs::DiagnosticStatus diagnostic_msg_;
    
    float avoid_offset_;
    PathTracking tracker_;
    controlCmd_t tracker_cmd_;
};