
#include <ros/ros.h>
#include "path_tracking.h"
#include "car_following.h"
#include "extern_control.h"
#include "reverse_drive.h"

#include <ant_msgs/ControlCmd1.h>
#include <ant_msgs/ControlCmd2.h>
#include <ant_msgs/State1.h>  //gear
#include <ant_msgs/State3.h>  //
#include <ant_msgs/State2.h>  //speed
#include <ant_msgs/State4.h>  //steerAngle
#include "auto_drive_base.h"


class AutoDrive : public AutoDriveBase
{
public:
    AutoDrive();
    ~AutoDrive();
    virtual bool init(ros::NodeHandle nh,ros::NodeHandle nh_private) override;
    void run();

    enum State
    {
        State_Stop    = 0,  //停止
        State_Drive   = 1,  //前进
        State_Reverse = 2,  //后退
        State_Idle    = 3,  //空闲
    };

private:
    bool loadVehicleParams();
    bool loadTrackingTaskFile(const std::string& file);
	void publishPathTrackingState();
    bool isGpsPointValid(const GpsPoint& point);
    void vehicleSpeed_callback(const ant_msgs::State2::ConstPtr& msg);
    void vehicleState4_callback(const ant_msgs::State4::ConstPtr& msg);
    void vehicleState1_callback(const ant_msgs::State1::ConstPtr& msg);

    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
    void sendCmd1_callback(const ros::TimerEvent&);
	void sendCmd2_callback(const ros::TimerEvent&);
    void decisionMaking();

    bool isReverseGear();
    bool isDriveGear();
    bool isNeutralGear();

private: 
    float max_speed_;
    float max_roadwheelAngle_;
    bool  use_avoiding_;
    bool  use_car_following_;
    bool  is_offline_debug_;

    int system_state_;
    bool has_new_task_;

	ros::Timer cmd1_timer_, cmd2_timer_;
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_vehicleState1_;
	ros::Subscriber sub_vehicleState2_;
	ros::Subscriber sub_vehicleState4_;

    ros::Publisher pub_cmd1_, pub_cmd2_;
    
    std::mutex command_mutex_;

	ant_msgs::ControlCmd1 controlCmd1_;
	ant_msgs::ControlCmd2 controlCmd2_;
    
    float avoid_offset_;
    PathTracking tracker_;
    controlCmd_t tracker_cmd_;

    CarFollowing car_follower_;
    controlCmd_t follower_cmd_;
 
    ExternControl extern_controler_;
    controlCmd_t  extern_cmd_;

    ReverseDrive reverse_controler_;
    controlCmd_t  reverse_cmd_;
};
