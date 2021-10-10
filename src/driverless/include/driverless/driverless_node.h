
#include <ros/ros.h>
#include <memory>
#include "path_tracking.h"
#include "car_following.h"
#include "reverse_drive.h"
#include "extern_control/extern_control.h"
#include <pathplaning_msgs/expected_path.h>


#include "auto_drive_base.h"
#include <condition_variable>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <driverless_common/DoDriverlessTaskAction.h>   // Note: "Action" is appended
#include <driverless_common/SystemState.h>
#include <driverless_common/VehicleCtrlCmd.h>
#include <driverless_common/VehicleState.h>
#include <driverless_common/common.h>


class AutoDrive : public AutoDriveBase
{
public:
    typedef actionlib::SimpleActionClient<driverless_common::DoDriverlessTaskAction> DoDriverlessTaskClient;
    typedef actionlib::SimpleActionServer<driverless_common::DoDriverlessTaskAction> DoDriverlessTaskServer;

    AutoDrive();
    ~AutoDrive();
    virtual bool init(ros::NodeHandle nh,ros::NodeHandle nh_private) override;
    void run(){};

private:
    bool loadVehicleParams();
    bool loadDriveTaskFile(const std::string& points_file, const std::string &extend_file="");
    bool setDriveTaskPathPoints(const driverless_common::DoDriverlessTaskGoalConstPtr& goal);
    bool isGpsPointValid(const GpsPoint& point);

    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
    void sendCmd_CB(const ros::TimerEvent&);
    void vehicleStateSet_CB(const driverless_common::VehicleState::ConstPtr& msg);
    void captureExernCmd_callback(const ros::TimerEvent&);
    void publishDriverlessState(const ros::TimerEvent&);
    
    void setSendControlCmdEnable(bool flag);
    void goal_callback(const pathplaning_msgs::expected_path::ConstPtr& msg);
    void executeDriverlessCallback(const driverless_common::DoDriverlessTaskGoalConstPtr& goal);
    void goalPreemptCallback();
    bool handleNewGoal(const driverless_common::DoDriverlessTaskGoalConstPtr& goal, std::string &result);

    void resetVehicleCtrlCmd();

    const driverless_common::VehicleCtrlCmd decisionMaking(const controlCmd_t& tracker_cmd);

    bool isReverseGear();
    bool isDriveGear();
    bool isNeutralGear();

    void workingThread();
    void doDriveWork();
    void doReverseWork();
    void doOfflineDebugWork();

    bool waitGearOk(int gear);
    void waitSpeedZero();
    
    RecursiveMutex switchStateRMutex_;
    bool switchSystemState(int state);
    
    
private:
    float expect_speed_;
    bool  use_avoiding_;
    bool  use_car_following_;
    bool  is_offline_debug_;

    std::atomic<int> system_state_;
    int last_system_state_;
    std::atomic<bool> task_processing_;
    
    //工作线程条件变量
    bool has_new_task_;
    std::mutex work_cv_mutex_;
    std::condition_variable work_cv_;
    std::atomic<bool> task_running_;  //任务正在执行？
    
    //任务监听线程条件变量
    bool request_listen_;
    std::mutex listen_cv_mutex_;
    std::condition_variable listen_cv_;
    
    std::vector<ros::Timer> timers_;
	ros::Timer cmd_timer_;
    ros::Timer capture_extern_cmd_timer_;

    std::vector<ros::Subscriber> subscribers_;

    ros::Publisher pub_cmd_;
    ros::Publisher pub_driverless_state_;

    ros::Subscriber sub_new_goal_;   //订阅外部目标任务请求
    ros::Publisher  pub_new_goal_;   //发布目标请求到actionlib服务 
    
    std::mutex cmd_msg_mutex_;
    driverless_common::VehicleCtrlCmd vehicleCtrlCmd_;

    DoDriverlessTaskServer* as_;
    bool goal_preempt_;  //当前任务目标被中断
    
    float avoid_offset_;
    PathTracking tracker_;
    controlCmd_t tracker_cmd_;

    bool use_car_follower_;
    CarFollowing car_follower_;
    controlCmd_t follower_cmd_;
 
    bool use_extern_controller_;
    ExternControl extern_controler_;
    controlCmd_t  extern_cmd_;
    std::mutex extern_cmd_mutex_;

    ReverseDrive reverse_controler_;
    controlCmd_t  reverse_cmd_;

    //AvoidObstacle avoider_;
    controlCmd_t avoid_cmd_;
};

