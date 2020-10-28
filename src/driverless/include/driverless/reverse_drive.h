#ifndef REVERSE_DRIVE_H_
#define REVERSE_DRIVE_H_

#include "utils.hpp"
#include <thread>
#include <mutex>
#include "driverless/auto_drive_base.h"
#include "driverless/DoReverseAction.h" // Note: "Action" is appended
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

class ReverseDrive : public AutoDriveBase 
{
public:
    typedef actionlib::SimpleActionClient<driverless::DoReverseAction> ActionlibClient;
    typedef actionlib::SimpleActionServer<driverless::DoReverseAction> ActionlibServer;

    ReverseDrive();
    virtual ~ReverseDrive();
	virtual bool init(ros::NodeHandle nh,ros::NodeHandle nh_private) override;
    virtual bool start() override;

    void stopCurrentWork();

    void executeCallback(const driverless::DoReverseGoalConstPtr& goal);
    bool reversePathPlan(const Pose& target_pose);
    void reverseControlThread();

private:
    float max_speed_;
    Path reverse_path_ ;
    ActionlibServer* as_;
    std::mutex working_mutex_;
};

#endif