#ifndef REVERSE_DRIVE_H_
#define REVERSE_DRIVE_H_

#include "utils.hpp"
#include <thread>
#include <mutex>
#include <nav_msgs/Path.h>
#include "driverless/auto_drive_base.h"

class ReverseDrive : public AutoDriveBase 
{
public:

    ReverseDrive();
    virtual ~ReverseDrive();
	virtual bool init(ros::NodeHandle nh,ros::NodeHandle nh_private) override;
    virtual bool start() override;
    virtual void stop()  override;
    bool loadReversePath(const std::string& file, bool reverse);
    void setExpectSpeed(float speed) {exp_speed_ = speed;}
    bool reversePathPlan(const Pose& origin_pose,  const Pose& target_pose);
    void publishLocalPath();
    void reverseControlThread();
    void waitSpeedZero();
    void waitGearOk(int gear);

private:
    float exp_speed_;           //期望倒车速度
    float max_speed_;           //最大倒车速度
    Path reverse_path_;
    std::mutex working_mutex_;
    float preview_dis_;

    ros::Publisher pub_local_path_;
    
};

#endif
