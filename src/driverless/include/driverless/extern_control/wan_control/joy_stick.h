#ifndef JOY_STICK_H_
#define JOY_STICK_H_

#include <sensor_msgs/Joy.h>
#include <ant_msgs/ControlCmd2.h>

enum JoyFunction
{
    //button
    button_angleGradeChange = 0,
    button_setDriverless = 1,
    button_setGear = 2 ,
    button_hand_brake = 3,
    button_speedRangeDec = 4,
    button_speedRangeAdd = 5,
    button_isManual = 8,
    //axes
    axes_setSpeed = 1,
    axes_leftOffset = 2,
    axes_steeringAngle = 3,
    axes_rightOffset = 5,
};

class JoyCmd
{
public:
    float max_speed;
    float max_steer_angle;
    int speed_grade_cnt; //速度级别数
    int steer_grade_cnt; //角度级别数

    int  speed_grade = 1;  //速度级别
    float speed_increment; //速度增量
    int  steer_grade = 1;  //角度级别
    float steer_increment; //角度增量

    bool  is_manual = false;

    bool set_hand_brake = false; //手刹,默认false
    uint8_t set_gear = ant_msgs::ControlCmd2::GEAR_NEUTRAL; //档位默认为N
    float   set_speed = 0.0;
    float   set_steer = 0.0;
    uint8_t  set_brake = 0.0;

    void display()
    {
        printf("v_grade:%d  a_grade:%d  v_inc:%2.2f  a_inc:%2.2f  v:%2.2f  a:%2.2f  gear:%d  brake:%d\r\n",
            speed_grade, steer_grade, steer_increment, speed_increment, set_speed, set_steer, set_gear, set_brake);
    }

};

static bool parseJoyMsgs(const sensor_msgs::Joy& joy_msg, JoyCmd& joy_cmd)
{
    if (joy_msg.buttons[button_isManual] == 1)
        joy_cmd.is_manual = ! joy_cmd.is_manual;
    if(!joy_cmd.is_manual)
        return false;

    if(joy_msg.buttons[button_hand_brake] == 1)  //手刹
        joy_cmd.set_hand_brake = !joy_cmd.set_hand_brake;
    
    if (joy_msg.buttons[button_setGear] == 1)      //档位切换
    {
        //I->D->N->R->I
        if(joy_cmd.set_gear == ant_msgs::ControlCmd2::GEAR_INITIAL)
            joy_cmd.set_gear = ant_msgs::ControlCmd2::GEAR_DRIVE;
        else if(joy_cmd.set_gear == ant_msgs::ControlCmd2::GEAR_DRIVE)
            joy_cmd.set_gear = ant_msgs::ControlCmd2::GEAR_NEUTRAL;
        else if(joy_cmd.set_gear == ant_msgs::ControlCmd2::GEAR_NEUTRAL) 
            joy_cmd.set_gear = ant_msgs::ControlCmd2::GEAR_REVERSE;
        else if(joy_cmd.set_gear == ant_msgs::ControlCmd2::GEAR_REVERSE)
            joy_cmd.set_gear = ant_msgs::ControlCmd2::GEAR_INITIAL;
    }

    //角度档位切换
    if (joy_msg.buttons[button_angleGradeChange] == 1)
    {
        ++joy_cmd.steer_grade;
        if(joy_cmd.steer_grade > joy_cmd.steer_grade_cnt)
            joy_cmd.steer_grade = 1;
    }

    joy_cmd.set_steer = joy_msg.axes[axes_steeringAngle] * joy_cmd.steer_grade * joy_cmd.steer_increment; 
    
    if(joy_msg.buttons[button_speedRangeAdd] == 1) //速度增档
    {
        if (++joy_cmd.speed_grade > joy_cmd.speed_grade_cnt) 
            joy_cmd.speed_grade = joy_cmd.speed_grade_cnt;
    }
    if(joy_msg.buttons[button_speedRangeDec] == 1) //速度减档
    {
        if (--joy_cmd.speed_grade < 1) 
            joy_cmd.speed_grade = 1;
    }

    if(joy_cmd.set_gear == ant_msgs::ControlCmd2::GEAR_DRIVE) //D档
        joy_cmd.set_speed = (joy_cmd.speed_grade-1)*joy_cmd.speed_increment + 
                            joy_msg.axes[axes_setSpeed] * joy_cmd.speed_increment;
    else if(joy_cmd.set_gear == ant_msgs::ControlCmd2::GEAR_REVERSE) //R档
        joy_cmd.set_speed = joy_msg.axes[axes_setSpeed] * 3.0; //max reverse speed 3.0km/h
    else
        joy_cmd.set_speed = 0.0;

    if(joy_cmd.set_speed < 0) joy_cmd.set_speed = 0;
    if(joy_msg.axes[axes_setSpeed] < -0.2)
        joy_cmd.set_brake = -100*joy_msg.axes[axes_setSpeed];
    else
        joy_cmd.set_brake = 0.0;
/*
    if(joy_msg.axes[axes_leftOffset] != 1)
        offsetVal = (joy_msg.axes[axes_leftOffset] - 1)*offsetMax_/2;
        
    else if(joy_msg.axes[axes_rightOffset] != 1)
            offsetVal = -(joy_msg.axes[axes_rightOffset] - 1)*offsetMax_/2;
    else
        offsetVal = 0.0;
*/
    return true;
}
#endif