#ifndef JOY_STICK_H_
#define JOY_STICK_H_

#include <sensor_msgs/Joy.h>
#include <ant_msgs/ControlCmd2.h>



enum JoyFunction
{
    //button
    button_handBrake = 0,
    button_setDriverless = 1,
    button_setGear = 2 ,
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
    float speed_grade_cnt; //速度级别数
    float steer_grade_cnt; //角度级别数

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
#endif