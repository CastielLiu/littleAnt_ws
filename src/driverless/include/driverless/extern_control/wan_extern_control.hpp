#ifndef WAN_EXTERN_CONTROL_H_
#define WAN_EXTERN_CONTROL_H_

#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <ant_msgs/ControlCmd2.h>
#include "extern_control_base.hpp"
#include <ant_msgs/State.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "joy_stick.h"

#ifndef PACK
#define PACK( __Declaration__ ) __Declaration__ __attribute__((__packed__))
#endif

PACK(
typedef struct 
{
	uint8_t act_gear :4;
	uint8_t driverless_mode :1;
	uint8_t hand_brake :1;
	uint8_t emergency_brake :1;
	uint8_t car_state :1;
	uint16_t speed;
	uint16_t roadwheelAngle;
	
	uint8_t is_manual :1;
	uint8_t speed_grade :3;
    uint8_t steer_grade :2;
}) StateFeedback_t;

class WanExternControl : public ExternControlBase
{
private:
	std::string image_topic_;
	ros::Subscriber sub_image_;
	ros::Subscriber sub_vehicle_state_;
	ros::Publisher pub_reload_seq_;
	ros::Publisher pub_joy_;
	ros::Timer timer_;
	
	struct sockaddr_in sockaddr_;
	std::string socket_ip_;
	int socket_port_;
	int udp_fd_;
	std::string connect_code_;
    std::mutex udp_fd_mutex_;
    std::mutex working_mutex_;

	int image_cut_h_;
	int image_quality_;
    JoyCmd joy_cmd_;
    std::mutex joy_cmd_mutex_;

public:
    WanExternControl():
        ExternControlBase("WanControler"),
        connect_code_("move0")
    {
        udp_fd_ = -1; 
    }

    ~WanExternControl()
    {
    }

    virtual bool init(ros::NodeHandle nh,ros::NodeHandle nh_private) override
    {
        image_topic_ = nh_private.param<std::string>("wan_control/image", "/image_raw");
        socket_ip_   = nh_private.param<std::string>("wan_control/server_ip","");
        socket_port_ = nh_private.param<int>("wan_control/sever_port",-1);
        image_cut_h_ = nh_private.param<int>("wan_control/image_cut_h",0);
        image_quality_ = nh_private.param<int>("wan_control/image_quality",50);
        joy_cmd_.max_steer_angle = nh_private.param<float>("vehicle/max_roadwheel_angle",25.0);
        joy_cmd_.max_speed  = nh_private.param<float>("vehicle/max_speed",40.0);
        joy_cmd_.steer_grade_cnt = 2;
        joy_cmd_.speed_grade_cnt = 6;
        joy_cmd_.steer_grade = joy_cmd_.speed_grade = 1;
        joy_cmd_.speed_increment = joy_cmd_.max_speed/joy_cmd_.speed_grade_cnt;
        joy_cmd_.steer_increment = joy_cmd_.max_steer_angle/joy_cmd_.steer_grade_cnt;

        ROS_INFO("[%s] ip:%s, port:%d", name_.c_str(), socket_ip_.c_str(),socket_port_);
        
        if(socket_ip_.empty() || socket_port_==-1)
        {
            ROS_ERROR("[%s] Please config the socket ip and port!", name_.c_str());
            return false;
        }

        if(!initSocket()) return false;
        if(!connectToServer()) return false;
            
        sub_image_ = nh.subscribe(image_topic_, 1, &WanExternControl::imageCallback, this);
        sub_vehicle_state_ = nh.subscribe("/vehicleState",1,&WanExternControl::vehicleStateCallback, this);
        pub_reload_seq_ = nh.advertise<std_msgs::UInt8>("/reload_seq",1);

        is_initialed_ = true;
        return true;
    }

    virtual bool start() override
    {
        if(!is_initialed_)
        {
            ROS_ERROR("[%s] Please inital before start!", name_.c_str());
            return false;
        }
        is_running_ = true;
        std::thread t = std::thread(std::bind(&WanExternControl::recvThread,this));
        t.detach();

        return true;
    }

    virtual void stop() override
    {
        is_running_ = false;
        std::lock_guard<std::mutex> lock(working_mutex_); //等待获得锁
        closeSocket();
    }

private:
    bool initSocket()
    {
        bzero(&sockaddr_,sizeof(sockaddr_));//init 0

        sockaddr_.sin_port = htons(socket_port_);
        sockaddr_.sin_family = AF_INET;
        int convert_ret = inet_pton(AF_INET, socket_ip_.c_str(), &sockaddr_.sin_addr);
        if(convert_ret !=1)
        {
            ROS_ERROR("[%s] Convert socket ip failed, please check the format!", name_.c_str());
            return false;
        }
        
        //UDP
        udp_fd_ = socket(PF_INET,SOCK_DGRAM , 0);
        if(udp_fd_ < 0)
        {
            ROS_ERROR("[%s] Build socket error!", name_.c_str());
            return false;
        }
        
        // 配置超时
        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 200000;
        if (setsockopt(udp_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) == -1)
        {
            ROS_ERROR("[%s] setsockopt failed !!!", name_.c_str());
            return false;
        }
        return true;
    }

    bool connectToServer()
    {
        if(udp_fd_ <0)
        {
            ROS_ERROR("[%s] Please initial socket before call connectToServer.", name_.c_str());
            return false;
        }

        int try_cnt = 5;
        while(ros::ok() && (try_cnt--) > 0)
        {
            int send_ret   = sendto(udp_fd_, connect_code_.c_str(), connect_code_.length(),0, 
                            (struct sockaddr*)&sockaddr_, sizeof(sockaddr_));
            ROS_INFO("[%s] send: %s -> try to connect to server",  name_.c_str(), connect_code_.c_str());
            if(send_ret <= 0)
                continue;
            
            char ans[21];
            socklen_t clientLen = sizeof(sockaddr_);
            int len = recvfrom(udp_fd_, ans, 20,0,(struct sockaddr*)&sockaddr_, &clientLen);
            if(len > 0 && std::string(ans) == (connect_code_ + "ok"))
            {
                ROS_INFO("[%s] connect server ok.", name_.c_str());
                return true;
            }
            
            ros::Duration(0.2).sleep();
        }
        ROS_ERROR("[%s] Connect to server failed!", name_.c_str());
        return false;
    }

    void recvThread()
    {
        std::lock_guard<std::mutex> lock(working_mutex_);
        const int BufLen = 200;
        uint8_t *recvbuf = new uint8_t [BufLen+1];
        char msg_type[6]; msg_type[5]='\0';
        socklen_t clientLen = sizeof(sockaddr_);
        while(ros::ok() && is_running_)
        {
            int len = recvfrom(udp_fd_, recvbuf, BufLen,0,(struct sockaddr*)&sockaddr_, &clientLen);
            if(len < 0) //reconnect
            {
                std::lock_guard<std::mutex> lock(udp_fd_mutex_);
                sendto(udp_fd_, connect_code_.c_str(), connect_code_.length(),0, (struct sockaddr*)&sockaddr_, sizeof(sockaddr_));
                continue;
            }
            else if(len < 5)
                continue;
                
            memcpy(msg_type,recvbuf,5);
            const std::string type(msg_type);
            if(type == "joy00")
            {
                static sensor_msgs::Joy last_joy_msg;
                sensor_msgs::Joy joy_msg;
                int axes_cnt = recvbuf[5];
                int buttons_cnt = recvbuf[6];
                int data_len = axes_cnt*4+buttons_cnt;
                //std::cout << "received joy00\r\n" ;
                if(generateCheckValue(recvbuf+7, data_len) != recvbuf[data_len+7])
                    continue;
                
                std::vector<float> axes((float*)(recvbuf+7),(float*)(recvbuf+7)+axes_cnt);
                joy_msg.axes.swap(axes);
                joy_msg.buttons.resize(buttons_cnt);
                for(size_t i=0; i<buttons_cnt; ++i)
                    joy_msg.buttons[i] = recvbuf[7+axes_cnt*4+i];
                
                //if(joy_msg.axes != last_joy_msg.axes || joy_msg.buttons != last_joy_msg.buttons)
                {
                    if(parseJoyMsgs(joy_msg))
                    {
                        cmd_mutex_.lock();
                        cmd_.validity = true;
                        cmd_.speed    = joy_cmd_.set_speed;
                        cmd_.brake    = joy_cmd_.set_brake;
                        cmd_.gear     = joy_cmd_.set_gear;
                        cmd_.roadWheelAngle = joy_cmd_.set_steer;
                        cmd_.hand_brake = joy_cmd_.set_hand_brake;
                        cmd_mutex_.unlock();
                    }
                    else
                    {
                        cmd_mutex_.lock();
                        cmd_.validity = false;
                        cmd_mutex_.unlock();
                    }
                        
                    joy_cmd_.display();

                    /*
                    pub_joy_.publish(joy_msg);
                    if(joy_msg.buttons.size())
                    {
                        std_msgs::UInt8 reload_seq;
                        reload_seq.data = joy_msg.buttons[joy_msg.buttons.size()-1];
                        if(reload_seq.data)
                            pub_reload_seq_.publish(reload_seq);
                    }
                    */
                }
                    
                last_joy_msg = joy_msg;
            }
        }
        delete [] recvbuf;
    }

    bool parseJoyMsgs(const sensor_msgs::Joy& joy_msg)
    {
        std::lock_guard<std::mutex> lck(joy_cmd_mutex_);
        if (joy_msg.buttons[button_isManual] == 1)
            joy_cmd_.is_manual = ! joy_cmd_.is_manual;
        if(!joy_cmd_.is_manual)
            return false;

        if(joy_msg.buttons[button_hand_brake] == 1)  //手刹
            joy_cmd_.set_hand_brake = !joy_cmd_.set_hand_brake;
        
        if (joy_msg.buttons[button_setGear] == 1)      //档位切换
        {
            //I->D->N->R->I
            if(joy_cmd_.set_gear == ant_msgs::ControlCmd2::GEAR_INITIAL)
                joy_cmd_.set_gear = ant_msgs::ControlCmd2::GEAR_DRIVE;
            else if(joy_cmd_.set_gear == ant_msgs::ControlCmd2::GEAR_DRIVE)
                joy_cmd_.set_gear = ant_msgs::ControlCmd2::GEAR_NEUTRAL;
            else if(joy_cmd_.set_gear == ant_msgs::ControlCmd2::GEAR_NEUTRAL) 
                joy_cmd_.set_gear = ant_msgs::ControlCmd2::GEAR_REVERSE;
            else if(joy_cmd_.set_gear == ant_msgs::ControlCmd2::GEAR_REVERSE)
                joy_cmd_.set_gear = ant_msgs::ControlCmd2::GEAR_INITIAL;
        }

        //角度档位切换
        if (joy_msg.buttons[button_angleGradeChange] == 1)
        {
            ++joy_cmd_.steer_grade;
            if(joy_cmd_.steer_grade > joy_cmd_.steer_grade_cnt)
                joy_cmd_.steer_grade = 1;
        }

        joy_cmd_.set_steer = joy_msg.axes[axes_steeringAngle] * joy_cmd_.steer_grade * joy_cmd_.steer_increment; 
        
        if(joy_msg.buttons[button_speedRangeAdd] == 1) //速度增档
        {
            if (++joy_cmd_.speed_grade > joy_cmd_.speed_grade_cnt) 
                joy_cmd_.speed_grade = joy_cmd_.speed_grade_cnt;
        }
        if(joy_msg.buttons[button_speedRangeDec] == 1) //速度减档
        {
            if (--joy_cmd_.speed_grade < 1) 
                joy_cmd_.speed_grade = 1;
        }

        if(joy_cmd_.set_gear == ant_msgs::ControlCmd2::GEAR_DRIVE) //D档
            joy_cmd_.set_speed = (joy_cmd_.speed_grade-1)*joy_cmd_.speed_increment + 
                                joy_msg.axes[axes_setSpeed] * joy_cmd_.speed_increment;
        else if(joy_cmd_.set_gear == ant_msgs::ControlCmd2::GEAR_REVERSE) //R档
            joy_cmd_.set_speed = joy_msg.axes[axes_setSpeed] * 3.0; //max reverse speed 3.0km/h
        else
            joy_cmd_.set_speed = 0.0;

        if(joy_cmd_.set_speed < 0) joy_cmd_.set_speed = 0;
        if(joy_msg.axes[axes_setSpeed] < -0.2)
            joy_cmd_.set_brake = -100*joy_msg.axes[axes_setSpeed];
        else
            joy_cmd_.set_brake = 0.0;
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

    void vehicleStateCallback(const ant_msgs::State::ConstPtr &msg)
    {
        if(!is_running_) return ;

        static uint8_t * vehicle_state_buf_ = nullptr;
        if(vehicle_state_buf_ == nullptr)
        {
            vehicle_state_buf_ = new uint8_t[14];
            strcpy((char*)vehicle_state_buf_, "state");
        }

        StateFeedback_t state;
        state.act_gear = msg->act_gear;
        state.driverless_mode = msg->driverless_mode;
        state.hand_brake = msg->hand_brake;
        state.emergency_brake = msg->emergency_brake;
        state.car_state = msg->vehicle_ready;
        state.speed = msg->vehicle_speed * 3.6 *100;
        state.roadwheelAngle = msg->roadwheelAngle *100 + 5000;

        joy_cmd_mutex_.lock();
        state.is_manual = joy_cmd_.is_manual;
        state.speed_grade = joy_cmd_.speed_grade;
        state.steer_grade = joy_cmd_.steer_grade;
        joy_cmd_mutex_.unlock();
        

        memcpy(vehicle_state_buf_+5, &state ,sizeof(state));
        vehicle_state_buf_[13] = generateCheckValue(vehicle_state_buf_+5, 8);
        
        std::lock_guard<std::mutex> lock(udp_fd_mutex_);
        sendto(udp_fd_, vehicle_state_buf_, 14,
                0, (struct sockaddr*)&sockaddr_, sizeof(sockaddr_));

    //	std::cout << int(state.act_gear) << "\t"
    //			  << int(state.driverless_mode) << "\t"
    //			  << int(state.hand_brake) << "\t"
    //			  << int(state.emergency_brake) << "\t"
    //			  << int(state.car_state) << "\t"
    //			  << state.speed*0.01 << "km/h\t"
    //			  << (state.roadwheelAngle-5000)*0.01 << "deg\n";
    }

    void imageCallback(const sensor_msgs::Image::ConstPtr &msg)
    {
        if(!is_running_) return ;

        cv_bridge::CvImageConstPtr cv_image = cv_bridge::toCvShare(msg, "bgr8");

        cv::line(cv_image->image, cv::Point(240,312), cv::Point(344,183), cv::Scalar(0, 255, 0), 2); //lane left 0.5m
        cv::line(cv_image->image, cv::Point(280,316), cv::Point(350,182), cv::Scalar(255, 255, 0), 3); // car left
        
        cv::line(cv_image->image, cv::Point(456,313), cv::Point(377,181), cv::Scalar(255, 255, 0), 3); //car right
        cv::line(cv_image->image, cv::Point(456,313), cv::Point(383,181), cv::Scalar(0, 255, 0), 2); //lane right 0.5m
//					
//	    cv::line(cv_image->image, cv::Point(439,624), cv::Point(715,345), cv::Scalar(0, 255, 0), 3); //lane left 1.0m
//					
        cv::line(cv_image->image, cv::Point(320,241), cv::Point(320+15,241), cv::Scalar(0, 0, 255), 3);
        cv::line(cv_image->image, cv::Point(337,209), cv::Point(337+17,209), cv::Scalar(0, 0, 255), 3);
        cv::line(cv_image->image, cv::Point(343,196), cv::Point(343+19,196), cv::Scalar(0, 0, 255), 2);
        cv::line(cv_image->image, cv::Point(347,189), cv::Point(347+21,189), cv::Scalar(0, 0, 255), 2);
        cv::line(cv_image->image, cv::Point(350,185), cv::Point(350+23,185), cv::Scalar(0, 0, 255), 2);
        cv::line(cv_image->image, cv::Point(352,181), cv::Point(352+25,181), cv::Scalar(0, 0, 255), 2);

        cv::Size size = cv_image->image.size();
        //cv::Mat dstImage;
        //cv::resize(cv_image->image,dstImage, size);
        
        cv::Rect rect(0,image_cut_h_,size.width ,size.height-image_cut_h_);
        std::vector<uint8_t> image_data;
        std::vector<int> param= {cv::IMWRITE_JPEG_QUALITY, image_quality_};
        cv::imencode(".jpg", cv_image->image(rect), image_data, param);
        
        std::lock_guard<std::mutex> lock(udp_fd_mutex_);
        int send_ret   = sendto(udp_fd_, image_data.data(), image_data.size(),
                                0, (struct sockaddr*)&sockaddr_, sizeof(sockaddr_));
        if(send_ret < 0)
            printf("send image to server failed!");
    }

    void closeSocket()
    {
        if(udp_fd_ != -1)
            close(udp_fd_);
    }

    uint8_t generateCheckValue(const uint8_t* buf,int len)
    {
        uint8_t result = 0;
        for(int i=0; i<len; ++i)
            result += buf[i];
        return result;
    }
};

#endif