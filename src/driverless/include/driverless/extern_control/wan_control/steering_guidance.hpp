#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <cmath>
#define PI 3.1415926

class SteeringGuidance
{
    private:   
        double length_b_;
        double length_l_;
        double length_offset_;
        double height_of_camera_;
        
        double cam_fx_;
        double cam_fy_;
        double cam_u0_;
        double cam_v0_;
        
        int image_width_;
        int image_height_;
        
        int num_points_;
        double min_distance_;
        double max_distance_;
    
        void draw_guidance_points(cv::Mat &img, std::vector<double> x_left, std::vector<double> y_left, 
                                  std::vector<double> x_right, std::vector<double> y_right)
        {
            int r = 0;
            int g = 255;
            int b = 0;
            
            int radius = 4;
            int thickness = -1;
            int lineType = 8;
            
            int uu, vv;
            
            for (int idx = 0; idx < num_points_; idx ++)
            {
                if (x_left[idx] == 0 && y_left[idx] == 0)
                {
                    continue;
                }
                
                uu = cam_fx_ * x_left[idx] / y_left[idx] + cam_u0_;
                vv = cam_fy_ * height_of_camera_ / y_left[idx] + cam_v0_;
                
                if (uu >= 0 && uu < image_width_ && vv >= 0 && vv < image_height_)
                {
                    cv::circle(img, cv::Point(uu, vv), radius, cv::Scalar(r, g, b), thickness, lineType);
                }
            }
            
            for (int idx = 0; idx < num_points_; idx ++)
            {
                if (x_right[idx] == 0 && y_right[idx] == 0)
                {
                    continue;
                }
                
                uu = cam_fx_ * x_right[idx] / y_right[idx] + cam_u0_;
                vv = cam_fy_ * height_of_camera_ / y_right[idx] + cam_v0_;
                
                if (uu >= 0 && uu < image_width_ && vv >= 0 && vv < image_height_)
                {
                    cv::circle(img, cv::Point(uu, vv), radius, cv::Scalar(r, g, b), thickness, lineType);
                }
            }
        }
    
        void compute_guidance_points(double angle_, std::vector<double> &x_left, std::vector<double> &y_left, 
                                     std::vector<double> &x_right, std::vector<double> &y_right)
        {
            double theta;
            double theta_min;
            double theta_max;
            double theta_interval;
            
            double st_radius;
            double st_x0;
            double st_y0;
            
            if (fabs(angle_) < 1)
            {
                for (int idx = 0; idx < num_points_; idx ++)
                {
                    x_left[idx] = - length_b_ / 2;
                    y_left[idx] = (max_distance_ - min_distance_) / num_points_ * idx;
                    
                    x_right[idx] = length_b_ / 2;
                    y_right[idx] = (max_distance_ - min_distance_) / num_points_ * idx;
                }
            }
            
            else if (angle_ > 0)
            {
                theta = fabs(angle_ * PI / 180);
                theta_min = 0 * PI / 180;
                theta_max = 90 * PI / 180;
                theta_interval = (theta_max - theta_min) / num_points_;
                
                st_x0 = - length_l_ / tan(theta) - length_b_ / 2;
                st_y0 = length_l_ - length_offset_;
                
                st_radius = length_l_ / tan(theta);
                for (int i = 0; i < num_points_; i ++)
                {
                    double theta_i = i * theta_interval + theta_min;
                    x_left[i] = st_x0 + st_radius * cos(theta_i);
                    y_left[i] = st_y0 + st_radius * sin(theta_i);
                }
                
                st_radius = length_l_ / tan(theta) + length_b_;
                for (int i = 0; i < num_points_; i ++)
                {
                    double theta_i = i * theta_interval + theta_min;
                    x_right[i] = st_x0 + st_radius * cos(theta_i);
                    y_right[i] = st_y0 + st_radius * sin(theta_i);
                }
            }
            
            else if (angle_ < 0)
            {
                theta = fabs(angle_ * PI / 180);
                theta_min = 90 * PI / 180;
                theta_max = 180 * PI / 180;
                theta_interval = (theta_max - theta_min) / num_points_;
                
                st_x0 = length_l_ / tan(theta) + length_b_ / 2;
                st_y0 = length_l_ - length_offset_;
                
                st_radius = length_l_ / tan(theta);
                for (int i = 0; i < num_points_; i ++)
                {
                    double theta_i = i * theta_interval + theta_min;
                    x_left[i] = st_x0 + st_radius * cos(theta_i);
                    y_left[i] = st_y0 + st_radius * sin(theta_i);
                }
                
                st_radius = length_l_ / tan(theta) + length_b_;
                for (int i = 0; i < num_points_; i ++)
                {
                    double theta_i = i * theta_interval + theta_min;
                    x_right[i] = st_x0 + st_radius * cos(theta_i);
                    y_right[i] = st_y0 + st_radius * sin(theta_i);
                }
            }

            for (int idx = 0; idx < num_points_; idx ++)
            {
                if (y_left[idx] < min_distance_ || y_left[idx] > max_distance_)
                {
                    x_left[idx] = 0;
                    y_left[idx] = 0;
                }
                if (y_right[idx] < min_distance_ || y_right[idx] > max_distance_)
                {
                    x_right[idx] = 0;
                    y_right[idx] = 0;
                }
            }
        }

public:
        void drawGuideCurveLines(cv::Mat& img, float steering_angle)
        {              
            std::vector<double> pts_x_left(num_points_);
            std::vector<double> pts_y_left(num_points_);
            std::vector<double> pts_x_right(num_points_);
            std::vector<double> pts_y_right(num_points_);
            compute_guidance_points(steering_angle, pts_x_left, pts_y_left, pts_x_right, pts_y_right);
            draw_guidance_points(img, pts_x_left, pts_y_left, pts_x_right, pts_y_right);
            
        }

        void drawGuideStraightLines(cv::Mat& image)
        {
            cv::line(image, cv::Point(240,312), cv::Point(344,183), cv::Scalar(0, 255, 0), 2); //lane left 0.5m
            cv::line(image, cv::Point(280,316), cv::Point(350,182), cv::Scalar(255, 255, 0), 3); // car left
            
            cv::line(image, cv::Point(456,313), cv::Point(377,181), cv::Scalar(255, 255, 0), 3); //car right
            cv::line(image, cv::Point(456,313), cv::Point(383,181), cv::Scalar(0, 255, 0), 2); //lane right 0.5m
    //					
    //	    cv::line(image, cv::Point(439,624), cv::Point(715,345), cv::Scalar(0, 255, 0), 3); //lane left 1.0m
    //					
            cv::line(image, cv::Point(320,241), cv::Point(320+15,241), cv::Scalar(0, 0, 255), 3);
            cv::line(image, cv::Point(337,209), cv::Point(337+17,209), cv::Scalar(0, 0, 255), 3);
            cv::line(image, cv::Point(343,196), cv::Point(343+19,196), cv::Scalar(0, 0, 255), 2);
            cv::line(image, cv::Point(347,189), cv::Point(347+21,189), cv::Scalar(0, 0, 255), 2);
            cv::line(image, cv::Point(350,185), cv::Point(350+23,185), cv::Scalar(0, 0, 255), 2);
            cv::line(image, cv::Point(352,181), cv::Point(352+25,181), cv::Scalar(0, 0, 255), 2);
        }

        ~SteeringGuidance(){}

        SteeringGuidance(ros::NodeHandle &nh)
        {
            ros::NodeHandle nh_private("steering_guidance");
            nh_private.param<double>("length_b", length_b_, 2.0);
            nh_private.param<double>("length_l", length_l_, 2.3);
            nh_private.param<double>("length_offset", length_offset_, 1.5);
            nh_private.param<double>("height_of_camera", height_of_camera_, 1.8);
            
            nh_private.param<double>("cam_fx", cam_fx_, 300);
            nh_private.param<double>("cam_fy", cam_fy_, 300);
            nh_private.param<double>("cam_u0", cam_u0_, 320);
            nh_private.param<double>("cam_v0", cam_v0_, 180);
            
            nh_private.param<int>("image_width", image_width_, 640);
            nh_private.param<int>("image_height", image_height_, 360);
            
            nh_private.param<int>("num_points", num_points_, 100);
            nh_private.param<double>("min_distance", min_distance_, 1);
            nh_private.param<double>("max_distance", max_distance_, 20);

        }
};








