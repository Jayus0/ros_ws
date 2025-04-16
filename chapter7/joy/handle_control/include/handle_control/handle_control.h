#ifndef __HANDLE_CONTROL_H__
#define __HANDLE_CONTROL_H__


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <thread>


class YLHandleControl
{
public:
    YLHandleControl();
    ~YLHandleControl();

public:
    void YLJoyCallBack(const sensor_msgs::Joy::ConstPtr &joy);
    void callbackTimer(const ros::TimerEvent&);

    void YLSpeedThread();

private:
    //v值范围限制在[lo, hi]
    template<typename T>
    T clamp(const T &v, const T &v1, const T &v2)
    {
        T lo = std::min(v1, v2);
        T hi = std::max(v1, v2);
        return (v < lo) ? lo : (v > hi) ? hi : v;
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber joy_sub;
    ros::Publisher twist_pub;
    ros::Publisher limit_vel_pub;
    ros::Timer timer;

    std::string twist_topic_name;
    int axis_linear;
    int axis_angular;
    int button_linear_accel;
    int button_linear_decel;
    int button_angular_accel;
    int button_angular_decel;

    geometry_msgs::Twist limit_vel;
    geometry_msgs::Twist pub_vel;

    double min_vel;
    double max_vel;

    bool is_key;
    
    std::thread speed_thread;
};



#endif