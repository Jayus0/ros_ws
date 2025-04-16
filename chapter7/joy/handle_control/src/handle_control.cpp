#include "handle_control/handle_control.h"


YLHandleControl::YLHandleControl(): nh("~")
{
    nh.param<int>("axis_linear", axis_linear, 1);
    nh.param<int>("axis_angular", axis_angular, 0);
    nh.param<int>("button_linear_accel", button_linear_accel, 0);
    nh.param<int>("button_linear_decel", button_linear_decel, 1);
    nh.param<int>("button_angular_accel", button_angular_accel, 2);
    nh.param<int>("button_angular_decel", button_angular_decel, 3);

    nh.param<std::string>("twist_topic_name", twist_topic_name, "/cmd_vel");

    twist_pub = nh.advertise<geometry_msgs::Twist>(twist_topic_name, 1);
    joy_sub = nh.subscribe("/joy", 1, &YLHandleControl::YLJoyCallBack, this);
    limit_vel_pub = nh.advertise<geometry_msgs::Twist>("/handle/limit_vel", 1);
    timer = nh.createTimer(ros::Duration(1.0), &YLHandleControl::callbackTimer, this);

    limit_vel.linear.x = 0.70;
    limit_vel.angular.z = 0.70;

    min_vel = 0.00;
    max_vel = 1.00;

    is_key = false;

    ros::AsyncSpinner spinner(2);
	spinner.start();

    speed_thread = std::thread(&YLHandleControl::YLSpeedThread, this);
    speed_thread.detach();
}

YLHandleControl::~YLHandleControl()
{

}

void YLHandleControl::YLSpeedThread()
{
    ros::Rate loop(15);
    auto temp = ros::Time::now();
    while (ros::ok())
    {

        if (is_key){
            temp = ros::Time::now();
            twist_pub.publish(pub_vel);
        }
        else{
            if ((ros::Time::now() - temp).toSec() < 3.0)
            {
                pub_vel.angular.z = 0.0;
                pub_vel.linear.x  = 0.0;
                twist_pub.publish(pub_vel);
            }
        }

        loop.sleep();
    }
    
}

void YLHandleControl::YLJoyCallBack(const sensor_msgs::Joy::ConstPtr &joy)
{
    /**
     * pub vel
    */
   pub_vel.angular.z = 0.0;
   pub_vel.linear.x  = 0.0;
   pub_vel.linear.z = 11;

    if (joy->axes[0] != 0.0 or joy->axes[1] != 0.0 or joy->axes[6] != 0.0 or joy->axes[7] != 0.0 )
    {
        is_key = true;
        
        if (joy->axes[0] != 0.0)
            pub_vel.angular.z = clamp(static_cast<double>(joy->axes[0]), -1 * limit_vel.angular.z, limit_vel.angular.z);
        if (joy->axes[1] != 0.0)
            pub_vel.linear.x  = clamp(static_cast<double>(joy->axes[1]), -1 * limit_vel.linear.x, limit_vel.linear.x);
        if (joy->axes[6] != 0.0)
            pub_vel.angular.z = clamp(static_cast<double>(joy->axes[6]), -1 * limit_vel.angular.z, limit_vel.angular.z);
        if (joy->axes[7] != 0.0)
            pub_vel.linear.x  = clamp(static_cast<double>(joy->axes[7]), -1 * limit_vel.linear.x, limit_vel.linear.x);
        
    }else
        is_key = false;



    /**
     * Increase or decrease the speed limit
    */
   if (joy->buttons[button_linear_accel] == 1.0)
   {
        limit_vel.linear.x += 0.10;
        limit_vel.linear.x = clamp(limit_vel.linear.x, min_vel, max_vel);
   }
   else if (joy->buttons[button_linear_decel] == 1.0)
   {
        limit_vel.linear.x -= 0.10;
        limit_vel.linear.x = clamp(limit_vel.linear.x, min_vel, max_vel);
   }
   else if (joy->buttons[button_angular_accel] == 1.0)
   {
        limit_vel.angular.z += 0.10;
        limit_vel.angular.z = clamp(limit_vel.angular.z, min_vel, max_vel);
   }
   else if (joy->buttons[button_angular_decel] == 1.0)
   {
        limit_vel.angular.z -= 0.10;
        limit_vel.angular.z = clamp(limit_vel.angular.z, min_vel, max_vel);
   }
}

void YLHandleControl::callbackTimer(const ros::TimerEvent&)
{
    limit_vel_pub.publish(limit_vel);
}


int main(int argc, char** argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "handle_control");
    YLHandleControl hc;

    ros::spin();
    return 0;
}