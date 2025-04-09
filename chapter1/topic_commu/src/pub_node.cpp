#include "ros/ros.h"
#include <std_msgs/String.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pub_node");
    ros::NodeHandle nh;
    
    ros::Publisher msg_pub = nh.advertise<std_msgs::String>("hello_pub", 1);

    std_msgs::String msg;
    msg.data = "hello ros";

    ros::Rate loop(1);
    while (ros::ok())
    {
        msg_pub.publish(msg);
        loop.sleep();
        ros::spinOnce();
    }

    return 0;
}