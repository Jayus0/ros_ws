#include "ros/ros.h"
#include <common_msg/People.h>

class SubNode
{
public:
    SubNode();
    ~SubNode();

    void msgCallBack(const common_msg::)

private:
    ros::NodeHandle nh;
    ros::Subscriber msg_sub;

};