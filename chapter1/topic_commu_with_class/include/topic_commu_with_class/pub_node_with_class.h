#include "ros/ros.h"
#include <common_msg/People.h>

class PubNode
{
public:
    PubNode();
    ~PubNode();

    void callbackTimer(const ros::TimerEvent&);

private:
    ros::NodeHandle nh;
    ros::Timer timer;
    ros::Publisher msg_pub;

};
