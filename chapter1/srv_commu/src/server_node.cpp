#include "ros/ros.h"
#include "common_msg/AddInt.h"


bool srvTestCallback(common_msg::AddInt::Request &req,common_msg::AddInt::Response &res)
{
    int num1 = req.num1;
    int num2 = req.num2;
    res.sum = num1 + num2;

    ROS_INFO_STREAM(num1 << " + " << num2 <<" = " << res.sum);

    return true;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "server_node");
    ros::NodeHandle nh;

    ros::ServiceServer server = nh.advertiseService("/add_srv_test", srvTestCallback);

    ros::spin();

    return 0;
}