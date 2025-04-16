#include "ros/ros.h"
#include "math_test_dll/math_test_dll.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "dll_test");
    // ros::NodeHandle nh;

    Calculator c;
    int num1 = 1;
    int num2 = 3;
    c.mAdd(num1, num2);

    ROS_INFO_STREAM(c.mAdd(num1, num2));

    // ros::spin();
    return 0;
}