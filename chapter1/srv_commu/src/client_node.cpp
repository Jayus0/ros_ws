#include"ros/ros.h"
#include "common_msg/AddInt.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "client_node");
    ros::NodeHandle nh;

    ros::ServiceClient add_client = nh.serviceClient<common_msg::AddInt>("/add_srv_test");

    int num1 = 0;
    int num2 = 0;
    ros::Rate loop(1);
    while (ros::ok())
    {
        ros::service::waitForService("add_srv_test");
        common_msg::AddInt addSrv;
        addSrv.request.num1 = num1;
        addSrv.request.num2 = num2;
        add_client.call(addSrv);

        num1++;
        num2++;

        ros::spinOnce();
        loop.sleep();
    }



    return 0;
}