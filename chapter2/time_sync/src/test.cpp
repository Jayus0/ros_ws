#include "ros/ros.h"
#include <thread>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>

ros::Publisher pub1;
ros::Publisher pub2;

void thread1();
void thread2();


int main(int argc, char** argv)
{
    ros::init(argc, argv, "time_sync_test");
    ros::NodeHandle nh;

    pub1 = nh.advertise<sensor_msgs::LaserScan>("/time_sync_scan", 1);
    pub2 = nh.advertise<sensor_msgs::Imu>("/time_sync_imu", 1);

    std::thread t1(thread1);
    std::thread t2(thread2);

    t1.join();
    t2.join();
    ros::spin();
    return 0;
}

void thread1()
{
    sensor_msgs::LaserScan msg;
    ros::Rate loop(1);
    int num = 0;
    while (ros::ok())
    {
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "base_link";

        pub1.publish(msg);
        loop.sleep();
    }
}

void thread2()
{
    sensor_msgs::Imu msg;
    ros::Rate loop(5);
    int num = 0;
    while (ros::ok())
    {
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "base_link";
        pub2.publish(msg);
        loop.sleep();

    }
}