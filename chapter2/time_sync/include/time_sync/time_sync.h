#ifndef __TIME_SYNC_H__
#define __TIME_SYNC_H__

#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class TimeSync
{
public:
    TimeSync();
    ~TimeSync();

private:
    void syncCallback(const sensor_msgs::LaserScan& msg1, const sensor_msgs::Imu& msg2);

private:
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::LaserScan> sub1;
    message_filters::Subscriber<sensor_msgs::Imu> sub2;

//     //定义近似时间同步策略
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::Imu> syncPolicy;
    boost::shared_ptr<message_filters::Synchronizer<syncPolicy>> sync_;


};





#endif