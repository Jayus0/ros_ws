#include "time_sync/time_sync.h"


TimeSync::TimeSync()
{
    sub1.subscribe(nh, "/time_sync_topic1", 1);
    sub2.subscribe(nh, "/time_sync_topic2", 1);

    syncPolicy policy(10);
    policy.setMaxIntervalDuration(ros::Duration(0.1));

    // //初始化同步器
    sync_.reset(new message_filters::Synchronizer<syncPolicy>(syncPolicy(10), sub1, sub2));
    sync_->registerCallback(&TimeSync::syncCallback, this);
}

TimeSync::~TimeSync()
{

}

void TimeSync::syncCallback(const sensor_msgs::LaserScan& msg1, const sensor_msgs::Imu& msg2)
{

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "time_sync");

    ros::spin();

    return 0;
}