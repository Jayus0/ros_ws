#include "ros/ros.h"
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

void transformPoint(const tf::TransformListener& listener)
{
    geometry_msgs::PointStamped laser_point;
    laser_point.header.frame_id = "base_laser";
    laser_point.header.stamp = ros::Time();
    laser_point.point.x = 1.0;
    laser_point.point.y = 2.0;
    laser_point.point.z = 0.0;

    geometry_msgs::PointStamped base_point;
    listener.transformPoint("base_link", laser_point, base_point);
    
    ROS_INFO_STREAM("base_laser: { "<<laser_point.point.x<<", "<<laser_point.point.y<<", "<<laser_point.point.z
    <<"} ------> base_link: { "<<base_point.point.x<<", "<<base_point.point.y<<", "<<base_point.point.z<<"}  at time "<<base_point.header.stamp.toSec());
}




int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_tf_listener");
    ros::NodeHandle nh;

    tf::TransformListener listener(ros::Duration(10));
    
    ros::Timer timer = nh.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener)));
    
    ros::spin();
    return 0;
}