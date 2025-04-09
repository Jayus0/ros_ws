 #include "ros/ros.h"
 #include <std_msgs/String.h>

 void msgCallBack(const std_msgs::StringConstPtr& msg);

 int main(int argc, char** argv)
 {
    ros::init(argc, argv, "sub_node");
    ros::NodeHandle nh;

    ros::Subscriber msg_sub = nh.subscribe("/hello_pub", 10, msgCallBack);

    ros::spin();

    return 0;
 }

  void msgCallBack(const std_msgs::StringConstPtr& msg)
  {
    ROS_INFO_STREAM("msg: "<<msg->data);
  }