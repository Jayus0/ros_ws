#include <topic_commu_with_class/pub_node_with_class.h>


PubNode::PubNode()
{
    msg_pub = nh.advertise<common_msg::People>("/hello_people", 1, this);
    timer = nh.createTimer(ros::Duration(1.0), &PubNode::callbackTimer, this);
}

PubNode::~PubNode()
{

}

void PubNode::callbackTimer(const ros::TimerEvent&)
{
    common_msg::People msg;
    static int a = 0;

    msg.name = "Tom";
    msg.age = a;

    msg_pub.publish(msg);

    a++;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pub_node_c");
    PubNode p;
    ros::spin();
    return 0;
}