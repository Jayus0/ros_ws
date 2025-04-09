#include <topic_commu_with_class/sub_node_with_class.h>


SubNode::SubNode()
{

}

SubNode::~SubNode()
{
    
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "sub_node_c");

    ros::spin();
    return 0;
}