#include "ros/ros.h"
#include "polygon_plugin/polyBase.h"
#include <pluginlib/class_loader.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "polygon_lab");
  //类加载器 -- 参数1:基类功能包名称 参数2:基类全限定名称
  pluginlib::ClassLoader<polyBase::Polygon> polyLoader("polygon_plugin", "polyBase::Polygon");

  try
  {
    //创建插件类实例 -- 参数:插件类全限定名称
    boost::shared_ptr<polyBase::Polygon> triangle = polyLoader.createInstance("polyPlugin::Triangle");
    triangle->initialize(10.0);

    boost::shared_ptr<polyBase::Polygon> square = polyLoader.createInstance("polyPlugin::Square");
    square->initialize(10.0);

    ROS_INFO("Triangle area: %.2f", triangle->calArea());
    ROS_INFO("Square area: %.2f", square->calArea());
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
  }
  ros::spin();
  return 0;
}
