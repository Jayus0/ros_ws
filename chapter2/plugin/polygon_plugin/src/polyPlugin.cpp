#include <pluginlib/class_list_macros.h>
#include "polygon_plugin/polyBase.h"
#include "polygon_plugin/polyPlugin.h"


//参数1:插件类 参数2:基类
PLUGINLIB_EXPORT_CLASS(polyPlugin::Triangle, polyBase::Polygon)
PLUGINLIB_EXPORT_CLASS(polyPlugin::Square, polyBase::Polygon)

