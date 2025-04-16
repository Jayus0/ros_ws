#ifndef __MULTI_NAV_PLUGIN_H__
#define __MULTI_NAV_PLUGIN_H__

#include "ros/ros.h"
#include <rviz/panel.h>

#include <QPushButton>
#include <QTableWidget>
#include <QCheckBox>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <actionlib_msgs/GoalStatus.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <tf/transform_datatypes.h>


namespace multi_nav_plugin
{
    class MultiNavPlugin : public rviz::Panel
    {
        Q_OBJECT
        public:
            MultiNavPlugin(QWidget *parent = 0);

        public Q_SLOTS:
            void initialize();

        protected Q_SLOTS:
            void _startNavigation();

            void _cancelNavigation();

            void _completeNavigation();

            void _cycleNavigation();

            void _updatePoseTable();

            void _onGoalUpdate(const geometry_msgs::PoseStamped::ConstPtr &pose);

            void _onStatusUpdate(const actionlib_msgs::GoalStatusArray::ConstPtr &statuses);

            void _checkCycle();

            static void _startSpin();

        protected:
            // max number of goal
            QLineEdit *max_goal_editor_;
            // functional button
            QPushButton *max_goal_btn_, *reset_btn_, *start_btn_, *cancel_btn_;
            // pose table
            QTableWidget *pose_table_;
            // whether start cycle navigaiton(1->2->3->...->1) or not
            QCheckBox *cycle_checkbox_;

            // The ROS node handle.
            ros::NodeHandle nh_;
            // goal publisher
            ros::Publisher goal_pub_;
            // goal cancle publisher
            ros::Publisher cancel_pub_;
            // map marker publisher
            ros::Publisher marker_pub_;
            // goal and status subscriber, listening Rviz setting
            ros::Subscriber goal_sub_, status_sub_;

            // max number of goal to be set
            int max_goal_;
            //  current navigation goal index
            int cur_goal_idx_;
            actionlib_msgs::GoalID cur_goalid_;
            // the number of cycles
            int cycle_cnt_;
            // whether allow to start navigation
            bool permit_;
            // whether cycle navigation or not
            bool cycle_;
            // whether arrive the goal or not
            bool arrived_;
            // pose array, the size is equal to `max_goal`
            geometry_msgs::PoseArray pose_array_;

    };
};


#endif