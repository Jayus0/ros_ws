#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QDebug>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/qheaderview.h>
#include <pluginlib/class_list_macros.h>

#include "../include/multi_navi/multi_nav_plugin.h"

PLUGINLIB_EXPORT_CLASS(multi_nav_plugin::MultiNavPlugin, rviz::Panel)

namespace multi_nav_plugin
{
    MultiNavPlugin::MultiNavPlugin(QWidget *parent): rviz::Panel(parent), nh_(), max_goal_(1), 
        cur_goal_idx_(0), cycle_cnt_(0), permit_(false), cycle_(false), arrived_(false)
    {
        // subscriber
        this->goal_sub_ = this->nh_.subscribe<geometry_msgs::PoseStamped>
                          ("move_base_simple/goal_temp", 1, boost::bind(&MultiNavPlugin::_onGoalUpdate, this, _1));
        this->status_sub_ = this->nh_.subscribe<actionlib_msgs::GoalStatusArray>
                          ("move_base/status", 1, boost::bind(&MultiNavPlugin::_onStatusUpdate, this, _1));
        // publisher
        this->goal_pub_ = this->nh_.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
        this->cancel_pub_ = this->nh_.advertise<actionlib_msgs::GoalID>("move_base/cancel", 1);
        this->marker_pub_ = this->nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);

        // main layout
        QVBoxLayout *root_layout = new QVBoxLayout;

        // max number of goal panel
        QHBoxLayout *max_goal_layout = new QHBoxLayout;
        max_goal_layout->addWidget(new QLabel("max goal number"));
        this->max_goal_editor_ = new QLineEdit;
        max_goal_layout->addWidget(this->max_goal_editor_);
        this->max_goal_btn_ = new QPushButton("OK");
        max_goal_layout->addWidget(this->max_goal_btn_);

        // cycles and pose table
        this->cycle_checkbox_ = new QCheckBox("cycle");
        this->pose_table_ = new QTableWidget;

        // manipulate layout
        QHBoxLayout *manipulate_layout = new QHBoxLayout;
        this->reset_btn_ = new QPushButton("reset");
        manipulate_layout->addWidget(this->reset_btn_);
        this->start_btn_ = new QPushButton("start");
        manipulate_layout->addWidget(this->start_btn_);
        this->cancel_btn_ = new QPushButton("cancle");
        manipulate_layout->addWidget(this->cancel_btn_);

        root_layout->addLayout(max_goal_layout);
        root_layout->addWidget(this->cycle_checkbox_);
        root_layout->addWidget(this->pose_table_);
        root_layout->addLayout(manipulate_layout);
        setLayout(root_layout);

        // set a Qtimer to start a spin for subscriptions
        QTimer *output_timer = new QTimer(this);
        output_timer->start(200);

        // connect the slot and signal
        connect(this->max_goal_btn_, SIGNAL(clicked()), this,SLOT(_updatePoseTable()));
        connect(this->reset_btn_, SIGNAL(clicked()), this, SLOT(initialize()));
        connect(this->start_btn_, SIGNAL(clicked()), this, SLOT(_startNavigation()));
        connect(this->cancel_btn_, SIGNAL(clicked()), this, SLOT(_cancelNavigation()));
        connect(this->cycle_checkbox_, SIGNAL(clicked(bool)), this, SLOT(_checkCycle()));
        connect(output_timer, SIGNAL(timeout()), this, SLOT(_startSpin()));
    }

    void MultiNavPlugin::initialize() 
    {
        ROS_INFO("Initialize Multi-Nav Pluggin!");
        this->cur_goal_idx_ = 0;
        this->cycle_cnt_ = 0;
        this->permit_ = false;
        this->cycle_ = false;
        this->pose_table_->clear();
        this->pose_array_.poses.clear();

        // delete mark
        visualization_msgs::Marker marker_delete;
        marker_delete.action = visualization_msgs::Marker::DELETEALL;
        this->marker_pub_.publish(marker_delete);

        this->_updatePoseTable();
        this->cycle_checkbox_->setCheckState(Qt::Unchecked);
    }

    void MultiNavPlugin::_startNavigation() 
    {
        this->cur_goal_idx_  %= this->pose_array_.poses.size();
        if (!this->pose_array_.poses.empty() && this->cur_goal_idx_ < this->max_goal_) 
        {
            geometry_msgs::PoseStamped goal;
            goal.header = this->pose_array_.header;
            goal.pose = this->pose_array_.poses.at(this->cur_goal_idx_);
            this->goal_pub_.publish(goal);
            ROS_INFO("Navigation to the Goal%d", this->cur_goal_idx_ + 1);
            this->pose_table_->item(this->cur_goal_idx_, 0)->setBackgroundColor(QColor(255, 69, 0));
            this->pose_table_->item(this->cur_goal_idx_, 1)->setBackgroundColor(QColor(255, 69, 0));
            this->pose_table_->item(this->cur_goal_idx_, 2)->setBackgroundColor(QColor(255, 69, 0));
            this->cur_goal_idx_ += 1;
            this->permit_ = true;
        } else 
            ROS_ERROR("Something Wrong");
    }

    void MultiNavPlugin::_cancelNavigation() 
    {
        if (!this->cur_goalid_.id.empty()) 
        {
            this->cancel_pub_.publish(this->cur_goalid_);
            ROS_INFO("Navigation has been canceled.");
        }
    }

    void MultiNavPlugin::_completeNavigation() {
        if (this->cur_goal_idx_ < this->pose_array_.poses.size()) {
            geometry_msgs::PoseStamped goal;
            goal.header = pose_array_.header;
            goal.pose = pose_array_.poses.at(this->cur_goal_idx_);
            this->goal_pub_.publish(goal);
            this->pose_table_->item(this->cur_goal_idx_, 0)->setBackgroundColor(QColor(255, 69, 0));
            this->pose_table_->item(this->cur_goal_idx_, 1)->setBackgroundColor(QColor(255, 69, 0));
            this->pose_table_->item(this->cur_goal_idx_, 2)->setBackgroundColor(QColor(255, 69, 0));
            this->cur_goal_idx_ += 1;
            this->permit_ = true;
        } else {
            ROS_ERROR("All goals are completed");
            this->permit_ = false;
        }
    }

    void MultiNavPlugin::_cycleNavigation() {
        if (this->permit_) {
            geometry_msgs::PoseStamped goal;
            goal.header = this->pose_array_.header;
            goal.pose = this->pose_array_.poses.at(this->cur_goal_idx_ % this->pose_array_.poses.size());
            this->goal_pub_.publish(goal);
            ROS_INFO("Navigation to the Goal %lu, in the %dth cycle", this->cur_goal_idx_ % this->pose_array_.poses.size() + 1,
                     this->cycle_cnt_ + 1);
            QColor color_table;
            if ((this->cycle_cnt_ + 1) % 2 != 0)
                color_table = QColor(255, 69, 0);
            else
                color_table = QColor(100, 149, 237);
            this->pose_table_->item(this->cur_goal_idx_ % this->pose_array_.poses.size(), 0)->setBackgroundColor(color_table);
            this->pose_table_->item(this->cur_goal_idx_ % this->pose_array_.poses.size(), 1)->setBackgroundColor(color_table);
            this->pose_table_->item(this->cur_goal_idx_ % this->pose_array_.poses.size(), 2)->setBackgroundColor(color_table);
            this->cur_goal_idx_ += 1;
            this->cycle_cnt_ = this->cur_goal_idx_ / this->pose_array_.poses.size();
        }
    }

    void MultiNavPlugin::_updatePoseTable() {
        QString max_goal_str = this->max_goal_editor_->text();
        this->max_goal_ = max_goal_str.toInt();
        this->pose_table_->setRowCount(this->max_goal_);
        this->pose_table_->setColumnCount(3);
        this->pose_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
        this->pose_table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
        QStringList pose_header;
        pose_header << "x" << "y" << "yaw";
        this->pose_table_->setHorizontalHeaderLabels(pose_header);
        this->pose_table_->show();
    }

    void MultiNavPlugin::_onGoalUpdate(const geometry_msgs::PoseStamped::ConstPtr &pose) {
        if (this->pose_array_.poses.size() < this->max_goal_) {
            // push new pose to pose array
            this->pose_array_.poses.push_back(pose->pose);
            this->pose_array_.header.frame_id = pose->header.frame_id;
            
            // update pose table
            this->pose_table_->setItem(this->pose_array_.poses.size() - 1, 0, new QTableWidgetItem
                                      (QString::number(pose->pose.position.x, 'f', 2)));
            this->pose_table_->setItem(this->pose_array_.poses.size() - 1, 1, new QTableWidgetItem
                                      (QString::number(pose->pose.position.y, 'f', 2)));
            this->pose_table_->setItem(this->pose_array_.poses.size() - 1, 2, new QTableWidgetItem
                                      (QString::number(tf::getYaw(pose->pose.orientation) * 180.0 / 3.14, 'f', 2)));

            // mark pose on the map
            if (ros::ok()) {
                visualization_msgs::Marker arrow;
                visualization_msgs::Marker number;
                arrow.header.frame_id = number.header.frame_id = pose->header.frame_id;
                arrow.ns = "navigation_point_arrow";
                number.ns = "navigation_point_number";
                arrow.action = number.action = visualization_msgs::Marker::ADD;
                arrow.type = visualization_msgs::Marker::ARROW;
                number.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                arrow.pose = number.pose = pose->pose;
                number.pose.position.z += 1.0;
                arrow.scale.x = 1.0;
                arrow.scale.y = 0.2;
                number.scale.z = 1.0;
                arrow.color.r = number.color.r = 1.0f;
                arrow.color.g = number.color.g = 0.98f;
                arrow.color.b = number.color.b = 0.80f;
                arrow.color.a = number.color.a = 1.0;
                arrow.id = number.id = pose_array_.poses.size();
                number.text = std::to_string(pose_array_.poses.size());
                this->marker_pub_.publish(arrow);
                this->marker_pub_.publish(number);
            }
        } else 
            ROS_ERROR("Beyond the maximum number of goals: %d", max_goal_);
    }

    void MultiNavPlugin::_onStatusUpdate(const actionlib_msgs::GoalStatusArray::ConstPtr &statuses) {
        bool arrived_pre = this->arrived_;

        // check goal status
        if (!statuses->status_list.empty()) {
            for (auto &i : statuses->status_list) {
                if (i.status == 3) {
                    ROS_INFO("Completed Goal: %d", this->cur_goal_idx_);
                    this->arrived_ = true;
                } else if (i.status == 4) {
                    ROS_ERROR("Goal %d is Invalid, Navigation to Next Goal %d", 
                            this->cur_goal_idx_, this->cur_goal_idx_ + 1);
                    this->arrived_ = true;
                } else if (i.status == 0) {
                    this->arrived_ = true;
                } else if (i.status == 1) {
                    this->cur_goalid_ = i.goal_id;
                    this->arrived_ = false;
                } else this->arrived_ = false;
            }
        } else this->arrived_ = false;

        if (this->arrived_ && !arrived_pre && ros::ok() && permit_) {
            if (cycle_)
                this->_cycleNavigation();
            else
                this->_completeNavigation();
        }
    }

    void MultiNavPlugin::_checkCycle() {
        this->cycle_ = this->cycle_checkbox_->isChecked();
    }

    void MultiNavPlugin::_startSpin() {
        if (ros::ok()) 
            ros::spinOnce();
    }

}