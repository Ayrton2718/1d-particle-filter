#pragma once

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <cmath>
#include <float.h>
#include <random>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <lcmcl_msgs/msg/localization.hpp>
#include <lcmcl_msgs/msg/laser.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <std_msgs/msg/string.hpp>
#include <lc_map/lc_tf.hpp>
#include <lc_map/lc_map.hpp>


using namespace rclcpp;
using namespace std::chrono_literals;

class LcmclTfPublisher : public blackbox::BlackBoxNode
{
private:
    rclcpp::Subscription<lcmcl_msgs::msg::Localization>::SharedPtr   _est_pos_sub;

    std::shared_ptr<lc::Tf> _tf;

    lc::pos_t   _true_pos;


    void est_subscriber(const lcmcl_msgs::msg::Localization::SharedPtr msg)
    {
        lc::pos_t est_pos;
        est_pos.x = msg->abs_x;
        est_pos.y = msg->abs_y;
        est_pos.rad = msg->abs_rad;
        _tf->set_est(est_pos, msg->header.stamp);

        _true_pos.x = msg->abs_x;
        _true_pos.y = msg->abs_y;
        _true_pos.rad = msg->abs_rad;
        _tf->set_true(_true_pos, msg->header.stamp);
    }

public:
    LcmclTfPublisher(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : LcmclTfPublisher("", options){}
    LcmclTfPublisher(const std::string &name_space, const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) 
        : blackbox::BlackBoxNode(blackbox::debug_mode_t::RELEASE, "lcmcl_tf_publisher", name_space, options)
    {
        this->_tf = std::make_shared<lc::Tf>(this);
        _tf->enable_send_tf();

        this->_est_pos_sub = this->create_subscription<lcmcl_msgs::msg::Localization>("est_localization", rclcpp::QoS(1).reliable(), std::bind(&LcmclTfPublisher::est_subscriber, this, std::placeholders::_1));    }
};