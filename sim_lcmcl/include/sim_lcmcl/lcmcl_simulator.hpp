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
#include <tut_tool/tut_tool.hpp>

#include "sim/sim_odometry.hpp"
#include "sim/sim_sick.hpp"
#include <lc_map/lc_tf.hpp>

#include <lcmcl_topics/lcmcl_topics.hpp>

#define MY_JOY

using namespace rclcpp;
using namespace std::chrono_literals;

class LcmclSimulator : public blackbox::BlackBoxNode
{
private:
    sim::Odometry   _odometry;
    sim::Sick       _sick;
    
    std::shared_ptr<lc::Map>     _map;
    std::shared_ptr<lc::Tf>     _tf;

    rclcpp::Publisher<lcmcl_msgs::msg::Laser>::SharedPtr            _laser_pub;
    rclcpp::Publisher<lcmcl_msgs::msg::Odometry>::SharedPtr         _odom_pub;

    rclcpp::TimerBase::SharedPtr                                    _odom_pub_tim;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr          _joy_sub;
    rclcpp::Publisher<lcmcl_msgs::msg::Localization>::SharedPtr     _true_pos_pub;

    const uint32_t  _odom_interval = 10;

    pos_t _cmd_pos;
    pos_t _true_pos;

    const float _err_k = 0.1;
    pos_t _avg_err = {0, 0, 0};
    pos_t _disp_err = {0, 0, 0};

    lcmcl::Localization _topics;

public:
    LcmclSimulator(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : LcmclSimulator("", options){}
    LcmclSimulator(const std::string &name_space, const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) 
        : blackbox::BlackBoxNode(blackbox::debug_mode_t::RELEASE, "sim_lcmcl", name_space, options)
    {
        this->_tf = std::make_shared<lc::Tf>(this, std::chrono::milliseconds(5000));
        this->_true_pos = _tf->get_initial_pos();
        
        this->_map = std::make_shared<lc::Map>(this, _tf);

        _odometry.init(this, _tf->get_initial_pos(), _tf);
        _sick.init(this,  _map, _tf);

        this->_laser_pub = this->create_publisher<lcmcl_msgs::msg::Laser>("sensor_laser", rclcpp::QoS(1).reliable());

        this->_true_pos_pub = this->create_publisher<lcmcl_msgs::msg::Localization>("true_localization", rclcpp::QoS(1).reliable());
        this->_joy_sub = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, std::bind(&LcmclSimulator::joy_subscriber, this, std::placeholders::_1));
        this->_odom_pub = this->create_publisher<lcmcl_msgs::msg::Odometry>("sensor_odometry", rclcpp::QoS(1).reliable());

        this->_cmd_pos.x = 0;
        this->_cmd_pos.y = 0;
        this->_cmd_pos.rad = 0;

        this->_odom_pub_tim = this->create_wall_timer(std::chrono::milliseconds(this->_odom_interval), std::bind(&LcmclSimulator::odom_est_publisher, this));

        _topics.init(this);
    }

    void joy_subscriber(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
#ifdef MY_JOY
        this->_cmd_pos.rad = (msg->axes[4] - msg->axes[5]);
        if(this->_cmd_pos.rad != 0)
        {
            this->_cmd_pos.x = msg->axes[3];
            this->_cmd_pos.y = msg->axes[2];
        }else{
            this->_cmd_pos.x = msg->axes[3] * 1.2;
            this->_cmd_pos.y = msg->axes[2] * 1.2;
        }
#else
        this->_cmd_pos.rad = (msg->axes[5] - msg->axes[2]);
        if(this->_cmd_pos.rad != 0)
        {
            this->_cmd_pos.x = msg->axes[1];
            this->_cmd_pos.y = msg->axes[0];
        }else{
            this->_cmd_pos.x = msg->axes[1] * 1.2;
            this->_cmd_pos.y = msg->axes[0] * 1.2;
        }
#endif /*_joy_sub*/
    }

    void odom_est_publisher(void)
    {
        float cmd_rate = 0.5;
        float d_x = ((float)this->_cmd_pos.x / 25) * cmd_rate;
        float d_y = ((float)this->_cmd_pos.y / 25) * cmd_rate;
        float d_rad = (((float)this->_cmd_pos.rad * M_PI) / 180) * cmd_rate;
        this->_true_pos.x += d_x;
        this->_true_pos.y += d_y;
        this->_true_pos.rad += d_rad;

        auto odom_msg = this->_odometry.sim(_true_pos);
        std::get<0>(odom_msg).header.stamp = this->get_clock()->now();
        _odom_pub->publish(std::get<0>(odom_msg));

        lcmcl_msgs::msg::Laser laser = this->_sick.sim(_true_pos, std::get<1>(odom_msg), std::get<2>(odom_msg));
        laser.header.stamp = this->get_clock()->now();
        _laser_pub->publish(laser);

        lcmcl_msgs::msg::Localization true_msg;
        true_msg.header.stamp = this->get_clock()->now();
        true_msg.abs_x = _true_pos.x;
        true_msg.abs_y = _true_pos.y;
        true_msg.abs_rad = _true_pos.rad;
        _true_pos_pub->publish(true_msg);
    }
};
