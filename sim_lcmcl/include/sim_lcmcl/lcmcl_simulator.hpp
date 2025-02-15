#pragma once

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <cmath>
#include <float.h>
#include <random>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <std_msgs/msg/string.hpp>

#include "sim/sim_odometry.hpp"
#include "sim/sim_sick.hpp"
#include <lc_map/lc_tf.hpp>

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

    rclcpp::TimerBase::SharedPtr                                    _odom_pub_tim;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr          _joy_sub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _true_odom_pub;

    const uint32_t  _odom_interval = 10;

    pos_t _cmd_pos;
    pos_t _true_pos;

    const float _err_k = 0.1;
    pos_t _avg_err = {0, 0, 0};
    pos_t _disp_err = {0, 0, 0};

    blackbox::Logger _info;

public:
    LcmclSimulator(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : LcmclSimulator("", options){}
    LcmclSimulator(const std::string &name_space, const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) 
        : blackbox::BlackBoxNode(blackbox::debug_mode_t::RELEASE, "sim_lcmcl", name_space, options)
    {
        _info.init(this, blackbox::INFO, "sim_info");

        this->_tf = std::make_shared<lc::Tf>(this);
        this->_true_pos = _tf->get_initial_pos();
        
        this->_map = std::make_shared<lc::Map>(this);

        _odometry.init(this, _tf->get_initial_pos());
        _sick.init(this,  _map, _tf);

        this->_true_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("/waffle_1d/gazebo_position", 10);
        this->_joy_sub = this->create_subscription<geometry_msgs::msg::Twist>("/waffle_1d/cmd_vel", 10, std::bind(&LcmclSimulator::joy_subscriber, this, std::placeholders::_1));

        this->_cmd_pos.x = 0;
        this->_cmd_pos.y = 0;
        this->_cmd_pos.rad = 0;

        this->_odom_pub_tim = this->create_wall_timer(std::chrono::milliseconds(this->_odom_interval), std::bind(&LcmclSimulator::odom_est_publisher, this));
    }

    void joy_subscriber(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        this->_cmd_pos.x = msg->linear.x * cos(this->_true_pos.rad) - msg->linear.y * sin(this->_true_pos.rad);
        this->_cmd_pos.y = msg->linear.y * cos(this->_true_pos.rad) + msg->linear.x * sin(this->_true_pos.rad);
        this->_cmd_pos.rad = msg->angular.z;
#ifdef MY_JOY
        // this->_cmd_pos.rad = (msg->axes[4] - msg->axes[5]);
        // if(this->_cmd_pos.rad != 0)
        // {
        //     this->_cmd_pos.x = msg->axes[3];
        //     this->_cmd_pos.y = msg->axes[2];
        // }else{
        //     this->_cmd_pos.x = msg->axes[3] * 1.2;
        //     this->_cmd_pos.y = msg->axes[2] * 1.2;
        // }
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

        this->_odometry.publish(_true_pos);
        this->_sick.publish(_true_pos);

        auto now = this->get_clock()->now();
        nav_msgs::msg::Odometry true_odom;
        tf2::Quaternion q;
        true_odom.header.stamp = now;
        true_odom.header.frame_id = "map";
        true_odom.child_frame_id  = "base_footprint";
        true_odom.pose.pose.position.x = this->_true_pos.x;
        true_odom.pose.pose.position.y = this->_true_pos.y;
        true_odom.pose.pose.position.z = 0.0;
        q.setRPY(0, 0, _true_pos.rad);
        true_odom.pose.pose.orientation = tf2::toMsg(q);
        _true_odom_pub->publish(true_odom);
    }
};
