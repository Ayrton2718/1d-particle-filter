#pragma once

#include <rclcpp/rclcpp.hpp>
#include <lcmcl_msgs/msg/localization.hpp>
#include <lcmcl_msgs/msg/odometry.hpp>
#include <lc_map/lc_type.hpp>
#include <Eigen/Geometry>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <atomic>

namespace lcmcl
{

class Localization
{
private:
    template<typename T>
    class FlipBuffer{
    public:
        FlipBuffer(){
            _read_index.store(0);
        }

        void store(T data){
            uint8_t next_write = (_read_index.load() + 1) % 2;
            _buff[next_write] = data;
            _read_index.store(next_write);
        }

        T load(void){
            return _buff[_read_index.load()];
        }

    private:
        std::atomic<uint8_t> _read_index;
        std::array<T, 2> _buff;
    };

private:
    rclcpp::Node* _node;

    rclcpp::Subscription<lcmcl_msgs::msg::Localization>::SharedPtr  _loc_sub;
    rclcpp::Subscription<lcmcl_msgs::msg::Odometry>::SharedPtr      _odom_sub;

    FlipBuffer<lcmcl_msgs::msg::Localization>       _loc;
    FlipBuffer<lcmcl_msgs::msg::Odometry>           _odom;

public:
    Localization(void)
    {
    }

    void init(rclcpp::Node* node){
        this->init(node, [](rclcpp::Time){});
    }

    void init(rclcpp::Node* node, std::function<void(rclcpp::Time)> callback)
    {
        lcmcl_msgs::msg::Localization loc_msg;
        loc_msg.header.stamp = rclcpp::Time(0, 0, node->get_clock()->get_clock_type());
        loc_msg.abs_x = 0;
        loc_msg.abs_y = 0;
        loc_msg.abs_rad = 0;
        loc_msg.rel_pitch = 0;
        loc_msg.rel_roll = 0;
        loc_msg.damper_r = -1;
        loc_msg.damper_l = -1;
        _loc.store(loc_msg);

        lcmcl_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = rclcpp::Time(0, 0, node->get_clock()->get_clock_type());
        odom_msg.rel_x = 0;
        odom_msg.rel_y = 0;
        odom_msg.rel_rad = 0;
        odom_msg.rel_vx = 0;
        odom_msg.rel_vy = 0;
        odom_msg.rel_vrad = 0;
        _odom.store(odom_msg);

        _loc_sub = node->create_subscription<lcmcl_msgs::msg::Localization>("/localization/est_localization", rclcpp::QoS(1).reliable(), 
                    [this, callback](const lcmcl_msgs::msg::Localization::SharedPtr msg){
                        _loc.store(*msg);
                        callback(msg->header.stamp);
                    });

        _odom_sub = node->create_subscription<lcmcl_msgs::msg::Odometry>("/localization/sensor_odometry", rclcpp::QoS(1).reliable(), 
                    [this](const lcmcl_msgs::msg::Odometry::SharedPtr msg){
                        _odom.store(*msg);
                    });
    }

    lc::pos_t abs_pos(void)
    {
        auto loc = _loc.load();
        return {loc.abs_x, loc.abs_y, loc.abs_rad};
    }

    lc::pos_t abs_vel(void)
    {
        auto loc = _loc.load();
        auto odom = _odom.load();
        lc::pos_t vel;
        vel.x = odom.rel_vx * cos(loc.abs_rad) + odom.rel_vy * -sin(loc.abs_rad);
        vel.y = odom.rel_vx * sin(loc.abs_rad) + odom.rel_vy * cos(loc.abs_rad);
        vel.rad = odom.rel_vrad;
        return vel;
    }

    lc::pos_t rel_pos(void)
    {
        auto odom = _odom.load();
        return {odom.rel_x, odom.rel_y, odom.rel_rad};
    }

    lc::pos_t rel_vel(void)
    {
        auto odom = _odom.load();
        return {odom.rel_vx, odom.rel_vy, odom.rel_vrad};
    }

    tf2::Quaternion sensor_rotation(const tf2::Quaternion& tf_q)
    {
        auto loc = _loc.load();
        tf2::Quaternion rel_q;
        rel_q.setRPY(loc.rel_roll, -loc.rel_pitch, 0);
        return rel_q * tf_q;
    }

    rclcpp::Time timestamp(void){
        auto loc = _loc.load();
        return loc.header.stamp;
    }

    Eigen::Vector3d abs_trans_vec(Eigen::Vector3d vec){
        auto loc = _loc.load();
        Eigen::Quaterniond rel_q(
            Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(loc.rel_pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(loc.rel_roll, Eigen::Vector3d::UnitX())
        );

        Eigen::Quaterniond yaw_q(
            Eigen::AngleAxisd(-loc.abs_rad, Eigen::Vector3d::UnitZ())
        );

        return yaw_q * (rel_q * vec);
    }

    /// @brief 
    /// @param  
    /// @return {damper_r, damper_l}
    std::pair<rclcpp::Time, std::pair<float, float>> damper_sick(void){
        auto loc = _loc.load();
        return {
            loc.header.stamp,
            {loc.damper_r, loc.damper_l}};
    }

    std::pair<rclcpp::Time, lc::pos_t> rel_vel_tim(void){
        auto odom = _odom.load();
        return {
            odom.header.stamp,
            {odom.rel_vx, odom.rel_vy, odom.rel_vrad}
            };
    }
};

}