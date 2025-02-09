#pragma once

#include "mc_pf.hpp"
#include "mc_ekf.hpp"
#include "mc_kf.hpp"
#include "mc_sick.hpp"

#include "nav_msgs/msg/odometry.hpp"

namespace mcl
{

class LPKf
{
private:
    blackbox::BlackBoxNode* _node;
    std::shared_ptr<lc::Tf> _tf;
    std::shared_ptr<lc::Map> _map;

    mcl::OdomSubscriber     _os;
    mcl::Sick               _sick;
    mcl::EKF                _kf;
    mcl::Pf                 _pf;

    pos_t    _tire_lf;
    pos_t    _tire_rf;
    pos_t    _tire_lb;
    pos_t    _tire_rb;

    pos_t   _damper_r_tf;
    pos_t   _damper_l_tf;
    std::vector<int> _damper_obs;
    int     _damper_silo_wall;

    // 出力メッセージ型を nav_msgs::msg::Odometry に変更
    blackbox::PubRecord<nav_msgs::msg::Odometry> _est_pub;
    rclcpp::TimerBase::SharedPtr _timeout_tim;

public:
    LPKf(blackbox::BlackBoxNode* node,
         std::shared_ptr<lc::Tf> tf,
         std::shared_ptr<lc::Map> map)
        : _os(node, std::bind(&LPKf::publish_localization, this, std::placeholders::_1)),
          _sick(node, map, tf, std::bind(&LPKf::sens_callback, this, std::placeholders::_1)),
          _kf(node, &_os, tf->get_initial_pos()),
          _pf(node, &_os, tf->get_initial_pos(), &_sick, &_kf)
    {
        _node = node;
        _tf = tf;
        _map = map;

        _est_pub.init(_node, "est_position");

        _timeout_tim = node->create_wall_timer(
            std::chrono::milliseconds(10),
            [this]() { this->sens_callback(_node->get_clock()->now()); });
    }

    void sens_callback(rclcpp::Time sens_tim)
    {
        _timeout_tim->reset();

        auto pf_obs = _pf.predict(sens_tim);
        _kf.observation(pf_obs);

        pos_t pos = _kf.predict_pos(std::get<0>(pf_obs));
    }

    void publish_localization(rclcpp::Time sens_tim)
    {
        pos_t pos = _kf.predict_pos(sens_tim);

        if(std::isnan(pos.x) || std::isnan(pos.y) || std::isnan(pos.rad))
        {
            pos.x = 0;
            pos.y = 0;
            pos.rad = 0;
        }

        nav_msgs::msg::Odometry odom_est;
        odom_est.header.frame_id = "map";
        odom_est.header.stamp = sens_tim;
        odom_est.child_frame_id = "base_footprint";

        odom_est.pose.pose.position.x = pos.x;
        odom_est.pose.pose.position.y = pos.y;
        odom_est.pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, pos.rad);
        q.normalize();
        odom_est.pose.pose.orientation.x = q.x();
        odom_est.pose.pose.orientation.y = q.y();
        odom_est.pose.pose.orientation.z = q.z();
        odom_est.pose.pose.orientation.w = q.w();

        odom_est.twist.twist.linear.x = 0.0;
        odom_est.twist.twist.linear.y = 0.0;
        odom_est.twist.twist.linear.z = 0.0;
        odom_est.twist.twist.angular.x = 0.0;
        odom_est.twist.twist.angular.y = 0.0;
        odom_est.twist.twist.angular.z = 0.0;

        _est_pub.publish(odom_est);
    }

};

}  // namespace mcl
