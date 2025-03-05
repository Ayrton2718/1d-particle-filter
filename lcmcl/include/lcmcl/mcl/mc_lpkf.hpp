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

    // 出力メッセージ型を nav_msgs::msg::Odometry に変更
    blackbox::PubRecord<nav_msgs::msg::Odometry> _pf_odom_pub;
    blackbox::PubRecord<nav_msgs::msg::Odometry> _ekf_odom_pub;


    rclcpp::TimerBase::SharedPtr _timeout_tim;

public:
    LPKf(blackbox::BlackBoxNode* node,
         std::shared_ptr<lc::Tf> tf,
         std::shared_ptr<lc::Map> map)
        : _os(node),
          _sick(node, map, tf, std::bind(&LPKf::predict_callback, this, std::placeholders::_1)),
          _kf(node, &_os, tf->get_initial_pos()),
          _pf(node, &_os, tf->get_initial_pos(), &_sick, &_kf)
    {
        _node = node;
        _tf = tf;
        _map = map;

        _pf_odom_pub.init(_node, "pf_odom");
        _ekf_odom_pub.init(_node, "kf_odom");

        _timeout_tim = node->create_wall_timer(
            std::chrono::milliseconds(10),
            [this]() { this->predict_callback(_node->now()); });
    }

    void predict_callback(rclcpp::Time sens_tim)
    {
        _timeout_tim->reset();

        auto pf_result = _pf.predict(sens_tim);
        auto kf_result = _kf.predict(pf_result);

        publish_odom(&_pf_odom_pub, &pf_result);
        publish_odom(&_ekf_odom_pub, &kf_result);
    }

    void publish_odom(blackbox::PubRecord<nav_msgs::msg::Odometry>* pub, const KfBase::odom_t* result){
        auto pos = result->pos;
        if(std::isnan(pos(0)) || std::isnan(pos(1)) || std::isnan(pos(2)))
        {
            pos(0) = 0;
            pos(1) = 0;
            pos(2) = 0;
        }

        nav_msgs::msg::Odometry odom_est;
        odom_est.header.frame_id = "map";
        odom_est.header.stamp = result->tim;
        odom_est.child_frame_id = "base_footprint";

        odom_est.pose.pose.position.x = pos(0);
        odom_est.pose.pose.position.y = pos(1);
        odom_est.pose.pose.position.z = 0.0;

        // 共分散
        std::array<double, 36> cov = {0.0};
        cov[0] = result->cov(0,0);   // x
        cov[1] = result->cov(0,1);   // x, y
        cov[5] = result->cov(0,2);   // x, yaw
        cov[6] = result->cov(1,0);   // y, x
        cov[7] = result->cov(1,1);   // y
        cov[11] = result->cov(1,2);  // y, yaw
        cov[30] = result->cov(2,0);  // yaw, x
        cov[31] = result->cov(2,1);  // yaw, y
        cov[35] = result->cov(2,2);  // yaw
        
        odom_est.pose.covariance = cov;

        tf2::Quaternion q;
        q.setRPY(0, 0, result->pos(2));
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

        pub->publish(odom_est);
    }
};

}  // namespace mcl
