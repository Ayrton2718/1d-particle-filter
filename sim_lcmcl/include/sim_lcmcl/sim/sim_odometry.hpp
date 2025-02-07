#pragma once

#include <cmath>
#include <random>

#include "sim_type.hpp"
#include <lcmcl_msgs/msg/odometry.hpp>
#include <lc_map/lc_tf.hpp>

// 追加するメッセージヘッダ
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace sim
{

class Odometry
{
private:
    blackbox::BlackBoxNode* _node;

    std::random_device          _gn_seed_gen;
    std::default_random_engine  _gn_engine;

    rclcpp::Time _last_tim;

    blackbox::Logger    _info;

    pos_t _enc;   // エンコーダからの積算値（ノイズ含む）
    pos_t _odom;  // ローカライゼーション結果（積分値）

    pos_t _befor_pos;
    pos_t _befor_enc;

    // 追加：Odometry, IMU用のパブリッシャ
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _odom_pub;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr _imu_pub;

public:
    Odometry(void) : _gn_engine(_gn_seed_gen())
    {
    }
    
    void init(blackbox::BlackBoxNode* node, pos_t initial_pos)
    {
        this->_node = node;

        this->_odom = initial_pos;
        this->_befor_pos = initial_pos;

        this->_enc = {0, 0, 0};
        this->_befor_enc = this->_enc;
        
        _last_tim = _node->get_clock()->now();

        _info.init(node, blackbox::INFO, "encoder");

        // パブリッシャの作成
        _odom_pub = _node->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        _imu_pub = _node->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    }

    // シミュレーション＆パブリッシュ（返り値不要）
    void publish(pos_t true_pos)
    {
        pos_t pos_delta;
        pos_delta.x = true_pos.x - _befor_pos.x;
        pos_delta.y = true_pos.y - _befor_pos.y;
        pos_delta.rad = true_pos.rad - _befor_pos.rad;

        double d_e_x = pos_delta.x * cos(true_pos.rad) + pos_delta.y * sin(true_pos.rad);
        double d_e_y = pos_delta.x * -sin(true_pos.rad) + pos_delta.y * cos(true_pos.rad);
        double d_e_rad = pos_delta.rad;

        std::normal_distribution<double> dist(0, 0.1);
        std::normal_distribution<double> dist_theta(0, 0.15);
        std::normal_distribution<double> white_theta(0, 0.1 *(M_PI / 180));
        _enc.x += d_e_x +(dist(_gn_engine) * d_e_x);
        _enc.y += d_e_y + (dist(_gn_engine) * d_e_y);
        _enc.rad += d_e_rad + (dist_theta(_gn_engine) * d_e_rad) + white_theta(_gn_engine);

        pos_t enc_delta;
        enc_delta.x = _enc.x - _befor_enc.x;
        enc_delta.y = _enc.y - _befor_enc.y;
        enc_delta.rad = _enc.rad - _befor_enc.rad;

        _odom.x += enc_delta.x * cos(_enc.rad) - enc_delta.y * sin(_enc.rad);
        _odom.y += enc_delta.x * sin(_enc.rad) + enc_delta.y * cos(_enc.rad);
        _odom.rad += enc_delta.rad;

        _befor_pos = true_pos;
        _befor_enc = _enc;

        auto now_tim = _node->get_clock()->now();
        auto delta_tim = now_tim - _last_tim;
        _last_tim = now_tim;

        double dt = delta_tim.seconds();
        if (dt == 0) { dt = 0.001; } // ゼロ除算回避

        // ■ nav_msgs::msg::Odometry メッセージの作成
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = now_tim;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id  = "base_link";

        // pose：積分した位置・向き（Quaternionに変換）
        odom_msg.pose.pose.position.x = _odom.x;
        odom_msg.pose.pose.position.y = _odom.y;
        odom_msg.pose.pose.position.z = 0.0;
        {
            tf2::Quaternion q;
            q.setRPY(0, 0, _odom.rad);
            odom_msg.pose.pose.orientation = tf2::toMsg(q);
        }

        // twist：エンコーダ差分から速度（※単純な差分計算）
        odom_msg.twist.twist.linear.x  = enc_delta.x / dt;
        odom_msg.twist.twist.linear.y  = enc_delta.y / dt;
        odom_msg.twist.twist.angular.z = enc_delta.rad / dt;
        odom_msg.twist.twist.angular.x = 0.0;
        odom_msg.twist.twist.angular.y = 0.0;

        _odom_pub->publish(odom_msg);

        // ■ sensor_msgs::msg::Imu メッセージの作成
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp = now_tim;
        imu_msg.header.frame_id = "imu_link";  // 必要に応じて変更

        // orientation：エンコーダ角度（ノイズ付き）からQuaternionに変換
        {
            tf2::Quaternion imu_q;
            imu_q.setRPY(0, 0, _enc.rad);
            imu_msg.orientation = tf2::toMsg(imu_q);
        }
        // angular_velocity：エンコーダからの角速度（z軸のみ） 
        imu_msg.angular_velocity.z = enc_delta.rad / dt;
        imu_msg.angular_velocity.x = 0.0;
        imu_msg.angular_velocity.y = 0.0;

        // linear_acceleration：今回はシミュレーションしていないので0とする
        imu_msg.linear_acceleration.x = 0.0;
        imu_msg.linear_acceleration.y = 0.0;
        imu_msg.linear_acceleration.z = 0.0;

        // 必要に応じ、各種共分散行列を設定（ここでは「不明」を示す -1 をセット）
        for (int i = 0; i < 9; i++) {
            imu_msg.orientation_covariance[i]         = 0.0;
            imu_msg.angular_velocity_covariance[i]    = 0.0;
            imu_msg.linear_acceleration_covariance[i]   = 0.0;
        }
        imu_msg.orientation_covariance[0]         = -1.0;
        imu_msg.angular_velocity_covariance[0]    = -1.0;
        imu_msg.linear_acceleration_covariance[0]   = -1.0;

        _imu_pub->publish(imu_msg);
    }

};

} // namespace sim
