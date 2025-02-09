#pragma once

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <random>
#include <float.h>

#include <std_msgs/msg/float32_multi_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <visualization_msgs/msg/marker.hpp>

#include "mc_type.hpp"
#include "mc_kf_base.hpp"

#include <Eigen/Dense>

// #define EKF_VIEW_COVARIANCE

namespace mcl
{

class EKF : public KfBase
{
private:
    blackbox::BlackBoxNode* _node;
    blackbox::Logger    _ekf_info;

    Delta               _delta;

    Eigen::Matrix<float, 3, 1> _x;
    Eigen::Matrix<float, 3, 3> _P;
    Eigen::Matrix<float, 3, 3> _Q;
    Eigen::Matrix<float, 3, 3> _QStorage;

    blackbox::Param<float> _param_Qxx;
    blackbox::Param<float> _param_Qzz;
    blackbox::Param<float> _param_activeQxx;
    blackbox::Param<float> _param_activeQzz;
    blackbox::Param<float> _param_QxxStorage;
    blackbox::Param<float> _param_QzzStorage;
    blackbox::Param<float> _param_activeQxxStorage;
    blackbox::Param<float> _param_activeQzzStorage;

    uint32_t _cut_counter = 0;
    
    blackbox::Record<std_msgs::msg::Float32MultiArray, false, true> _gain_record;


#ifdef EKF_VIEW_COVARIANCE
    blackbox::PubRecord<visualization_msgs::msg::Marker>   _cov_pub;
#endif /*EKF_VIEW_COVARIANCE*/

    Eigen::Matrix<float, 3, 1> predict_x(pos_t delta)
    {
        Eigen::Matrix<float, 3, 1> u;
        u <<    delta.x*cos(_x(2)) - delta.y*sin(_x(2)),
                delta.x*sin(_x(2)) + delta.y*cos(_x(2)),
                delta.rad;

        return _x + u;
    }

    void view_covariance(KfBase::observation_t pf_obs){
#ifdef EKF_VIEW_COVARIANCE
        // エリプスの軸の長さ（固有値の平方根）
        auto pf_cov = std::get<2>(pf_obs);
        double length_x = 2 * std::sqrt(pf_cov(0, 0));
        double length_y = 2 * std::sqrt(pf_cov(1, 1));

        // Markerの設定
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = std::get<0>(pf_obs);
        marker.ns = "ellipse";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // 位置（適当に設定）
        auto pf_pos = std::get<1>(pf_obs);
        marker.pose.position.x = pf_pos(0);
        marker.pose.position.y = pf_pos(1);
        marker.pose.position.z = 0.0;

        // 向き
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 1;

        // 大きさ
        marker.scale.x = length_x;
        marker.scale.y = length_y;
        marker.scale.z = 0.1;

        // 色
        marker.color.a = 0.5;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        _cov_pub.publish(marker);
#endif /*EKF_VIEW_COVARIANCE*/
    }

public:
    EKF(blackbox::BlackBoxNode* node, OdomSubscriber* os, pos_t initial_pos) :  _delta(os)
    {
        this->_node = node;

        _x <<   initial_pos.x,
                initial_pos.y,
                initial_pos.rad;

        _P <<   1.0E-8, 1.0E-12, 1.0E-12,
                1.0E-12, 1.0E-8, 1.0E-12,
                1.0E-12, 1.0E-12, 1.0E-8;

        _param_Qxx.init(node, "Qxx", 1.0E-6);
        _param_Qzz.init(node, "Qzz", 1.0E-6);
        _param_activeQxx.init(node, "activeQxx", 0);
        _param_activeQzz.init(node, "activeQzz", 0);
        _param_QxxStorage.init(node, "QxxStorage", 1.0E-6);
        _param_QzzStorage.init(node, "QzzStorage", 1.0E-12);
        _param_activeQxxStorage.init(node, "activeQxxStorage", 0);
        _param_activeQzzStorage.init(node, "activeQzzStorage", 0);

        _ekf_info.init(node, blackbox::INFO, "ekf");
        TAGGER(_ekf_info, "param Qxx: %e", _param_Qxx.get());
        TAGGER(_ekf_info, "param Qxy: %e", _param_Qzz.get());
        TAGGER(_ekf_info, "param activeQxx: %e", _param_activeQxx.get());
        TAGGER(_ekf_info, "param activeQzz: %e", _param_activeQzz.get());
        TAGGER(_ekf_info, "param QxxStorage: %e", _param_QxxStorage.get());
        TAGGER(_ekf_info, "param QxyStorage: %e", _param_QzzStorage.get());
        TAGGER(_ekf_info, "param activeQxxStorage: %e", _param_activeQxxStorage.get());
        TAGGER(_ekf_info, "param activeQzzStorage: %e", _param_activeQzzStorage.get());

        float q_xx = _param_Qxx.get();
        float q_zz = _param_Qzz.get();
        _Q <<   q_xx,   0,      0,
                0,      q_xx,   0,
                0,      0,      q_zz;

        float qs_xx = _param_QxxStorage.get();
        float qs_zz = _param_QzzStorage.get();
        _QStorage <<    qs_xx,  0,      0,
                        0,      qs_xx,  0,
                        0,      0,      qs_zz;

        _gain_record.init(node, "gain_record", 2);

#ifdef EKF_VIEW_COVARIANCE
        _cov_pub.init(node, "ekf_covariance", rclcpp::SensorDataQoS());
#endif /*EKF_VIEW_COVARIANCE*/
    }

    std::array<double, MCLTYPE_N> calc_weight(rclcpp::Time pf_tim, std::array<lc::pos_t, MCLTYPE_N> pf_pos)
    {
        std::array<double, MCLTYPE_N> weight_list;

        // RCLCPP_INFO(this->_node->get_logger(), "%f, %f, %f", now_pos.x, now_pos.y, now_pos.rad);
        
        Eigen::Matrix<float, 3, 1> now_pos = predict_x(_delta.delta(pf_tim, false));

        pos_t disp;
        disp.x = 0.5;
        disp.y = 0.5;
        disp.rad = 5 * (M_PI/180);

        if(500 < _cut_counter)
        {
            for(size_t i = 0; i < MCLTYPE_N; i++)
            {
                pos_t pos = pf_pos[i];
                float w_x = exp(-((pos.x - now_pos(0)) * (pos.x - now_pos(0))) / (2 * (disp.x) * (disp.x)));
                float w_y = exp(-((pos.y - now_pos(1)) * (pos.y - now_pos(1))) / (2 * disp.y * disp.y));
                float w_rad = exp(-((pos.rad - now_pos(2)) * (pos.rad - now_pos(2))) / (2 * disp.rad * disp.rad));
                weight_list[i] = w_x * w_y * w_rad;
            }
        }else{
            for(size_t i = 0; i < MCLTYPE_N; i++)
            {
                weight_list[i] = 1;
            }
            this->_cut_counter++;
        }

        return weight_list;
    }

    void observation(KfBase::observation_t pf_obs)
    {
        this->ekf(pf_obs);
        this->view_covariance(pf_obs);
    }


    void ekf(KfBase::observation_t pf_obs)
    {
        pos_t rel_vel = _delta.velocity();
        pos_t delta = _delta.delta(std::get<0>(pf_obs), true);
        Eigen::Matrix<float, 3, 1> pre_x = predict_x(delta);

        Eigen::Matrix<float, 3, 3> A;
        A <<    1, 0, -delta.x*sin(_x(2)) - delta.y*cos(_x(2)),
                0, 1, delta.x*cos(_x(2)) - delta.y*sin(_x(2)),
                0, 0, 1;
        
        pos_t abs_vel;
        abs_vel.x = rel_vel.x*cos(_x(2)) - rel_vel.y*sin(_x(2));
        abs_vel.y = rel_vel.x*sin(_x(2)) + rel_vel.y*cos(_x(2));
        abs_vel.rad = rel_vel.rad;

        Eigen::Matrix<float, 3, 3> Q;
        pos_t pre_pos = {pre_x(0), pre_x(1), pre_x(2)};
        Q = _Q;
        Q(0, 0) += fabs(abs_vel.x * _param_activeQxx.get());
        Q(1, 1) += fabs(abs_vel.y * _param_activeQxx.get());
        Q(2, 2) += fabs(abs_vel.rad * _param_activeQzz.get());

        Eigen::Matrix<float, 3, 3> pre_P;
        pre_P = A * _P * A.transpose() + Q;


        Eigen::Matrix<float, 3, 3> C;
        C <<    1, 0, 0,
                0, 1, 0,
                0, 0, 1;
        auto C_T = C.transpose();

        Eigen::Matrix<float, 3, 3> R = std::get<2>(pf_obs);

        Eigen::Matrix<float, 3, 3> G;
        G = pre_P * C_T * (C * pre_P * C_T + R).inverse();
        G = G.unaryExpr([](float x) {
            return x > (float)0.05 ? (float)0.05 : x;
        });

        Eigen::Matrix<float, 3, 1> y = std::get<1>(pf_obs);
        
        _x = pre_x + G * (y - pre_x);
        _P = (Eigen::Matrix3f::Identity() - G * C) * pre_P;

        std_msgs::msg::Float32MultiArray rec_msg;
        rec_msg.data = {G(0, 0), G(1, 1), G(2, 2), R(0, 0), R(1, 1), R(2, 2), pre_P(0, 0), pre_P(1, 1), pre_P(2, 2)};
        _gain_record.record(rec_msg);
    }


    pos_t predict_pos(rclcpp::Time sens_tim)
    {
        Eigen::Matrix<float, 3, 1> now_pos = predict_x(_delta.delta(sens_tim, false));
        return {now_pos(0), now_pos(1), now_pos(2)};
    }
};

}