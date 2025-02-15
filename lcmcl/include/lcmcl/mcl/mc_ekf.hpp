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

    blackbox::Param<float> _param_Qxx;
    blackbox::Param<float> _param_Qzz;
    blackbox::Param<float> _param_activeQxx;
    blackbox::Param<float> _param_activeQzz;

    uint32_t _cut_counter = 0;
    
    blackbox::Record<std_msgs::msg::Float32MultiArray, false, true> _gain_record;


    Eigen::Matrix<float, 3, 1> predict_x(pos_t delta)
    {
        Eigen::Matrix<float, 3, 1> u;
        u <<    delta.x*cos(_x(2)) - delta.y*sin(_x(2)),
                delta.x*sin(_x(2)) + delta.y*cos(_x(2)),
                delta.rad;

        return _x + u;
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

        _param_Qxx.init(node, "ekf.Qxx", 1.0E-6);
        _param_Qzz.init(node, "ekf.Qzz", 1.0E-6);
        _param_activeQxx.init(node, "ekf.activeQxx", 0);
        _param_activeQzz.init(node, "ekf.activeQzz", 0);

        _ekf_info.init(node, blackbox::INFO, "ekf");
        TAGGER(_ekf_info, "param Qxx: %e", _param_Qxx.get());
        TAGGER(_ekf_info, "param Qxy: %e", _param_Qzz.get());
        TAGGER(_ekf_info, "param activeQxx: %e", _param_activeQxx.get());
        TAGGER(_ekf_info, "param activeQzz: %e", _param_activeQzz.get());

        float q_xx = _param_Qxx.get();
        float q_zz = _param_Qzz.get();
        _Q <<   q_xx,   0,      0,
                0,      q_xx,   0,
                0,      0,      q_zz;

        _gain_record.init(node, "gain_record", 2);
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


    KfBase::odom_t predict(KfBase::odom_t pf_result)
    {
        pos_t rel_vel = _delta.velocity();
        pos_t delta = _delta.delta(pf_result.tim, true);
        Eigen::Matrix<float, 3, 1> pre_x = predict_x(delta);

        Eigen::Matrix<float, 3, 3> A;
        A <<    1, 0, -delta.x*sin(_x(2)) - delta.y*cos(_x(2)),
                0, 1, delta.x*cos(_x(2)) - delta.y*sin(_x(2)),
                0, 0, 1;
        
        pos_t abs_vel;
        abs_vel.x = rel_vel.x*cos(_x(2)) - rel_vel.y*sin(_x(2));
        abs_vel.y = rel_vel.x*sin(_x(2)) + rel_vel.y*cos(_x(2));
        abs_vel.rad = rel_vel.rad;

        Eigen::Matrix<float, 3, 3> Q = _Q;
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

        Eigen::Matrix<float, 3, 3> R = pf_result.cov;

        Eigen::Matrix<float, 3, 3> G;
        G = pre_P * C_T * (C * pre_P * C_T + R).inverse();
        G = G.unaryExpr([](float x) {
            return x > (float)0.05 ? (float)0.05 : x;
        });

        Eigen::Matrix<float, 3, 1> y = pf_result.pos;
        
        _x = pre_x + G * (y - pre_x);
        _P = (Eigen::Matrix3f::Identity() - G * C) * pre_P;

        std_msgs::msg::Float32MultiArray rec_msg;
        rec_msg.data = {_x(0), _x(1), _x(2), G(0, 0), G(1, 1), G(2, 2), R(0, 0), R(1, 1), R(2, 2), pre_P(0, 0), pre_P(1, 1), pre_P(2, 2)};
        _gain_record.record(rec_msg);

        KfBase::odom_t result;
        result.tim = pf_result.tim;
        result.pos = _x;
        result.cov = _P;
        return result;
    }
};

}