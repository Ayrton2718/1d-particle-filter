#pragma once

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <random>
#include <float.h>

#include <std_msgs/msg/float32_multi_array.hpp>
#include <lcmcl_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include "mc_type.hpp"

namespace mcl
{

class KF : public KfBase
{
private:
    blackbox::BlackBoxNode* _node;
    
    Delta _delta;

    // pos_t   _delta;
    pos_t   _x;
    pos_t   _x_disp;
    pos_t   _x_noise;

    uint32_t _cut_counter = 0;
    // rclcpp::Subscription<lcmcl_msgs::msg::Odometry>::SharedPtr _sub;

    uint8_t is_enable = 1;
    
    blackbox::Record<std_msgs::msg::Float32MultiArray, false, true> _gain_record;

public:
    KF(blackbox::BlackBoxNode* node, OdomSubscriber* os, pos_t initial_pos) : _delta(os)
    {
        this->_node = node;

        this->_x = initial_pos;

        this->_x_disp.x = 1;
        this->_x_disp.y = 1;
        this->_x_disp.rad = 30 * (M_PI/180);

        this->_x_noise.x = 1.0E-8;
        this->_x_noise.y = 1.0E-8;
        this->_x_noise.rad = 1.0E-8 * (M_PI/180);

        _gain_record.init(node, "gain_record", 2);
    }

    std::array<double, MCLTYPE_N> calc_weight(rclcpp::Time pf_tim, std::array<lc::pos_t, MCLTYPE_N> pf_pos)
    {
        std::array<double, MCLTYPE_N> weight_list;

        pos_t d = _delta.delta(pf_tim, false);
        pos_t abs_d ={
            d.x * cos(_x.rad) + d.y * -sin(_x.rad),
            d.x * sin(_x.rad) + d.y * cos(_x.rad),
            d.rad
        };

        pos_t now_pos;
        now_pos.x = this->_x.x + abs_d.x;
        now_pos.y = this->_x.y + abs_d.y;
        now_pos.rad = this->_x.rad + abs_d.rad;

        // RCLCPP_INFO(this->_node->get_logger(), "%f, %f, %f", now_pos.x, now_pos.y, now_pos.rad);

        pos_t disp;
        disp.x = 0.5;
        disp.y = 0.5;
        disp.rad = 5 * (M_PI/180);

        if(500 < _cut_counter)
        {
            for(size_t i = 0; i < MCLTYPE_N; i++)
            {
                pos_t pos = pf_pos[i];
                float w_x = exp(-((pos.x - now_pos.x) * (pos.x - now_pos.x)) / (2 * (disp.x) * (disp.x)));
                float w_y = exp(-((pos.y - now_pos.y) * (pos.y - now_pos.y)) / (2 * disp.y * disp.y));
                float w_rad = exp(-((pos.rad - now_pos.rad) * (pos.rad - now_pos.rad)) / (2 * disp.rad * disp.rad));
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

    void observation(rclcpp::Time pf_tim, pos_t obs, pos_t pf_disp)
    {
        pos_t d = _delta.delta(pf_tim, true);
        pos_t abs_d ={
            d.x * cos(_x.rad) + d.y * -sin(_x.rad),
            d.x * sin(_x.rad) + d.y * cos(_x.rad),
            d.rad
        };

        pos_t befor_x = _x;

        pos_t x_odo;
        x_odo.x = this->_x.x + abs_d.x;
        x_odo.y = this->_x.y + abs_d.y;
        x_odo.rad = this->_x.rad + abs_d.rad;

        this->_x_disp.x += this->_x_noise.x + fabs(abs_d.x) * 1.0E-4;
        this->_x_disp.y += this->_x_noise.y + fabs(abs_d.y) * 1.0E-4;
        this->_x_disp.rad += this->_x_noise.rad + fabs(abs_d.rad) * 1.0E-4;
        pos_t odom_disp = this->_x_disp;

        pos_t k;
        if(500 < _cut_counter)
        {
            k.x = this->_x_disp.x / (this->_x_disp.x + pf_disp.x);
            k.y = this->_x_disp.y / (this->_x_disp.y + pf_disp.y);
            k.rad = this->_x_disp.rad / (this->_x_disp.rad + pf_disp.rad);

            if(0.02 < k.x){
                k.x = 0.02;
            }

            if(0.02 < k.y){
                k.y = 0.02;
            }

            if(0.02 < k.rad){
                k.rad = 0.02;
            }

            
            k.x *= is_enable;
            k.y *= is_enable;
            k.rad *= is_enable;

            // is_enable = 0;
        }
        else
        {
            k.x = 0.02;
            k.y = 0.02;
            k.rad = 0.02;
        }
        
        this->_x.x = (1 - k.x) * x_odo.x + k.x * obs.x;
        this->_x.y = (1 - k.y) * x_odo.y + k.y * obs.y;
        this->_x.rad = (1 - k.rad) * x_odo.rad + k.rad * obs.rad;

        this->_x_disp.x = (1 - k.x) * this->_x_disp.x;
        this->_x_disp.y = (1 - k.y) * this->_x_disp.y;
        this->_x_disp.rad = (1 - k.rad) * this->_x_disp.rad;

        befor_x.x = _x.x - befor_x.x;
        befor_x.y = _x.y - befor_x.y;
        befor_x.rad = _x.rad - befor_x.rad;

        std_msgs::msg::Float32MultiArray rec_msg;
        rec_msg.data = {(float)k.x, (float)k.y, (float)k.rad, (float)pf_disp.x, (float)pf_disp.y, (float)pf_disp.rad, (float)odom_disp.x, (float)odom_disp.y, (float)odom_disp.rad};
        _gain_record.record(rec_msg);
    }

    pos_t predict_pos(rclcpp::Time sens_tim)
    {
        pos_t d = _delta.delta(sens_tim, false);
        pos_t abs_d ={
            d.x * cos(_x.rad) + d.y * -sin(_x.rad),
            d.x * sin(_x.rad) + d.y * cos(_x.rad),
            d.rad
        };

        pos_t now_pos;
        now_pos.x = this->_x.x + abs_d.x;
        now_pos.y = this->_x.y + abs_d.y;
        now_pos.rad = this->_x.rad + abs_d.rad; 
        // TAGGER_INFO(_tagger, "delta", "(%f, %f, %f), (%f, %f, %f)", _x.x, _x.y, _x.rad, _delta.x, _delta.y, _delta.rad);
        return now_pos;
    }
};

}