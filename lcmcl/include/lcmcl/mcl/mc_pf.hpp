#pragma once

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <random>

#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include "mc_type.hpp"
#include "mc_sens_base.hpp"
#include "mc_kf_base.hpp"
#include "mc_delta.hpp"

#define PF_VIEW_PARTICLE

namespace mcl
{

class Pf
{
private:
    typedef struct
    {
        pos_t pos;
        double weight;
    } particle_t;

private:
    blackbox::BlackBoxNode* _node;

    std::default_random_engine      _gn_engine;
    Delta                           _delta;
    mcl::SensBase*                  _sens;
    mcl::KfBase*                    _kf;
    
    blackbox::Logger    _weights_info;
    blackbox::Logger    _resampling_info;
    blackbox::Record<std_msgs::msg::Float32MultiArray, false, true> _result_record;

    std::array<particle_t, MCLTYPE_N>   _particle;
    std::array<double, MCLTYPE_N>       _outer_weight;

    rclcpp::Time                                    _last_tim;

    blackbox::Param<float> _pf_disp_x;
    blackbox::Param<float> _pf_disp_y;
    blackbox::Param<float> _pf_disp_yaw;

    pos_t _disp_delta0 = {0.03, 0.03, 0.25 * M_PI / 180};

    KfBase::mat_pos_t _est_pos;
    KfBase::mat_cov_t _est_cov;

#ifdef PF_VIEW_PARTICLE
    blackbox::PubRecord<visualization_msgs::msg::MarkerArray>   _part_pub;
#endif /*PF_VIEW_PARTICLE*/


    void calc_cov(void)
    {
        double weight_sum = 0.0;
        _est_cov = KfBase::mat_cov_t::Zero();

        for(size_t i = 0; i < MCLTYPE_N; i++)
        {
            KfBase::mat_pos_t diff;
            diff(0) = _particle[i].pos.x - _est_pos(0);
            diff(1) = _particle[i].pos.y - _est_pos(1);
            diff(2) = _particle[i].pos.rad - _est_pos(2);

            _est_cov += _outer_weight[i] * (diff * diff.transpose());
            weight_sum += _outer_weight[i];
        }

        if(weight_sum == 0)
        {
            weight_sum = FLT_MIN;
        }
        _est_cov /= weight_sum;
    }

    void calc_est(void)
    {
        double weight_sum = 0.0;
        _est_pos = KfBase::mat_pos_t::Zero();

        for(size_t i = 0; i < 10; i++)
        {
            _est_pos(0) += _particle[i].pos.x * _outer_weight[i];
            _est_pos(1) += _particle[i].pos.y * _outer_weight[i];
            _est_pos(2) += _particle[i].pos.rad * _outer_weight[i];
            weight_sum += _outer_weight[i];
        }
        
        if(weight_sum == 0)
        {
            weight_sum = FLT_MIN;
        }
        _est_pos /= weight_sum;
    }

    void move(rclcpp::Time pf_tim)
    {
        pos_t delta = _delta.delta(pf_tim, true);

        pos_t dist = {0, 0, 0};
        dist.x = this->_disp_delta0.x;
        dist.y = this->_disp_delta0.y;
        dist.rad = this->_disp_delta0.rad;
        

        double weight_sum = 0;
        std::normal_distribution<float> dist_x(0, dist.x);
        std::normal_distribution<float> dist_y(0, dist.y);
        std::normal_distribution<float> dist_theta(0, dist.rad);
        for(size_t i = 0; i < MCLTYPE_N; i++)
        {
            if(this->_particle[i].weight < FLT_MIN)
            {
                this->_particle[i].weight = FLT_MIN;
            }
            weight_sum += this->_particle[i].weight;
        }

        for(size_t i = 0; i < MCLTYPE_N; i++)
        {
            float rate = 1;// - (this->_particle[i].weight / weight_sum);

            pos_t dist_delta;
            dist_delta.x = delta.x + (dist_x(this->_gn_engine) * rate);
            dist_delta.y = delta.y + (dist_y(this->_gn_engine) * rate);
            dist_delta.rad = delta.rad + (dist_theta(this->_gn_engine) * rate);

            float p_rad = _particle[i].pos.rad + (dist_delta.rad / 2);
            this->_particle[i].pos.x += dist_delta.x * cos(p_rad) - dist_delta.y * sin(p_rad);
            this->_particle[i].pos.y += dist_delta.x * sin(p_rad) + dist_delta.y * cos(p_rad);
            this->_particle[i].pos.rad += dist_delta.rad;
            this->_particle[i].weight = 1;
            _outer_weight[i] = 1;
        }
    }

    void likelihood(rclcpp::Time pf_tim)
    {
        std::array<lc::pos_t, MCLTYPE_N> pf_pos;
        for(size_t i = 0; i < MCLTYPE_N; i++)
        {
            pf_pos[i].x = _particle[i].pos.x;
            pf_pos[i].y = _particle[i].pos.y;
            pf_pos[i].rad = _particle[i].pos.rad;
        }

        const std::array<double, MCLTYPE_N> sens_weight = _sens->calc_weight(pf_pos);
        const std::array<double, MCLTYPE_N> kf_weight = _kf->calc_weight(pf_tim, pf_pos);
        for(size_t i = 0; i < MCLTYPE_N; i++)
        {
            this->_particle[i].weight = sens_weight[i] * kf_weight[i];
        }
        _outer_weight = sens_weight;

        double weight_sum = 0;
        double outer_weight_sum = 0;
        for(size_t i = 0; i < MCLTYPE_N; i++)
        {
            if(this->_particle[i].weight < FLT_MIN || (this->_particle[i].weight != this->_particle[i].weight))
            {
                this->_particle[i].weight = FLT_MIN;
            }
            weight_sum += this->_particle[i].weight;

            if(_outer_weight[i] < FLT_MIN || (_outer_weight[i] != _outer_weight[i]))
            {
                _outer_weight[i] = FLT_MIN;
            }
            outer_weight_sum += _outer_weight[i];
        }

        for(size_t i = 0; i < MCLTYPE_N; i++)
        {
            this->_particle[i].weight /= weight_sum;
            _outer_weight[i] /= outer_weight_sum;
        }
    }

    void resampling(void)
    {
        std::array<particle_t, MCLTYPE_N> befor_particle(this->_particle);

        double T = 50.0;
        double denom = 0;
        for(size_t i = 0; i < MCLTYPE_N; i++)
        {
            denom += pow(befor_particle[i].weight, 1 / T);
        }
        for(size_t i = 0; i < MCLTYPE_N; i++)
        {
            befor_particle[i].weight = pow(befor_particle[i].weight, 1/T) / denom;
        }

        double count_inv = 1.0 / MCLTYPE_N;
        std::uniform_real_distribution<double> rand_dist(0, count_inv);
        double r = rand_dist(_gn_engine);
        double c = befor_particle[0].weight;
        size_t i = 0; 
        for(size_t m = 0; m < MCLTYPE_N; m++)
        {
            double u = r + m * count_inv;
            while(u > c)
            {
                i++;
                c += befor_particle[i].weight;
            }
            _particle[m].pos = befor_particle[i].pos;
            _particle[m].weight = 1;
        }
    }


    void view_particle(void)
    {
#ifdef PF_VIEW_PARTICLE
        size_t view_count = MCLTYPE_N;

        visualization_msgs::msg::MarkerArray marker_msg;
        marker_msg.markers.resize(view_count);

        geometry_msgs::msg::Vector3 arrow;  // config arrow shape
        arrow.x = 0.01;
        arrow.y = 0.02;
        arrow.z = 0.05;

        for(size_t i = 0; i < view_count; i++)
        {
            // RCLCPP_INFO(this->_node->get_logger(), "%f, %f, %f, %f", 
            //                 this->_particle[i].pos.x,
            //                 this->_particle[i].pos.y,
            //                 this->_particle[i].pos.rad,
            //                 this->_particle[i].weight);

            visualization_msgs::msg::Marker arrow;
            tf2::Quaternion q;
            arrow.header.frame_id = "map";
            arrow.header.stamp = this->_node->now();
            arrow.ns = "pf_particle";
            arrow.lifetime = rclcpp::Duration(0, 0);
            arrow.id = i;
            arrow.type = visualization_msgs::msg::Marker::ARROW;
            arrow.action = visualization_msgs::msg::Marker::ADD;
            arrow.pose.position.x = this->_particle[i].pos.x;
            arrow.pose.position.y = this->_particle[i].pos.y;
            arrow.pose.position.z = 0;
            q.setRPY(0, 0, this->_particle[i].pos.rad);
            arrow.pose.orientation.x = q.x();
            arrow.pose.orientation.y = q.y();
            arrow.pose.orientation.z = q.z();
            arrow.pose.orientation.w = q.w();
            arrow.scale.x = 0.1;
            arrow.scale.y = 0.01;
            arrow.scale.z = 0.01;

            if(i < view_count / 2)
            {
                arrow.color.r = 1.0;
                arrow.color.g = (i / ((float)view_count / 2));
            }else{
                arrow.color.r = 1.0 - ((i - (float)view_count / 2) / ((float)view_count / 2));
                arrow.color.g = 1.0;
            }
            arrow.color.b = 0.0;
            arrow.color.a = 1.0;

            marker_msg.markers[i] = arrow;
        }
        _part_pub.publish(marker_msg);
#endif /*PF_VIEW_PARTICLE*/
    }

public:
    Pf(blackbox::BlackBoxNode* node, OdomSubscriber* os, pos_t initial_pos, mcl::SensBase* sens, mcl::KfBase* kf) 
        : _gn_engine(100), _delta(os), _sens(sens), _kf(kf)
    {
        this->_node = node;

        _pf_disp_x.init(node, "pf.disp_x", 0.03);
        _pf_disp_y.init(node, "pf.disp_y", 0.03);
        _pf_disp_yaw.init(node, "pf.disp_yaw", 0.25);

        _disp_delta0.x = _pf_disp_x.get();
        _disp_delta0.y = _pf_disp_y.get();
        _disp_delta0.rad = _pf_disp_yaw.get() * M_PI / 180;

        _weights_info.init(_node, blackbox::LIB_DEBUG, "pf_weights");
        _resampling_info.init(_node, blackbox::LIB_DEBUG, "pf_resampling");

        _result_record.init(node, "pf_result");

        _last_tim = _node->now();

        std::normal_distribution<float> dist_x(initial_pos.x, _disp_delta0.x * 6);
        std::normal_distribution<float> dist_y(initial_pos.y, _disp_delta0.y * 6);
        std::normal_distribution<float> dist_theta(initial_pos.rad, _disp_delta0.rad * 2.5);

        float weight = (float)1 / (float)MCLTYPE_N;
        for(size_t i = 0; i < MCLTYPE_N; i++)
        {
            this->_particle[i].weight = weight;
            this->_particle[i].pos.x = dist_x(this->_gn_engine);
            this->_particle[i].pos.y = dist_y(this->_gn_engine);
            this->_particle[i].pos.rad = dist_theta(this->_gn_engine);
            _outer_weight[i] = weight;
        }

        _est_pos(0) = initial_pos.x;
        _est_pos(1) = initial_pos.y;
        _est_pos(2) = initial_pos.rad;

        _est_cov(0, 0) = 1.0E+10;
        _est_cov(1, 1) = 1.0E+10;
        _est_cov(2, 2) = 1.0E+10;

#ifdef PF_VIEW_PARTICLE
        _part_pub.init(node, "pf_particle", rclcpp::QoS(1));
#endif /*PF_VIEW_PARTICLE*/
    }

    KfBase::odom_t predict(rclcpp::Time sens_tim)
    {
        this->move(sens_tim);
        this->likelihood(sens_tim);
        this->calc_est();
        this->calc_cov();

        this->_last_tim = sens_tim;

        std::sort(this->_particle.begin(), this->_particle.end(),
            [](const Pf::particle_t &l, const Pf::particle_t &r) {
                return l.weight > r.weight;
            });
        this->view_particle();
        this->resampling();

        std_msgs::msg::Float32MultiArray rec_msg;
        rec_msg.data = {(float)_est_pos(0), (float)_est_pos(1), (float)_est_pos(2),
                        (float)_est_cov(0, 0), (float)_est_cov(1, 1), (float)_est_cov(2, 2)};
        _result_record.record(rec_msg);
        
        KfBase::odom_t result;
        result.tim = sens_tim;
        result.pos = _est_pos;
        result.cov = _est_cov;
        return result;
    }
};

}