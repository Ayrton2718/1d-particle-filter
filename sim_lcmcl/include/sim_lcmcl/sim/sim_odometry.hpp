#pragma once

#include <cmath>

#include "sim_type.hpp"
#include <lcmcl_msgs/msg/odometry.hpp>
#include <lc_map/lc_tf.hpp>
#include <lc_map/lc_area.hpp>
#include <tut_tool/tt_timer.hpp>

namespace sim
{

class Odometry : private Area
{
private:
    blackbox::BlackBoxNode* _node;

    std::random_device          _gn_seed_gen;
    std::default_random_engine  _gn_engine;

    tut_tool::RealTimer _tim;

    blackbox::Logger    _info;

    pos_t _enc;
    pos_t _odom;

    pos_t _befor_pos;
    pos_t _befor_enc;

    pos_t    _tire_lf;
    pos_t    _tire_rf;
    pos_t    _tire_lb;
    pos_t    _tire_rb;

public:
    Odometry(void) : Area(), _gn_engine(_gn_seed_gen())
    {
    }
    
    void init(blackbox::BlackBoxNode* node, pos_t initial_pos, std::shared_ptr<Tf> tf)
    {
        this->_node = node;

        this->_odom = initial_pos;
        this->_befor_pos = initial_pos;

        this->_enc = {0, 0, 0};
        this->_befor_enc = this->_enc;

        _tire_lf = tf->get_tf("tire_lf").pos;
        _tire_rf = tf->get_tf("tire_rf").pos;
        _tire_lb = tf->get_tf("tire_lb").pos;
        _tire_rb = tf->get_tf("tire_rb").pos;

        _tim.reset();

        _info.init(node, blackbox::INFO, "encoder");
    }

    std::tuple<lcmcl_msgs::msg::Odometry, float, float> sim(pos_t true_pos)
    {
        // sim
        pos_t pos_delta;
        pos_delta.x = true_pos.x - _befor_pos.x;
        pos_delta.y = true_pos.y - _befor_pos.y;
        pos_delta.rad = true_pos.rad - _befor_pos.rad;

        if(is_area(area_t::slope1, true_pos) || is_area(area_t::slope2, true_pos)){
            pos_delta.x = pos_delta.x * (100.5 / 100);
            pos_delta.y = pos_delta.y;
        }else if(is_area(area_t::slope3, true_pos)){
            pos_delta.x = pos_delta.x;
            pos_delta.y = pos_delta.y* (100.5 / 100);
        }else{
            pos_delta.x = pos_delta.x;
            pos_delta.y = pos_delta.y;
        }

        double d_e_x = pos_delta.x * cos(true_pos.rad) + pos_delta.y * sin(true_pos.rad);
        double d_e_y = pos_delta.x * -sin(true_pos.rad) + pos_delta.y * cos(true_pos.rad);
        double d_e_rad = pos_delta.rad;

        std::normal_distribution<double> dist(0, 0.1);
        std::normal_distribution<double> dist_theta(0, 0.15);
        std::normal_distribution<double> white_theta(0, 0.1 *(M_PI / 180));
        _enc.x += d_e_x +(dist(_gn_engine) * d_e_x);
        _enc.y += d_e_y + (dist(_gn_engine) * d_e_y);
        _enc.rad += d_e_rad + (dist_theta(_gn_engine) * d_e_rad) + white_theta(_gn_engine);

        // localization
        pos_t enc_delta;
        enc_delta.x = _enc.x - _befor_enc.x;
        enc_delta.y = _enc.y - _befor_enc.y;
        enc_delta.rad = _enc.rad - _befor_enc.rad;

        _odom.x += enc_delta.x * cos(_enc.rad) - enc_delta.y * sin(_enc.rad);
        _odom.y += enc_delta.x * sin(_enc.rad) + enc_delta.y * cos(_enc.rad);
        _odom.rad += enc_delta.rad;

        _befor_pos = true_pos;
        _befor_enc = _enc;

        float lf_height = this->height_real(lc::pos_transformer(true_pos, _tire_lf));
        float rf_height = this->height_real(lc::pos_transformer(true_pos, _tire_rf));
        float lb_height = this->height_real(lc::pos_transformer(true_pos, _tire_lb));
        float rb_height = this->height_real(lc::pos_transformer(true_pos, _tire_rb));
        float roll_diff = ((lf_height + lb_height) / 2) - ((rf_height + rb_height) / 2);
        float roll_length = fabs(_tire_lf.y) + fabs(_tire_rf.y);
        float pitch_diff = ((lf_height + rf_height) / 2) - ((lb_height + rb_height) / 2);
        float pitch_length = fabs(_tire_lf.x) + fabs(_tire_lb.x);
        float roll = atan2f(roll_diff, roll_length);
        float pitch = atan2f(pitch_diff, pitch_length);

        lcmcl_msgs::msg::Odometry msg;
        msg.rel_x = _enc.x;
        msg.rel_y = _enc.y;
        msg.rel_rad = _enc.rad;
        msg.rel_vx = enc_delta.x / _tim.getSec();
        msg.rel_vy = enc_delta.y / _tim.getSec();
        msg.rel_vrad = enc_delta.rad / _tim.getSec();
        _tim.reset();

        // TAGGER_INFO(_tagger, "encoder", "(%f, %f, %f), (%f, %f, %f)", odom.abs.x, odom.abs.y, odom.abs.rad, odom.rel.x, odom.rel.y, odom.rel.rad);

        return std::make_tuple(msg, roll, pitch);
    }

};

}