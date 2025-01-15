#pragma once

#include "mc_pf.hpp"
#include "mc_ekf.hpp"
#include "mc_kf.hpp"
#include "mc_sick.hpp"

namespace mcl
{

class LPKf : private Area
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
    std::vector<int>                    _damper_obs;
    int     _damper_silo_wall;

    blackbox::PubRecord<lcmcl_msgs::msg::Localization>*  _est_pub;
    rclcpp::TimerBase::SharedPtr                        _timeout_tim;

public:
    LPKf(blackbox::BlackBoxNode* node, std::shared_ptr<lc::Tf> tf, std::shared_ptr<lc::Map> map, blackbox::PubRecord<lcmcl_msgs::msg::Localization>* est_pub)
        :   Area(),
            _os(node, std::bind(&LPKf::publish_localization, this, std::placeholders::_1)),
            _sick(node, map, tf, std::bind(&LPKf::sens_callback, this, std::placeholders::_1)),
            _kf(node, &_os, tf->get_initial_pos()),
            _pf(node, &_os, tf->get_initial_pos(), &_sick, &_kf)
    {
        _node = node;
        _tf = tf;
        _map = map;
        _est_pub = est_pub;

        _tire_lf = tf->get_tf("tire_lf").pos;
        _tire_rf = tf->get_tf("tire_rf").pos;
        _tire_lb = tf->get_tf("tire_lb").pos;
        _tire_rb = tf->get_tf("tire_rb").pos;

        _damper_r_tf =tf->get_tf("damper_r").pos;
        _damper_l_tf =tf->get_tf("damper_l").pos;
        _damper_obs.push_back(map->get_obs_index("area2_hill"));
        _damper_obs.push_back(map->get_obs_index("area3_wall"));
        _damper_obs.push_back(map->get_obs_index("silo_wall"));
        _damper_silo_wall = map->get_obs_index("silo_wall");

        _timeout_tim = node->create_wall_timer(std::chrono::milliseconds(10), [this](){
                this->sens_callback(_node->get_clock()->now());
            });
    }

    void sens_callback(rclcpp::Time sens_tim){
        _timeout_tim->reset();

        auto pf_obs = _pf.predict(sens_tim);
        _kf.observation(pf_obs);

        pos_t pos = _kf.predict_pos(std::get<0>(pf_obs));
        _tf->set_est(pos, std::get<0>(pf_obs));
    }

    void publish_localization(rclcpp::Time sens_tim){
        pos_t pos = _kf.predict_pos(sens_tim);
        float lf_height = this->height_real(lc::pos_transformer(pos, _tire_lf));
        float rf_height = this->height_real(lc::pos_transformer(pos, _tire_rf));
        float lb_height = this->height_real(lc::pos_transformer(pos, _tire_lb));
        float rb_height = this->height_real(lc::pos_transformer(pos, _tire_rb));
        float roll_diff = ((lf_height + lb_height) / 2) - ((rf_height + rb_height) / 2);
        float roll_length = fabs(_tire_lf.y) + fabs(_tire_rf.y);
        float pitch_diff = ((lf_height + rf_height) / 2) - ((lb_height + rb_height) / 2);
        float pitch_length = fabs(_tire_lf.x) + fabs(_tire_lb.x);
        float roll = atan2f(roll_diff, roll_length);
        float pitch = atan2f(pitch_diff, pitch_length);
        if((lf_height == 0) || (rf_height == 0) || (lb_height == 0) || (rb_height == 0)){
            roll = 0;
            pitch = 0;
        }

        if(std::isnan(pos.x) || std::isnan(pos.y) || std::isnan(pos.rad)){
            pos.x = 0;
            pos.y = 0;
            pos.rad = 0;
        }

        lcmcl_msgs::msg::Localization odom_est;
        tf2::Quaternion q;
        odom_est.header.frame_id = "map";
        odom_est.header.stamp = sens_tim;
        odom_est.abs_x = pos.x;
        odom_est.abs_y = pos.y;
        odom_est.abs_rad = pos.rad;
        odom_est.rel_pitch = pitch;
        odom_est.rel_roll = roll;

        Map::laser_t damper_r = _map->sick(lc::pos_transformer(pos, _damper_r_tf), &_damper_obs);
        if(damper_r.hit_type == _damper_silo_wall){
            odom_est.damper_r = damper_r.range;
        }else{
            odom_est.damper_r = -1;
        }

        Map::laser_t damper_l = _map->sick(lc::pos_transformer(pos, _damper_l_tf), &_damper_obs);
        if(damper_l.hit_type == _damper_silo_wall){
            odom_est.damper_l = damper_l.range;
        }else{
            odom_est.damper_l = -1;
        }
        
        _est_pub->publish(odom_est);
        // _tf->set_est(pos, odom_est.header.stamp);
    }

};

}