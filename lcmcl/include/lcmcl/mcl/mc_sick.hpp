#pragma once

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <random>

#include <lcmcl_msgs/msg/laser.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <sensor_msgs/msg/range.hpp>

#include "mc_type.hpp"
#include "mc_sens_base.hpp"

#include <lc_map/lc_area.hpp>
#include <lcmcl_topics/lcmcl_laser.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

namespace mcl
{

using namespace lcmcl_topics;

class Sick : public SensBase, Area
{
private:
    blackbox::BlackBoxNode* _node;

    blackbox::Logger    _sick_info;

    std::shared_ptr<lc::Map>    _map;

    const float             _lambda_short = 0.5;
    const float             _z_short = 0.03;

    int                                 _ignore_index;
    int                                 _area1_hill_index;
    int                                 _area2_hill_index;
    int                                 _area3_wall_index;
    std::array<std::vector<int>, 6>     _target_obs;
    std::vector<int>                    _target_obs100;

    pos_t    _tire_lf;
    pos_t    _tire_rf;
    pos_t    _tire_lb;
    pos_t    _tire_rb;

    LaserMsg _laser_msg;
    std::array<rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr, lcmcl_topics::LaserMsg::size>     _laser_pub;
    std::array<rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr, lcmcl_topics::LaserMsg::size100>  _laser100_pub;

    blackbox::SubRecord<lcmcl_msgs::msg::Laser, true> _sub;
    blackbox::Record<std_msgs::msg::Float32MultiArray, false, true> _weight_record;

public:
    Sick(blackbox::BlackBoxNode* node, std::shared_ptr<lc::Map> map, std::shared_ptr<Tf> tf, std::function<void(rclcpp::Time sens_tim)> sens_cb) : SensBase(sens_cb), Area()
    {
        this->_node = node;
        this->_map = map;

        _sick_info.init(node, blackbox::LIB_DEBUG, "sick");

        this->_ignore_index = this->_map->get_obs_index("ignore");
        this->_area1_hill_index = this->_map->get_obs_index("area1_hill");
        this->_area2_hill_index = this->_map->get_obs_index("area2_hill");
        this->_area3_wall_index = this->_map->get_obs_index("area3_wall");

        _target_obs[height_index(height_type_t::height_0)].push_back(map->get_obs_index("area1_wall"));
        _target_obs[height_index(height_type_t::height_0)].push_back(map->get_obs_index("area1_hill"));

        _target_obs[height_index(height_type_t::height_100)].push_back(map->get_obs_index("area2_wall"));
        _target_obs[height_index(height_type_t::height_100)].push_back(map->get_obs_index("area2_hill"));
        _target_obs[height_index(height_type_t::height_100)].push_back(map->get_obs_index("area23_wall"));

        _target_obs[height_index(height_type_t::height_200)].push_back(map->get_obs_index("area23_wall"));
        _target_obs[height_index(height_type_t::height_200)].push_back(map->get_obs_index("area3_wall"));
        _target_obs[height_index(height_type_t::height_200)].push_back(map->get_obs_index("silo_wall"));

        _target_obs[height_index(height_type_t::height_slope1)].push_back(map->get_obs_index("area1_wall"));
        _target_obs[height_index(height_type_t::height_slope1)].push_back(map->get_obs_index("area2_wall"));
        _target_obs[height_index(height_type_t::height_slope1)].push_back(map->get_obs_index("area23_wall"));

        _target_obs[height_index(height_type_t::height_slope2)].push_back(map->get_obs_index("area2_wall"));
        _target_obs[height_index(height_type_t::height_slope2)].push_back(map->get_obs_index("area23_wall"));
        _target_obs[height_index(height_type_t::height_slope2)].push_back(map->get_obs_index("area3_wall"));

        _target_obs[height_index(height_type_t::height_slope3)].push_back(map->get_obs_index("area23_wall"));
        _target_obs[height_index(height_type_t::height_slope3)].push_back(map->get_obs_index("area3_wall"));
        _target_obs[height_index(height_type_t::height_slope3)].push_back(map->get_obs_index("silo_wall"));

        _target_obs100.push_back(map->get_obs_index("area23_wall"));
        _target_obs100.push_back(map->get_obs_index("area3_wall"));
        _target_obs100.push_back(map->get_obs_index("silo_wall"));

        _tire_lf = tf->get_tf("tire_lf").pos;
        _tire_rf = tf->get_tf("tire_rf").pos;
        _tire_lb = tf->get_tf("tire_lb").pos;
        _tire_rb = tf->get_tf("tire_rb").pos;

        _laser_msg.init(tf);

        _weight_record.init(node, "sick_weight", 0);

        for(size_t i = 0; i < _laser_msg.size; i++)
        {
            _laser_pub[i] = node->create_publisher<sensor_msgs::msg::Range>(_laser_msg.get_tf_array()[i], rclcpp::SensorDataQoS());
        }

        for(size_t i = 0; i < _laser_msg.size100; i++)
        {
            _laser100_pub[i] = node->create_publisher<sensor_msgs::msg::Range>(_laser_msg.get_tf_array100()[i], rclcpp::SensorDataQoS());
        }

        _sub.init(node, "sensor_laser", rclcpp::QoS(1).reliable(), [this](const lcmcl_msgs::msg::Laser::SharedPtr msg) {
                    _laser_msg.input_msg(msg.get());
                    this->sensor_callback(msg->header.stamp);

                    sensor_msgs::msg::Range send_msg;
                    send_msg.header.stamp = msg->header.stamp;
                    send_msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
                    send_msg.max_range = 30.1;
                    send_msg.min_range = 0;
                    send_msg.field_of_view = 0.5 * M_PI / 180;
                    for(size_t i = 0; i < _laser_msg.size; i++)
                    {
                        send_msg.header.frame_id = _laser_msg.get_tf_array()[i];
                        send_msg.range = _laser_msg._laser[i];
                        _laser_pub[i]->publish(send_msg);
                    }
                    for(size_t i = 0; i < _laser_msg.size100; i++)
                    {
                        send_msg.header.frame_id = _laser_msg.get_tf_array100()[i];
                        send_msg.range = _laser_msg._laser100[i];
                        _laser100_pub[i]->publish(send_msg);
                    }
                });
    }


    std::array<double, MCLTYPE_N> calc_weight(std::array<lc::pos_t, MCLTYPE_N> pf_pos)
    {
        std::array<double, MCLTYPE_N> weight_list;
        double weight_avg = 0;
        double max_weight = 0;

        for(size_t i = 0; i < MCLTYPE_N; i++)
        {
            double weight = 1;  

            pos_t pos = pf_pos[i];
            height_type_t ht = this->height_type(pos);
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

            // 判定キモい
            // ht != height_type_t::height_unknown : 平地かどうか
            // 15 < : スロープかどうか
            if((ht != height_type_t::height_unknown) || (15 < fabs(rad2deg(roll))) || (15 < fabs(rad2deg(pitch))))
            {
                float base_height = 0.0;
                if(ht == height_type_t::height_slope1 || ht == height_type_t::height_0)
                {
                    base_height = 0.0;
                }else if(ht == height_type_t::height_slope2 || ht == height_type_t::height_slope3 || ht == height_type_t::height_100){
                    base_height = 0.1;
                }else if(ht == height_type_t::height_200){
                    base_height = 0.2;
                }
                
                for(size_t i = 0; i < _laser_msg.size; i++)
                {
                    double pz = 0;

                    trans_t trans;
                    trans.pos = lc::pos_transformer(pos, this->_laser_msg._pos[i]);
                    trans.pitch = pitch;
                    trans.roll = roll;
                    trans.z = this->height_real(trans.pos) + 0.05;
                    
                    Map::laser_t laser;
                    if(ht == height_type_t::height_0 || ht == height_type_t::height_100 || ht == height_type_t::height_200)
                    {   
                        laser = _map->sick(trans.pos, &_target_obs[height_index(ht)], 30);
                    }
                    else if(ht == height_type_t::height_slope1 || ht == height_type_t::height_slope2 || ht == height_type_t::height_slope3)
                    {
                        laser = _map->sick_slope(trans, pos.rad, base_height, &_target_obs[height_index(ht)], 30);
                    }else{
                        laser.hit_type = -1;
                    }
                    
                    if(laser.hit_type != -1 && laser.hit_type != _ignore_index && _laser_msg._laser[i] != 0)
                    {
                        double z = _laser_msg._laser[i] - laser.range;
                        if(laser.hit_type == this->_area1_hill_index || laser.hit_type == this->_area2_hill_index)
                        {
                            if(fabs(z) < 0.2){
                                double z = 0.3;
                                const float dist1 = 0.05;
                                pz += exp(-(z * z) / (2 * dist1 * dist1)) * 0.89;
                                const float dist2 = 0.15;
                                pz += exp(-(z * z) / (2 * dist2 * dist2)) * 0.5; 
                            }
                        }
                        else
                        {
                            if(_laser_msg._laser[i] < 0.1){
                                const float dist1 = 0.01;
                                pz += exp(-(z * z) / (2 * dist1 * dist1)) * 1.39;
                                // const float dist2 = 0.15;
                                // pz += exp(-(z * z) / (2 * dist2 * dist2)) * 0.5;
                            }else if(_laser_msg._laser[i] < 4)
                            {
                                const float dist1 = 0.05;
                                pz += exp(-(z * z) / (2 * dist1 * dist1)) * 0.89;
                                const float dist2 = 0.15;
                                pz += exp(-(z * z) / (2 * dist2 * dist2)) * 0.5;
                            }else{
                                const float dist1 = 0.1;
                                pz += exp(-(z * z) / (2 * dist1 * dist1)) * 0.01;
                            }
                        }

                        // if(is_area(area_t::area1, pos) || is_area(area_t::area2, pos))
                        //     pz += _z_short * _lambda_short * exp(-_lambda_short * _laser_msg._laser[i]);
                        pz += 0.01;
                    }
                    else
                    {
                        double z = 0.3;
                        const float dist1 = 0.05;
                        pz += exp(-(z * z) / (2 * dist1 * dist1)) * 0.89;
                        const float dist2 = 0.15;
                        pz += exp(-(z * z) / (2 * dist2 * dist2)) * 0.5;
                    }

                    weight *= pz;
                }

                for(size_t i = 0; i < _laser_msg.size100; i++)
                {
                    double pz = 0;
                    pos_t trans_pos = lc::pos_transformer(pos, this->_laser_msg._pos100[i]);

                    Map::laser_t laser;
                    if(i == (size_t)LaserMsg::laser100_name_t::laser_100r || i == (size_t)LaserMsg::laser100_name_t::laser_100l)
                    {
                        if(ht == height_type_t::height_100 && is_area(area_t::storage, pos))
                        {   
                            laser = _map->sick(trans_pos, &_target_obs100, 30);
                        }else{
                            laser.hit_type = -1;
                        }
                    }else if(i == (size_t)LaserMsg::laser100_name_t::laser_100fr || i == (size_t)LaserMsg::laser100_name_t::laser_100fl){
                        if(ht == height_type_t::height_100 && is_area(area_t::area2, pos))
                        {
                            laser = _map->sick(trans_pos, &_target_obs100, 30);
                            if(_laser_msg._laser100[i] < 1.0 || 4.0 < _laser_msg._laser100[i]){
                                _laser_msg._laser100[i] = 0;
                            }
                        }else{
                            laser.hit_type = -1;
                        }
                    }else{
                        laser.hit_type = -1;
                    }
                    
                    if(laser.hit_type != -1 && laser.hit_type != _ignore_index && _laser_msg._laser100[i] != 0)
                    {
                        double z = _laser_msg._laser100[i] - laser.range;
                        if(_laser_msg._laser100[i] < 4)
                        {
                            const float dist1 = 0.05;
                            pz += exp(-(z * z) / (2 * dist1 * dist1)) * 0.89;
                            const float dist2 = 0.15;
                            pz += exp(-(z * z) / (2 * dist2 * dist2)) * 0.5;
                        }else{
                            const float dist1 = 0.1;
                            pz += exp(-(z * z) / (2 * dist1 * dist1)) * 0.01;
                        }

                        // if(z < -0.15 && (_laser_msg._laser100[i] != 0))
                        //     pz += _z_short * _lambda_short * exp(-_lambda_short * _laser_msg._laser100[i]);
                        pz += 0.01;
                    }
                    else
                    {
                        double z = 0.3;
                        const float dist1 = 0.05;
                        pz += exp(-(z * z) / (2 * dist1 * dist1)) * 0.89;
                        const float dist2 = 0.15;
                        pz += exp(-(z * z) / (2 * dist2 * dist2)) * 0.5;
                    }
                    weight *= pz;
                }
            }
            else
            {
                weight = FLT_MIN;
            }

            weight_avg += weight;
            if(max_weight < weight)
            {
                max_weight = weight;
            }

            weight_list[i] = weight;
            // TAGGER_INFO(_tagger, "sick", "%f", weight);
        }

        weight_avg /= MCLTYPE_N;

        float linear_start = 1.0e-6;
        float linear_end = 1.0e-12;
        float max_T = 60.0;
        double T;
        if(max_weight < linear_end)
        {
            T = max_T;
            double denom = 0;
            for(size_t i = 0; i < MCLTYPE_N; i++)
            {
                denom += pow(weight_list[i], 1 / T);
            }
            for(size_t i = 0; i < MCLTYPE_N; i++)
            {
                weight_list[i] = pow(weight_list[i], 1/T) / denom;
            }
        }
        else if(max_weight < linear_start)
        {
            double rate = (logf(max_weight) - logf(linear_start)) / (logf(linear_end) - logf(linear_start));
            if(rate < 0)
            {
                rate = 0;
            }
            T = (max_T * rate) + 1;
            double denom = 0;
            for(size_t i = 0; i < MCLTYPE_N; i++)
            {
                denom += pow(weight_list[i], 1 / T);
            }
            for(size_t i = 0; i < MCLTYPE_N; i++)
            {
                weight_list[i] = pow(weight_list[i], 1/T) / denom;
            }
        }else{
            T = 1.0;
            double denom = 0;
            for(size_t i = 0; i < MCLTYPE_N; i++)
            {
                denom += pow(weight_list[i], 1 / T);
            }
            for(size_t i = 0; i < MCLTYPE_N; i++)
            {
                weight_list[i] = pow(weight_list[i], 1/T) / denom;
            }
        }

        std_msgs::msg::Float32MultiArray rec_msg;
        rec_msg.data = {(float)weight_avg, (float)max_weight, (float)T};
        _weight_record.record(rec_msg);
        // TAGGER(_sick_info, "weight_max/avg := %le/%le T=%le", weight_avg, max_weight, T);

        return weight_list;
    }
};

}