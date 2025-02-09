#pragma once

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <random>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <sensor_msgs/msg/range.hpp>

#include "mc_type.hpp"
#include "mc_sens_base.hpp"

#include <std_msgs/msg/float32_multi_array.hpp>

namespace mcl
{


class Sick : public SensBase
{
private:
    blackbox::BlackBoxNode* _node;

    blackbox::Logger    _sick_info;

    std::shared_ptr<lc::Map>    _map;

    const float             _lambda_short = 0.5;
    const float             _z_short = 0.03;

    blackbox::Record<std_msgs::msg::Float32MultiArray, false, true> _weight_record;
    rclcpp::TimerBase::SharedPtr                            _callback_tim;

    struct range_info_t{
        bool    is_updated;
        lc::pos_t pos;
        float   range;
        blackbox::SubRecord<sensor_msgs::msg::Range, true> sub;
    };

    blackbox::Param<std::vector<std::string>>   _laser_topics;
    std::vector<std::shared_ptr<range_info_t>>  _laser_info;

public:
    Sick(blackbox::BlackBoxNode* node, std::shared_ptr<lc::Map> map, std::shared_ptr<Tf> tf, std::function<void(rclcpp::Time sens_tim)> sens_cb) : SensBase(sens_cb)
    {
        this->_node = node;
        this->_map = map;

        _sick_info.init(node, blackbox::LIB_DEBUG, "sick");

        _weight_record.init(node, "sick_weight", 0);

        _laser_topics.init(node, "range_topics", {"laser1", "laser2", "laser3", "laser4"});

        auto topics = _laser_topics.get();
        for(auto topic : topics)
        {
            auto info = std::make_shared<range_info_t>();
            info->is_updated = false;
            info->pos = {0, 0, 0};
            info->range = 0;
            info->sub.init(node, topic, rclcpp::QoS(1).reliable(), [this, info, tf](const sensor_msgs::msg::Range::SharedPtr msg) {
                    info->range = msg->range;

                    if(info->is_updated == false){
                        info->is_updated = true;
                        info->pos = tf->get_tf(msg->header.frame_id).pos;
                    }
                });

            _laser_info.push_back(info);

            TAGGER(_sick_info, "laser topic, %s", topic.c_str());
        }

        _callback_tim = node->create_wall_timer(std::chrono::milliseconds(10), [this](){
                this->sensor_callback(_node->get_clock()->now());
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

            // 判定キモい
            // ht != height_type_t::height_unknown : 平地かどうか
            // 15 < : スロープかどうか

            for(auto info : _laser_info)
            {
                if(info->is_updated)
                {
                    double pz = 0;

                    pos_t laser_pos = lc::pos_transformer(pos, info->pos);

                    Map::laser_t laser = _map->calculate_range(laser_pos, 30);
                    if(laser.hit_type != -1 && info->range != 0)
                    {
                        double z = info->range - laser.range;
                        if(info->range < 0.1){
                            const float dist1 = 0.01;
                            pz += exp(-(z * z) / (2 * dist1 * dist1)) * 1.39;
                            // const float dist2 = 0.15;
                            // pz += exp(-(z * z) / (2 * dist2 * dist2)) * 0.5;
                        }else if(info->range < 4)
                        {
                            const float dist1 = 0.05;
                            pz += exp(-(z * z) / (2 * dist1 * dist1)) * 0.89;
                            const float dist2 = 0.15;
                            pz += exp(-(z * z) / (2 * dist2 * dist2)) * 0.5;
                        }else{
                            const float dist1 = 0.1;
                            pz += exp(-(z * z) / (2 * dist1 * dist1)) * 0.01;
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