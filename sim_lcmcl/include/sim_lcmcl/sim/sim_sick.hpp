#pragma once

#include <cmath>
#include <random>

#include "sim_type.hpp"

#include <lc_map/lc_tf.hpp>

namespace sim
{
class Sick
{
private:
    blackbox::BlackBoxNode* _node;
    blackbox::Logger        _info;

    std::random_device          _gn_seed_gen;
    std::default_random_engine  _gn_engine;

    std::shared_ptr<Map>                 _map;

    struct range_info_t{
        std::string link_name;
        lc::pos_t   pos;
        float       range;
        blackbox::PubRecord<sensor_msgs::msg::Range> pub;
    };

    blackbox::Param<std::vector<std::string>>   _laser_topics;
    std::vector<std::shared_ptr<range_info_t>>  _laser_info;

public:
    Sick() : _gn_engine(_gn_seed_gen())
    {
    }

    void init(blackbox::BlackBoxNode* node, std::shared_ptr<Map> map, std::shared_ptr<lc::Tf> tf)
    {
        this->_node = node;
        this->_map = map;
        _info.init(node, blackbox::LIB_DEBUG, "sick");

        _laser_topics.init(node, "range_topics", {"laser1", "laser2", "laser3", "laser4"});

        auto topics = _laser_topics.get();
        for(auto topic : topics)
        {
            std::string link_name = topic;
            std::string prefix = "/waffle_1d/";
            if (topic.find(prefix) == 0) {
                link_name = link_name.substr(prefix.length());
            }

            std::string target = "_range";
            std::string replacement = "_link";
            size_t pos = link_name.find(target);
            if (pos != std::string::npos) {
                link_name.replace(pos, target.length(), replacement);
            }


            auto info = std::make_shared<range_info_t>();
            info->link_name = link_name;
            info->pos = tf->get_tf(link_name).pos;
            info->range = 0;
            info->pub.init(node, topic, rclcpp::QoS(1).reliable());

            _laser_info.push_back(info);

            TAGGER(_info, "laser topic, %s, %s", topic.c_str(), link_name.c_str());
        }
    }

    void publish(pos_t pos)
    {
        std::normal_distribution<double> dist(0, 0.001);

        for(auto laser : _laser_info)
        {
            pos_t local = lc::pos_transformer(pos, laser->pos);
        
            Map::laser_t hit = _map->calculate_range(local, 30);
            hit.range += dist(_gn_engine);
            if(hit.hit_type == -1)
            {
                hit.range = 0;
            }

            laser->range = hit.range;

            sensor_msgs::msg::Range msg;
            msg.header.stamp = _node->now();
            msg.header.frame_id = laser->link_name;
            msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
            msg.max_range = 30.1;
            msg.min_range = 0;
            msg.field_of_view = 0.5 * M_PI / 180;
            msg.range = hit.range;
            laser->pub.publish(msg);
        }
    }

};

}