#pragma once

#include <cmath>
#include <random>

#include "sim_type.hpp"

#include <lcmcl_msgs/msg/laser.hpp>

#include <lc_map/lc_area.hpp>
#include <lc_map/lc_tf.hpp>
#include <lcmcl_topics/lcmcl_laser.hpp>

namespace sim
{
class Sick : private Area
{
private:
    static constexpr float sick_height = 0.05;

    blackbox::BlackBoxNode* _node;

    std::random_device          _gn_seed_gen;
    std::default_random_engine  _gn_engine;

    lcmcl_topics::LaserMsg _laser_msg;

    std::shared_ptr<Map>                 _map;
    std::array<std::vector<int>, 6>     _target_obs;
    std::vector<int>     _target_obs100;

public:
    Sick() : Area(), _gn_engine(_gn_seed_gen())
    {
    }

    void init(blackbox::BlackBoxNode* node, std::shared_ptr<Map> map, std::shared_ptr<lc::Tf> tf)
    {
        this->_node = node;
        this->_map = map;

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

        _laser_msg.init(tf);
    }

    lcmcl_msgs::msg::Laser sim(pos_t pos, float roll, float pitch)
    {
        std::normal_distribution<double> dist(0, 0.001);

        for(size_t i = 0; i < _laser_msg.size; i++)
        {
            pos_t local = lc::pos_transformer(pos, _laser_msg._pos[i]);
            height_type_t ht = this->height_type(local);
            if(ht == height_type_t::height_0 || ht == height_type_t::height_100 || ht == height_type_t::height_200)
            {   
                Map::laser_t hit = _map->sick(local, &_target_obs[height_index(ht)], 30);
                hit.range += dist(_gn_engine);
                if(hit.hit_type == -1)
                {
                    hit.range = 0;
                }

                _laser_msg._laser[i] = hit.range;
            }
            else if(ht == height_type_t::height_slope1 || ht == height_type_t::height_slope2 || ht == height_type_t::height_slope3)
            {
                trans_t trans;
                trans.pos = local;
                trans.z = this->height_real(local) + 0.05;
                trans.roll = roll;
                trans.pitch = pitch;

                float base_height;
                if(ht == height_type_t::height_slope1 || ht == height_type_t::height_0)
                {
                    base_height = 0.0;
                }else if(ht == height_type_t::height_slope2 || ht == height_type_t::height_slope3 || ht == height_type_t::height_100){
                    base_height = 0.1;
                }else if(ht == height_type_t::height_200){
                    base_height = 0.2;
                }
                
                // Map::laser_t hit = _map->sitck(local, &_target_obs[height_index(ht)], 30);
                Map::laser_t hit = _map->sick_slope(trans, pos.rad, base_height, &_target_obs[height_index(ht)], 30);
                hit.range += dist(_gn_engine);
                if(hit.hit_type == -1)
                {
                    hit.range = 0;
                }

                _laser_msg._laser[i] = hit.range;
            }else{
                _laser_msg._laser[i] = 0;
            }
        }

        for(size_t i = 0; i < _laser_msg.size100; i++)
        {
            pos_t local = lc::pos_transformer(pos, _laser_msg._pos100[i]);
            height_type_t ht = this->height_type(local);
            if(ht == height_type_t::height_100)
            {   
                Map::laser_t hit = _map->sick(local, &_target_obs100, 30);
                hit.range += dist(_gn_engine);
                if(hit.hit_type == -1)
                {
                    hit.range = 0;
                }

                _laser_msg._laser100[i] = hit.range;
            }else{
                _laser_msg._laser100[i] = 0;
            }
        }


        return _laser_msg.get_msg();
    }

};

}