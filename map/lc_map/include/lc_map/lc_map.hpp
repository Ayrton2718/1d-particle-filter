#pragma once

// line crossing determination
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/callback_group.hpp>
#include <vector>
#include <cmath>
#include <float.h>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <blackbox/blackbox.hpp>

#include "lc_type.hpp"
#include "lc_tf.hpp"
#include <Eigen/Geometry>

namespace lc
{

class Map
{
public:
    typedef struct
    {
        int16_t     hit_type;
        float       range;
    } laser_t;

    typedef struct
    {
        float   x0;
        float   y0;
        float   vx;
        float   vy;
    } line_t;

    typedef struct
    {
        float   x0;
        float   y0;
        float   r;
    } circle_t;

    enum class zone_t{
        RED,
        BLUE,
        ALL,
        UNDEF
    };

    typedef std::vector<laser_t> scan_t;

private:
    typedef struct
    {
        std::string name;
        uint8_t     r;
        uint8_t     g;
        uint8_t     b;
    } line_type_t;

private:
    rclcpp::Node* _node;

    std::unique_ptr<blackbox::Logger>    _error;
    std::unique_ptr<blackbox::Logger>    _info;

    rclcpp::CallbackGroup::SharedPtr m_cb_grp1;
    rclcpp::TimerBase::SharedPtr                                        _map_pub_tim;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr  _map_pub;

    float _max_range;
    float _min_range;

    float       _max_pos[2];
    float       _min_pos[2];
    std::vector<line_type_t> _obs_types;
    std::vector<std::vector<line_t>>     _obs_lines;
    std::vector<std::vector<circle_t>>   _obs_circles;

private:
    void import_from_file(zone_t zone);
    void import_from_raw(size_t len, const char* bin, zone_t zone);

    int search_type_index(std::vector<line_type_t>* line_type, const char* name)
    {
        int i = 0;
        for(auto it = line_type->begin(); it < line_type->end(); it++, i++)
        {
            if(it->name.compare(name) == 0)
            {
                return i;
            }
        }
        
        if(_error)
            TAGGER(_error.get(), "Undefined type(%s)", name);
        return -1;
    }

public:
    Map(){}
    Map(blackbox::BlackBoxNode* node, std::shared_ptr<Tf> tf, float min_range = 0.1, float max_range = 10, zone_t zone=zone_t::UNDEF)
    {
        this->Map_cons(node, tf, min_range, max_range, zone);
    }

    void Map_cons(blackbox::BlackBoxNode* node, std::shared_ptr<Tf> tf, float min_range = 0, float max_range = 10, zone_t zone=zone_t::UNDEF)
    {
        this->_node = node;

        _error = std::make_unique<blackbox::Logger>();
        _info = std::make_unique<blackbox::Logger>();
        _error->init(node, blackbox::ERR, "lc_map");
        _info->init(node, blackbox::INFO, "lc_map");

        this->_min_range = min_range;
        this->_max_range = max_range;

        if(zone == zone_t::UNDEF)
        {
            if(0 <= tf->get_initial_pos().y){
                zone = zone_t::RED;
            }else{
                zone = zone_t::BLUE;
            }
        }
        
        this->import_from_file(zone);

        this->_map_pub = nullptr;
        this->m_cb_grp1 = nullptr;
        this->_map_pub_tim = NULL;
    }

    void Map_cons(rclcpp::Node* node)
    {
        this->_node = node;

        this->_min_range = 0;
        this->_max_range = 10;

        this->import_from_file(zone_t::ALL);

        this->_map_pub = nullptr;
        this->m_cb_grp1 = nullptr;
        this->_map_pub_tim = NULL;
    }

    void enable_publish_map(void)
    {
        this->_map_pub = this->_node->create_publisher<visualization_msgs::msg::MarkerArray>("line_map", rclcpp::SensorDataQoS());
        this->m_cb_grp1 = this->_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        this->_map_pub_tim = this->_node->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Map::map_publisher, this), this->m_cb_grp1);
    }

    int get_obs_index(const char* name)
    {
        return this->search_type_index(&this->_obs_types, name);
    }

    std::tuple<uint8_t, uint8_t, uint8_t> get_obs_rgb(const char* name)
    {
        int i = this->get_obs_index(name);
        if(i < 0){
            return std::tuple<uint8_t, uint8_t, uint8_t>(0, 0, 0);
        }
        return std::tuple<uint8_t, uint8_t, uint8_t>(this->_obs_types[i].r, this->_obs_types[i].g, this->_obs_types[i].b);
    }


    scan_t scan(pos_t pos, const std::vector<float>* rads, const std::vector<int>* obs, float range_max = FLT_MAX)
    {
        scan_t results;

        for(auto rad_it = rads->begin(); rad_it < rads->end(); rad_it++)
        {
            float dir_x = cosf(*rad_it + pos.rad) * -1;
            float dir_y = sinf(*rad_it + pos.rad) * -1;

            laser_t laser;
            laser.hit_type = -1;
            laser.range = range_max;
            for(auto type_it = obs->begin(); type_it != obs->end(); type_it++)
            {
                for(auto it = this->_obs_lines[*type_it].begin(); it < this->_obs_lines[*type_it].end(); it++)
                {
                    float d_x = pos.x - it->x0;
                    float d_y = pos.y - it->y0;

                    float donom = it->vx * dir_y - it->vy * dir_x;
                    if(10e-5 < fabs(donom))
                    {
                        float u = (d_x * dir_y - d_y * dir_x) / donom;
                        if((0 <= u) && (u <= 1))
                        {
                            float t = (it->vx * d_y - it->vy * d_x) / donom;
                            if((0 < t) && (t < laser.range))
                            {
                                laser.hit_type = *type_it;
                                laser.range = t;
                            }
                        }
                    }
                }
            }

            results.push_back(laser);
        }

        return results;
    }

    laser_t sick(pos_t pos, const std::vector<int>* obs, float range_max = FLT_MAX)
    {
        float dir_x = cosf(pos.rad) * -1;
        float dir_y = sinf(pos.rad) * -1;

        laser_t laser;
        laser.hit_type = -1;
        laser.range = range_max;
        for(auto type_it = obs->begin(); type_it != obs->end(); type_it++)
        {
            for(auto it = this->_obs_lines[*type_it].begin(); it < this->_obs_lines[*type_it].end(); it++)
            {
                float d_x = pos.x - it->x0;
                float d_y = pos.y - it->y0;

                float donom = it->vx * dir_y - it->vy * dir_x;
                if(10e-5 < fabs(donom))
                {
                    float u = (d_x * dir_y - d_y * dir_x) / donom;
                    if((0 <= u) && (u <= 1))
                    {
                        float t = (it->vx * d_y - it->vy * d_x) / donom;
                        if((0 < t) && (t < laser.range))
                        {
                            laser.hit_type = *type_it;
                            laser.range = t;
                        }
                    }
                }
            }
        }
        return laser;
    }

    laser_t sick_slope(trans_t trans, float base_rad, float base_height, const std::vector<int>* obs, float range_max = FLT_MAX)
    {
        laser_t laser = this->sick(trans.pos, obs, range_max);
        
        Eigen::Quaterniond current_orientation 
                            = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())
                            * Eigen::AngleAxisd(trans.pitch, Eigen::Vector3d::UnitY())
                            * Eigen::AngleAxisd(-trans.roll, Eigen::Vector3d::UnitX());

        Eigen::Quaterniond relative_rotation 
                            = Eigen::AngleAxisd(-(base_rad - trans.pos.rad), Eigen::Vector3d::UnitZ())
                            * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
                            * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
        
        Eigen::Quaterniond new_orientation = current_orientation * relative_rotation;
        // 新しい姿勢のロール、ピッチ、ヨーを取得
        auto new_orientation_euler = new_orientation.toRotationMatrix().eulerAngles(2, 1, 0);
        double new_pitch = new_orientation_euler[1];

        float height = trans.z - base_height;
        float range = height / (-sinf(new_pitch));
        if((laser.hit_type == -1) || (0 < range && range < laser.range))
        {
            laser.hit_type = 1;
            laser.range = range;
        }else if(sinf(atan2f(100.0f, 2625.0f)) < sinf(new_pitch)){
            laser.hit_type = -1;
            laser.range = 0;
        }
        return laser;
    }

    void map_publisher(void)
    {
        visualization_msgs::msg::MarkerArray markers;
        int32_t id = 0;
        rclcpp::Time stamp = this->_node->now();
        
        // TODO Use LINE_LIST and SPHERE_LIST.
        size_t type_i = 0;
        for(auto list_it = _obs_circles.begin(); list_it != _obs_circles.end(); list_it++, type_i++)
        {
            if(list_it->size() != 0)
            {
                visualization_msgs::msg::Marker mk;
                mk.header.frame_id = "map";
                mk.header.stamp = stamp;
                mk.ns = "line_map";
                mk.type = visualization_msgs::msg::Marker::SPHERE_LIST;
                mk.action = visualization_msgs::msg::Marker::ADD;
                mk.color.r = (float)this->_obs_types[type_i].r / 255;
                mk.color.g = (float)this->_obs_types[type_i].g / 255;
                mk.color.b = (float)this->_obs_types[type_i].b / 255;
                mk.color.a = 0.4;
                mk.pose.orientation.x = 0;
                mk.pose.orientation.y = 0;
                mk.pose.orientation.z = 0;
                mk.pose.orientation.w = 1.0;
                mk.pose.position.x = 0;
                mk.pose.position.y = 0;
                mk.pose.position.z = 0;
                mk.lifetime = rclcpp::Duration(0, 0);
                mk.id = id++;
                mk.points.clear();
                
                for(auto it = list_it->begin(); it < list_it->end(); it++)
                {
                    mk.scale.x = it->r * 2;
                    mk.scale.y = it->r * 2;
                    mk.scale.z = 0.000001;

                    // 球の位置を設定
                    geometry_msgs::msg::Point point;
                    point.x = it->x0;
                    point.y = it->y0;
                    point.z = 0;
                    mk.points.push_back(point);
                }

                markers.markers.push_back(mk);
            }
        }

        type_i = 0;
        for(auto list_it = _obs_lines.begin(); list_it != _obs_lines.end(); list_it++, type_i++)
        {
            if(list_it->size() != 0)
            {
                visualization_msgs::msg::Marker mk;
                mk.header.frame_id = "map";
                mk.header.stamp = stamp;
                mk.ns = "line_map";
                mk.type = visualization_msgs::msg::Marker::LINE_LIST;
                mk.action = visualization_msgs::msg::Marker::ADD;
                mk.scale.x = 0.01;
                mk.scale.y = 0.01;
                mk.scale.z = 0.01;
                mk.color.r = (float)this->_obs_types[type_i].r / 255;
                mk.color.g = (float)this->_obs_types[type_i].g / 255;
                mk.color.b = (float)this->_obs_types[type_i].b / 255;
                mk.color.a = 1;
                mk.pose.orientation.x = 0;
                mk.pose.orientation.y = 0;
                mk.pose.orientation.z = 0;
                mk.pose.orientation.w = 1.0;
                mk.pose.position.x = 0;
                mk.pose.position.y = 0;
                mk.pose.position.z = 0;
                mk.lifetime = rclcpp::Duration(0, 0);
                mk.id = id++;
                mk.points.clear();

                for(auto it = list_it->begin(); it < list_it->end(); it++)
                {
                    geometry_msgs::msg::Point point;
                    point.x = it->x0;
                    point.y = it->y0;
                    point.z = 0;
                    mk.points.push_back(point);
                    point.x = it->vx + it->x0;
                    point.y = it->vy + it->y0;
                    point.z = 0;
                    mk.points.push_back(point);
                }

                markers.markers.push_back(mk);
            }
        }

        this->_map_pub->publish(markers);
    }
};

}