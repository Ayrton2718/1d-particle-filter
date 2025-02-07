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

private:
    typedef struct {
        int occ_state; // Occupancy state (-1 = free, 0 = unknown, +1 = occupied)
        double occ_dist; // Distance to the nearest occupied cell
    } cell_t;

private:
    rclcpp::Node* _node;

    std::unique_ptr<blackbox::Logger>    _error;
    std::unique_ptr<blackbox::Logger>    _info;

    rclcpp::CallbackGroup::SharedPtr m_cb_grp1;
    rclcpp::TimerBase::SharedPtr                                        _map_pub_tim;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr  _map_pub;

    double origin_x, origin_y, origin_theta; // Map origin
    double resolution; // Map resolution (meters per cell)
    int size_x, size_y; // Map dimensions (number of cells)
    std::vector<cell_t> cells; // Grid data
    double max_occ_dist; // Maximum distance for obstacle consideration
    double occupied_thresh; // Threshold for occupied cells
    double free_thresh; // Threshold for free cells

private:
    void import_yaml(void);
    void import_pgm(void);

public:
    Map(){}
    Map(blackbox::BlackBoxNode* node)
    {
        this->Map_cons(node);
    }

    void Map_cons(blackbox::BlackBoxNode* node)
    {
        this->_node = node;

        _error = std::make_unique<blackbox::Logger>();
        _info = std::make_unique<blackbox::Logger>();
        _error->init(node, blackbox::ERR, "lc_map");
        _info->init(node, blackbox::INFO, "lc_map");

        this->origin_x = 0;
        this->origin_y = 0;
        this->origin_theta = 0;
        this->resolution = 0;
        this->size_x = 0;
        this->size_y = 0;
        this->max_occ_dist = 0;
        this->occupied_thresh = 0.65;
        this->free_thresh = 0.196;
        
        this->import_yaml();
        this->import_pgm();

        this->_map_pub = nullptr;
        this->m_cb_grp1 = nullptr;
        this->_map_pub_tim = NULL;
    }

    void Map_cons(rclcpp::Node* node)
    {
        this->_node = node;

        this->origin_x = 0;
        this->origin_y = 0;
        this->origin_theta = 0;
        this->resolution = 0;
        this->size_x = 0;
        this->size_y = 0;
        this->max_occ_dist = 0;
        this->occupied_thresh = 0.65;
        this->free_thresh = 0.196;

        this->import_yaml();
        this->import_pgm();

        this->_map_pub = nullptr;
        this->m_cb_grp1 = nullptr;
        this->_map_pub_tim = NULL;
    }

    void enable_publish_map(void)
    {
        // this->_map_pub = this->_node->create_publisher<visualization_msgs::msg::MarkerArray>("line_map", rclcpp::SensorDataQoS());
        // this->m_cb_grp1 = this->_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        // this->_map_pub_tim = this->_node->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Map::map_publisher, this), this->m_cb_grp1);
    }

    laser_t calculate_range(pos_t pos, float max_range)
    {
        int x0 = worldToMapX(pos.x);
        int y0 = worldToMapY(pos.y);
        int x1 = worldToMapX(pos.x + max_range * cos(pos.rad));
        int y1 = worldToMapY(pos.y + max_range * sin(pos.rad));

        bool steep = std::abs(y1 - y0) > std::abs(x1 - x0);
        if (steep) {
            std::swap(x0, y0);
            std::swap(x1, y1);
        }

        if (x0 > x1) {
            std::swap(x0, x1);
            std::swap(y0, y1);
        }

        int deltax = x1 - x0;
        int deltay = std::abs(y1 - y0);
        int error = 0;
        int ystep = (y0 < y1) ? 1 : -1;
        int y = y0;

        for (int x = x0; x <= x1; ++x) {
            int cx = steep ? y : x;
            int cy = steep ? x : y;

            if (!isValid(cx, cy) || getCell(cx, cy).occ_state > -1) {
                laser_t laser;
                laser.hit_type = 1;
                laser.range = std::sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0)) * resolution;
                return laser;
            }

            error += deltay;
            if (2 * error >= deltax) {
                y += ystep;
                error -= deltax;
            }
        }

        laser_t laser;
        laser.hit_type = -1;
        laser.range = max_range;
        return laser;
    }

    // void map_publisher(void)
    // {
    //     visualization_msgs::msg::MarkerArray markers;
    //     int32_t id = 0;
    //     rclcpp::Time stamp = this->_node->now();
        
    //     // TODO Use LINE_LIST and SPHERE_LIST.
    //     size_t type_i = 0;
    //     for(auto list_it = _obs_circles.begin(); list_it != _obs_circles.end(); list_it++, type_i++)
    //     {
    //         if(list_it->size() != 0)
    //         {
    //             visualization_msgs::msg::Marker mk;
    //             mk.header.frame_id = "map";
    //             mk.header.stamp = stamp;
    //             mk.ns = "line_map";
    //             mk.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    //             mk.action = visualization_msgs::msg::Marker::ADD;
    //             mk.color.r = (float)this->_obs_types[type_i].r / 255;
    //             mk.color.g = (float)this->_obs_types[type_i].g / 255;
    //             mk.color.b = (float)this->_obs_types[type_i].b / 255;
    //             mk.color.a = 0.4;
    //             mk.pose.orientation.x = 0;
    //             mk.pose.orientation.y = 0;
    //             mk.pose.orientation.z = 0;
    //             mk.pose.orientation.w = 1.0;
    //             mk.pose.position.x = 0;
    //             mk.pose.position.y = 0;
    //             mk.pose.position.z = 0;
    //             mk.lifetime = rclcpp::Duration(0, 0);
    //             mk.id = id++;
    //             mk.points.clear();
                
    //             for(auto it = list_it->begin(); it < list_it->end(); it++)
    //             {
    //                 mk.scale.x = it->r * 2;
    //                 mk.scale.y = it->r * 2;
    //                 mk.scale.z = 0.000001;

    //                 // 球の位置を設定
    //                 geometry_msgs::msg::Point point;
    //                 point.x = it->x0;
    //                 point.y = it->y0;
    //                 point.z = 0;
    //                 mk.points.push_back(point);
    //             }

    //             markers.markers.push_back(mk);
    //         }
    //     }

    //     type_i = 0;
    //     for(auto list_it = _obs_lines.begin(); list_it != _obs_lines.end(); list_it++, type_i++)
    //     {
    //         if(list_it->size() != 0)
    //         {
    //             visualization_msgs::msg::Marker mk;
    //             mk.header.frame_id = "map";
    //             mk.header.stamp = stamp;
    //             mk.ns = "line_map";
    //             mk.type = visualization_msgs::msg::Marker::LINE_LIST;
    //             mk.action = visualization_msgs::msg::Marker::ADD;
    //             mk.scale.x = 0.01;
    //             mk.scale.y = 0.01;
    //             mk.scale.z = 0.01;
    //             mk.color.r = (float)this->_obs_types[type_i].r / 255;
    //             mk.color.g = (float)this->_obs_types[type_i].g / 255;
    //             mk.color.b = (float)this->_obs_types[type_i].b / 255;
    //             mk.color.a = 1;
    //             mk.pose.orientation.x = 0;
    //             mk.pose.orientation.y = 0;
    //             mk.pose.orientation.z = 0;
    //             mk.pose.orientation.w = 1.0;
    //             mk.pose.position.x = 0;
    //             mk.pose.position.y = 0;
    //             mk.pose.position.z = 0;
    //             mk.lifetime = rclcpp::Duration(0, 0);
    //             mk.id = id++;
    //             mk.points.clear();

    //             for(auto it = list_it->begin(); it < list_it->end(); it++)
    //             {
    //                 geometry_msgs::msg::Point point;
    //                 point.x = it->x0;
    //                 point.y = it->y0;
    //                 point.z = 0;
    //                 mk.points.push_back(point);
    //                 point.x = it->vx + it->x0;
    //                 point.y = it->vy + it->y0;
    //                 point.z = 0;
    //                 mk.points.push_back(point);
    //             }

    //             markers.markers.push_back(mk);
    //         }
    //     }

    //     this->_map_pub->publish(markers);
    // }

private:
    void allocate(int width, int height) {
        size_x = width;
        size_y = height;
        cells.resize(width * height);
    }

    cell_t& getCell(int i, int j) {
        if (!isValid(i, j)) {
            throw std::out_of_range("Invalid cell coordinates");
        }
        return cells[i + j * size_x];
    }

    bool isValid(int i, int j) const {
        return (i >= 0 && i < size_x && j >= 0 && j < size_y);
    }

    int index(int i, int j) const {
        return i + j * size_x;
    }

    int worldToMapX(double x) const {
        return static_cast<int>(std::floor((x - origin_x) / resolution + 0.5) + size_x / 2);
    }

    int worldToMapY(double y) const {
        return static_cast<int>(std::floor((y - origin_y) / resolution + 0.5) + size_y / 2);
    }
};

}