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

    double _origin_x, _origin_y, _origin_theta; // Map origin
    double _scale; // Map _resolution (meters per cell)
    int _size_x, _size_y; // Map dimensions (number of cells)
    std::vector<cell_t> _cells; // Grid data
    double _max_occ_dist; // Maximum distance for obstacle consideration
    double _occupied_thresh; // Threshold for occupied _cells
    double _free_thresh; // Threshold for free cells

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

        this->_origin_x = 0;
        this->_origin_y = 0;
        this->_origin_theta = 0;
        this->_scale = 0;
        this->_size_x = 0;
        this->_size_y = 0;
        this->_max_occ_dist = 0;
        this->_occupied_thresh = 0.65;
        this->_free_thresh = 0.196;
        
        this->import_yaml();
        this->import_pgm();

        this->_map_pub = nullptr;
        this->m_cb_grp1 = nullptr;
        this->_map_pub_tim = NULL;
    }

    void Map_cons(rclcpp::Node* node)
    {
        this->_node = node;

        this->_origin_x = 0;
        this->_origin_y = 0;
        this->_origin_theta = 0;
        this->_scale = 0;
        this->_size_x = 0;
        this->_size_y = 0;
        this->_max_occ_dist = 0;
        this->_occupied_thresh = 0.65;
        this->_free_thresh = 0.196;

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

// Convert from map index to world coords
#define MAP_WXGX(ptr, i) (ptr->_origin_x + ((i) - ptr->_size_x / 2) * ptr->_scale)
#define MAP_WYGY(ptr, j) (ptr->_origin_y + ((j) - ptr->_size_y / 2) * ptr->_scale)

// Convert from world coords to map coords
#define MAP_GXWX(ptr, x) (floor((x - ptr->_origin_x) / ptr->_scale + 0.5) + ptr->_size_x / 2)
#define MAP_GYWY(ptr, y) (floor((y - ptr->_origin_y) / ptr->_scale + 0.5) + ptr->_size_y / 2)
// Test to see if the given map coords lie within the absolute map bounds.
#define MAP_VALID(ptr, i, j) ((i >= 0) && (i < ptr->_size_x) && (j >= 0) && (j < ptr->_size_y))

// Compute the cell index for the given map coords.
#define MAP_INDEX(ptr, i, j) ((i) + (j) * ptr->_size_x)

    laser_t calculate_range(pos_t pos, float max_range)
    {
        // Bresenham raytracing
        int x0,x1,y0,y1;
        int x,y;
        int xstep, ystep;
        char steep;
        int tmp;
        int deltax, deltay, error, deltaerr;

        x0 = MAP_GXWX(this,pos.x);
        y0 = MAP_GYWY(this,pos.y);
        
        x1 = MAP_GXWX(this,pos.x + max_range * cos(pos.rad));
        y1 = MAP_GYWY(this,pos.y + max_range * sin(pos.rad));

        if(abs(y1-y0) > abs(x1-x0))
            steep = 1;
        else
            steep = 0;

        if(steep)
        {
            tmp = x0;
            x0 = y0;
            y0 = tmp;

            tmp = x1;
            x1 = y1;
            y1 = tmp;
        }

        deltax = abs(x1-x0);
        deltay = abs(y1-y0);
        error = 0;
        deltaerr = deltay;

        x = x0;
        y = y0;

        if(x0 < x1)
            xstep = 1;
        else
            xstep = -1;
        if(y0 < y1)
            ystep = 1;
        else
            ystep = -1;

        if(steep)
        {
            if(!MAP_VALID(this,y,x) || this->_cells[MAP_INDEX(this,y,x)].occ_state > -1){
                laser_t hit;
                hit.hit_type = 1;
                hit.range = sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * this->_scale;
                return hit;
            }
        }
        else
        {
            if(!MAP_VALID(this,x,y) || this->_cells[MAP_INDEX(this,x,y)].occ_state > -1){
                laser_t hit;
                hit.hit_type = 1;
                hit.range = sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * this->_scale;
                return hit;
            }
        }

        while(x != (x1 + xstep * 1))
        {
            x += xstep;
            error += deltaerr;
            if(2*error >= deltax)
            {
            y += ystep;
            error -= deltax;
            }

            if(steep)
            {
                if(!MAP_VALID(this,y,x) || this->_cells[MAP_INDEX(this,y,x)].occ_state > -1){
                    laser_t hit;
                    hit.hit_type = 1;
                    hit.range = sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * this->_scale;
                    return hit;
                }
            }
            else
            {
                if(!MAP_VALID(this,x,y) || this->_cells[MAP_INDEX(this,x,y)].occ_state > -1){
                    laser_t hit;
                    hit.hit_type = 1;
                    hit.range = sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * this->_scale;
                    return hit;
                }
            }
        }

        laser_t hit;
        hit.hit_type = -1;
        hit.range = max_range;
        return hit;
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
        _size_x = width;
        _size_y = height;
        _cells.resize(width * height);
    }

    cell_t& getCell(int i, int j) {
        if (!isValid(i, j)) {
            throw std::out_of_range("Invalid cell coordinates");
        }
        return _cells[i + j * _size_x];
    }

    bool isValid(int i, int j) const {
        return (i >= 0 && i < _size_x && j >= 0 && j < _size_y);
    }

    int index(int i, int j) const {
        return i + j * _size_x;
    }

    int worldToMapX(double x) const {
        return static_cast<int>(std::floor((x - _origin_x) / _scale + 0.5) + _size_x / 2);
    }

    int worldToMapY(double y) const {
        return static_cast<int>(std::floor((y - _origin_y) / _scale + 0.5) + _size_y / 2);
    }
};

}