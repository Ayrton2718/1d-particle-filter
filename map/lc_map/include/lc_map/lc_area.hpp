#pragma once

#include <stdio.h>
#include <math.h>
#include <vector>
#include <array>
#include "lc_type.hpp"

namespace lc
{

enum class area_t{
    area1,
    area2,
    area3,
    slope1,
    slope2,
    slope3,
    storage,
    silo,
    water,
    // TODO
    start_r1,
    start_r2,
    retry_r2,
    carpet,
    __count
};

enum class height_type_t{
    height_0 = 0,
    height_100 = 1,
    height_200 = 2,
    height_slope1 = 3,
    height_slope2 = 4,
    height_slope3 = 5,
    height_unknown = 100
};

class Area
{
private:
    struct box_t
    {
        float left_up_x;
        float left_up_y;
        float right_bt_x;
        float right_bt_y;
    };

    struct slope_t
    {
        box_t slope;
        pos_t slope_vec; // {from_center_x, from_center_y, direction}
    };

    std::array<box_t, static_cast<size_t>(area_t::__count)> _area;

    std::array<slope_t, 3>  _slope;

public:
    Area(void)
    {
        size_t index;

        _area[area_index(area_t::area1)] = {    4.05,   0.125,     0.05,    6.00};
        _area[area_index(area_t::area2)] = {    8.05,   0.125,     4.05,    6.00};
        _area[area_index(area_t::area3)] = {    12.05,  0.125,     8.05,    6.00};
        _area[area_index(area_t::slope1)] = {   4.05,   5.0,       3.05,    6.00};
        _area[area_index(area_t::slope2)] = {   8.05,   1.2,       7.05,    2.2};
        _area[area_index(area_t::slope3)] = {   12.05,  2.75,      8.05,    3.75};
        _area[area_index(area_t::storage)] = {  12.05,  3.75,      8.05,    6.00};
        _area[area_index(area_t::silo)] = {     12.05,  0.125,     8.05,    2.75};
        _area[area_index(area_t::water)] = {    8.05,   2.2,       7.05,    6.00};

        _slope[0].slope = _area[area_index(area_t::slope1)];
        _slope[0].slope_vec = {3.05, 5.5, deg2rad(0)};

        _slope[1].slope = _area[area_index(area_t::slope2)];
        _slope[1].slope_vec = {7.05, 1.7, deg2rad(0)};

        _slope[2].slope = _area[area_index(area_t::slope3)];
        _slope[2].slope_vec = {10.05, 3.75, deg2rad(-90)};
    }

    bool is_area(const area_t area, pos_t pos)
    {
        pos.y = fabs(pos.y);

        box_t box = _area[static_cast<size_t>(area)];

        if((box.right_bt_x <= pos.x && pos.x <= box.left_up_x)
            && (box.left_up_y <= pos.y && pos.y <= box.right_bt_y))
        {
            return true;
        }

        return false;
    }

    float height_real(pos_t pos)
    {
        pos.y = fabs(pos.y);

        height_type_t type = height_type(pos);
        switch(type)
        {
        case height_type_t::height_0:
            return 0.0;

        case height_type_t::height_100:
            return 0.1;
            
        case height_type_t::height_200:
            return 0.2;

        case height_type_t::height_slope1:{
            slope_t* slope = &_slope[0];
            if((slope->slope.right_bt_x <= pos.x && pos.x <= slope->slope.left_up_x)
                && (slope->slope.left_up_y <= pos.y && pos.y <= slope->slope.right_bt_y))
            {
                float dx = fabs(pos.x - slope->slope_vec.x);
                float dy = fabs(pos.y - slope->slope_vec.y);
                float distance = dx * cos(slope->slope_vec.rad) - dy * sin(slope->slope_vec.rad);
                return distance / 10 + 0.0;
            }
            }return 0.0;

        case height_type_t::height_slope2:{
            slope_t* slope = &_slope[1];
            if((slope->slope.right_bt_x <= pos.x && pos.x <= slope->slope.left_up_x)
                && (slope->slope.left_up_y <= pos.y && pos.y <= slope->slope.right_bt_y))
            {
                float dx = fabs(pos.x - slope->slope_vec.x);
                float dy = fabs(pos.y - slope->slope_vec.y);
                float distance = dx * cos(slope->slope_vec.rad) - dy * sin(slope->slope_vec.rad);
                return distance / 10 + 0.1;
            }
            }return 0.1;
        
        case height_type_t::height_slope3:{
            slope_t* slope = &_slope[2];
            if((slope->slope.right_bt_x <= pos.x && pos.x <= slope->slope.left_up_x)
                && (slope->slope.left_up_y <= pos.y && pos.y <= slope->slope.right_bt_y))
            {
                float dx = fabs(pos.x - slope->slope_vec.x);
                float dy = fabs(pos.y - slope->slope_vec.y);
                float distance = dx * cos(slope->slope_vec.rad) - dy * sin(slope->slope_vec.rad);
                return distance / 10 + 0.1;
            }
            }return 0.1;
            
        case height_type_t::height_unknown:
            return 0;
        }
        return 0;
    }

    height_type_t height_type(pos_t pos)
    {
        if(is_area(area_t::slope1, pos)){
            return height_type_t::height_slope1;
        }

        if(is_area(area_t::slope2, pos)){
            return height_type_t::height_slope2;
        }

        if(is_area(area_t::slope3, pos)){
            return height_type_t::height_slope3;
        }

        if(is_area(area_t::area1, pos)){
            return height_type_t::height_0;
        }

        if(is_area(area_t::area2, pos) || is_area(area_t::storage, pos)){
            return height_type_t::height_100;
        }


        if(is_area(area_t::silo, pos)){
            return height_type_t::height_200;
        }

        return height_type_t::height_unknown;
    }

    size_t area_index(area_t area)
    {
        return static_cast<size_t>(area);
    }

    size_t height_index(height_type_t height)
    {
        return static_cast<size_t>(height);
    }
};

} // namespace lc
