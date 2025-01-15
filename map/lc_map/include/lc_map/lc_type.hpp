#pragma once

#include <blackbox/blackbox.hpp>

#define LC_DEFAULT_INITIAL_X   (0)
#define LC_DEFAULT_INITIAL_Y   (0)
#define LC_DEFAULT_INITIAL_DEG (0)

namespace lc
{

struct pos_t
{
    double x;
    double y;
    double rad;
};

struct trans_t{
    pos_t pos;
    double z;
    double pitch;
    double roll;
};

inline float rad2deg(float rad)
{
    return rad * 180 / M_PI;
}

inline float deg2rad(float deg)
{
    return deg * M_PI / 180;
}

}