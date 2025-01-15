#pragma once

#include <stdint.h>
#include <stddef.h>
#include <lc_map/lc_map.hpp>
#include <blackbox/blackbox.hpp>

#include <lcmcl_msgs/msg/localization.hpp>
#include <lcmcl_msgs/msg/odometry.hpp>

using namespace lc;

namespace sim
{

struct rp_t
{
    float roll;
    float pitch;
};

inline float rad2deg(float rad)
{
    return rad * 180 / M_PI;
}

inline float deg2rad(float deg)
{
    return deg * M_PI / 180;
}

inline pos_t pos_transformer(pos_t pos, pos_t tf)
{
    pos.x += (tf.x * cosf(pos.rad) + tf.y * -sinf(pos.rad));
    pos.y += (tf.x * sinf(pos.rad) + tf.y * cosf(pos.rad));
    pos.rad += tf.rad;
    return pos;
}

}