#pragma once

#include <stdint.h>
#include <stddef.h>
#include <lc_map/lc_map.hpp>
#include <lc_map/lc_tf.hpp>
#include <blackbox/blackbox.hpp>

#include <lcmcl_msgs/msg/localization.hpp>
#include <lcmcl_msgs/msg/odometry.hpp>

#define MCLTYPE_N   (64)

using namespace lc;

namespace mcl
{

struct rp_t
{
    float roll;
    float pitch;
};

}