#pragma once

#include <vector>
#include <random>
#include <float.h>
#include "mc_type.hpp"

namespace mcl
{

class SensBase
{
private:
    std::function<void(rclcpp::Time sens_tim)> _sens_cb;

protected:
    void sensor_callback(rclcpp::Time sens_tim){
        _sens_cb(sens_tim);
    }

public:
    SensBase(std::function<void(rclcpp::Time sens_tim)> sens_cb){
        _sens_cb = sens_cb;
    }

    virtual std::array<double, MCLTYPE_N> calc_weight(std::array<lc::pos_t, MCLTYPE_N> pf_pos) = 0;
};

}