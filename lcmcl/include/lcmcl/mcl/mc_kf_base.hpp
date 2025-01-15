#pragma once

#include <vector>
#include <random>
#include <float.h>
#include "mc_type.hpp"
#include <Eigen/Dense>

namespace mcl
{

class KfBase
{
public:
    KfBase(){}


    typedef Eigen::Matrix<float, 3, 1> mat_pos_t;
    typedef Eigen::Matrix<float, 3, 3> mat_cov_t;
    typedef std::tuple<rclcpp::Time, mat_pos_t, mat_cov_t> observation_t;

    virtual std::array<double, MCLTYPE_N> calc_weight(rclcpp::Time pf_tim, std::array<lc::pos_t, MCLTYPE_N> pf_pos) = 0;

    virtual void observation(observation_t pf_obs) = 0;

    virtual pos_t predict_pos(rclcpp::Time sens_tim) = 0;
};

}