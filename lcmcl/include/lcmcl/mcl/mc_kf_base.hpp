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

    struct odom_t{
        rclcpp::Time tim;
        mat_pos_t pos;
        mat_cov_t cov;
    };

    virtual std::array<double, MCLTYPE_N> calc_weight(rclcpp::Time pf_tim, std::array<lc::pos_t, MCLTYPE_N> pf_pos) = 0;

    virtual KfBase::odom_t predict(KfBase::odom_t pf_obs) = 0;
};

}