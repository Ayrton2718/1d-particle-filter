#pragma once

#include <vector>
#include <random>
#include <float.h>
#include "mc_type.hpp"

namespace mcl
{

class SensBase
{
public:
    SensBase(void){
    }

    virtual std::array<double, MCLTYPE_N> calc_weight(std::array<lc::pos_t, MCLTYPE_N> pf_pos) = 0;
};

}