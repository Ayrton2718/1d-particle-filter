#pragma once

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/callback_group.hpp>
#include <blackbox/blackbox.hpp>
#include "mcl/mcl.hpp"

using namespace rclcpp;
using namespace std::chrono_literals;

namespace mcl
{

class Lcmcl : public blackbox::BlackBoxNode
{
private:
    std::shared_ptr<lc::Tf>     _tf;
    std::shared_ptr<lc::Map>    _map;

    std::unique_ptr<mcl::LPKf>  _lpkf;

    blackbox::Param<float> _initial_pos_x;
    blackbox::Param<float> _initial_pos_y;
    blackbox::Param<float> _initial_pos_yaw;

public:
    Lcmcl(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : Lcmcl("", options){}
    Lcmcl(const std::string &name_space, const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) 
        : blackbox::BlackBoxNode(blackbox::debug_mode_t::RELEASE, "lcmcl", name_space, options, true)
    {
        this->_tf = std::make_shared<lc::Tf>(this);
        this->_map = std::make_shared<lc::Map>(this);

        _initial_pos_x.init(this, "initial_pos.x", 0.0);
        _initial_pos_y.init(this, "initial_pos.y", 0.0);
        _initial_pos_yaw.init(this, "initial_pos.yaw", 0.0);

        pos_t initial_pos;
        initial_pos.x = _initial_pos_x.get();
        initial_pos.y = _initial_pos_y.get();
        initial_pos.rad = _initial_pos_yaw.get() * M_PI / 180;

        _lpkf = std::make_unique<mcl::LPKf>(this, _tf, _map, initial_pos);
    }
};

}