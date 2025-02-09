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
    
public:
    Lcmcl(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : Lcmcl("", options){}
    Lcmcl(const std::string &name_space, const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) 
        : blackbox::BlackBoxNode(blackbox::debug_mode_t::RELEASE, "lcmcl", name_space, options, true)
    {
        this->_tf = std::make_shared<lc::Tf>(this);
        this->_map = std::make_shared<lc::Map>(this);

        _lpkf = std::make_unique<mcl::LPKf>(this, _tf, _map);
    }
};

}