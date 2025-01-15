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
    
    blackbox::PubRecord<lcmcl_msgs::msg::Localization>  _est_pub;
    rclcpp::TimerBase::SharedPtr                            _est_pub_tim;

public:
    Lcmcl(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : Lcmcl("", options){}
    Lcmcl(const std::string &name_space, const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) 
        : blackbox::BlackBoxNode(blackbox::debug_mode_t::RELEASE, "lcmcl", name_space, options)
    {
        this->_tf = std::make_shared<lc::Tf>(this, std::chrono::milliseconds(5000));
        _tf->enable_send_tf();
        this->_map = std::make_shared<lc::Map>(this, _tf);
        _map->enable_publish_map();

        _est_pub.init(this, "est_localization", rclcpp::QoS(1).reliable());

        _lpkf = std::make_unique<mcl::LPKf>(this, _tf, _map, &_est_pub);
    }
};

}