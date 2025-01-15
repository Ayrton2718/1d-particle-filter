#pragma once

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <random>

#include <lcmcl_msgs/msg/odometry.hpp>

#include "mc_type.hpp"
#include <blackbox/blackbox.hpp>

namespace mcl
{

class OdomSubscriber
{
public:
    class Delta
    {
    public:
        Delta(OdomSubscriber* os)
        {
            _os = os;
            _befor_tim = _os->now();
        }

        pos_t velocity(void){
            return _os->vel();
        }

        pos_t delta(rclcpp::Time stamp_tim, bool is_reset){
            if(stamp_tim.get_clock_type() == _befor_tim.get_clock_type())
            {
                rclcpp::Duration dur = stamp_tim - _befor_tim;
                
                if(is_reset){_befor_tim = stamp_tim;}

                pos_t vel = _os->vel();
                pos_t delta;
                delta.x = vel.x * dur.seconds();
                delta.y = vel.y * dur.seconds();
                delta.rad = vel.rad * dur.seconds();
                return delta;
            }
            else
            {
                rclcpp::Time now = _os->now();
                rclcpp::Duration dur = now - _befor_tim;
                
                if(is_reset){_befor_tim = now;}

                pos_t vel = _os->vel();
                pos_t delta;
                delta.x = vel.x * dur.seconds();
                delta.y = vel.y * dur.seconds();
                delta.rad = vel.rad * dur.seconds();
                return delta;
            }
        }


    private:
        OdomSubscriber* _os;
        rclcpp::Time _befor_tim;
    };

private:
    rclcpp::Time now(void)
    {
        return _node->get_clock()->now();
    }

    pos_t vel(void){
        return _vel;
    }

public:
    OdomSubscriber(blackbox::BlackBoxNode* node, std::function<void(rclcpp::Time)> cb){
        _node = node;

        _vel = {0, 0, 0};

        _sub.init(node, "sensor_odometry", rclcpp::QoS(1).reliable(), [this, cb](lcmcl_msgs::msg::Odometry::SharedPtr msg){
                _vel = {
                    msg->rel_vx,
                    msg->rel_vy,
                    msg->rel_vrad
                };

                cb(msg->header.stamp);
            });
    }

private:
    blackbox::BlackBoxNode* _node;
    
    pos_t _vel = {0, 0, 0};
    rclcpp::Time _befor_tim;

    blackbox::SubRecord<lcmcl_msgs::msg::Odometry, true> _sub;
};

using Delta=OdomSubscriber::Delta;

}