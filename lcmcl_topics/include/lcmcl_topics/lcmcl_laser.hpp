#pragma once

#include <rclcpp/rclcpp.hpp>
#include <lcmcl_msgs/msg/laser.hpp>
#include <lc_map/lc_type.hpp>
#include <lc_map/lc_tf.hpp>

namespace lcmcl_topics
{

class LaserMsg
{
public:
    enum class laser_name_t{
        laser_r = 0,
        laser_l = 1,
        laser_br = 2,
        laser_bl = 3,
    };

    enum class laser100_name_t{
        laser_100fr = 0,
        laser_100fl = 1,
        laser_100r = 2,
        laser_100l = 3,
    };

public:
    static const size_t size = 4;

    std::array<float, size> _laser;
    std::array<lc::pos_t, size>    _pos;

    static const size_t size100 = 4;
    std::array<float, size100> _laser100;
    std::array<lc::pos_t, size100>    _pos100;

public:
    LaserMsg(void)
    {
    }

    void init(std::shared_ptr<lc::Tf> tf)
    {
        std::array<std::string, size> tf_list = get_tf_array();
        for(size_t i = 0; i < tf_list.size(); i++)
        {
            _pos[i] = tf->get_tf(tf_list[i].c_str()).pos;
        }

        std::array<std::string, size100> tf_list100 = get_tf_array100();
        for(size_t i = 0; i < tf_list100.size(); i++)
        {
            _pos100[i] = tf->get_tf(tf_list100[i].c_str()).pos;
        }
    }

    void input_msg(const lcmcl_msgs::msg::Laser* msg)
    {
        _laser[0] = msg->laser_r;
        _laser[1] = msg->laser_l;
        _laser[2] = msg->laser_br;
        _laser[3] = msg->laser_bl;
        _laser100[0] = msg->laser_100fr;
        _laser100[1] = msg->laser_100fl;
        _laser100[2] = msg->laser_100r;
        _laser100[3] = msg->laser_100l;
    }

    std::array<std::string, size> get_tf_array(void)
    {
        return {
                "laser_r",
                "laser_l",
                "laser_br",
                "laser_bl"
            };
    }

    std::array<std::string, size100> get_tf_array100(void)
    {
        return {
                "laser_100fr",
                "laser_100fl",
                "laser_100r",
                "laser_100l"
            };
    }

    lcmcl_msgs::msg::Laser get_msg(void)
    {
        lcmcl_msgs::msg::Laser msg;
        msg.laser_r = _laser[0];
        msg.laser_l = _laser[1];
        msg.laser_br = _laser[2];
        msg.laser_bl =_laser[3];
        msg.laser_100fr = _laser100[0];
        msg.laser_100fl = _laser100[1];
        msg.laser_100r = _laser100[2];
        msg.laser_100l = _laser100[3];
        return msg;

    }
};

}