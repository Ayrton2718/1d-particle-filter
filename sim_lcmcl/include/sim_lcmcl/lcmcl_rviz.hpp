#pragma once

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <cmath>
#include <float.h>
#include <random>

#include <blackbox/blackbox.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <lcmcl_msgs/msg/localization.hpp>
#include <lcmcl_msgs/msg/laser.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <std_msgs/msg/string.hpp>
#include <tut_tool/tut_tool.hpp>
#include <lc_map/lc_tf.hpp>
#include <lc_map/lc_map.hpp>
#include <lcmcl_topics/lcmcl_laser.hpp>
#include <lcmcl_topics/lcmcl_topics.hpp>


using namespace rclcpp;
using namespace std::chrono_literals;

class LcmclRviz : public rclcpp::Node
{
private:
    lcmcl_topics::LaserMsg  _laser_msg;
    rclcpp::Subscription<lcmcl_msgs::msg::Laser>::SharedPtr                                             _laser_sub;
    std::array<rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr, lcmcl_topics::LaserMsg::size>    _laser_pub;
    std::array<rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr, lcmcl_topics::LaserMsg::size100>  _laser100_pub;

    lcmcl::Localization _lcmcl_topics;

    std::shared_ptr<lc::Map>       _map;
    std::shared_ptr<lc::Tf> _tf;

    void laser_sub(lcmcl_msgs::msg::Laser::SharedPtr msg)
    {
        _laser_msg.input_msg(msg.get());

        sensor_msgs::msg::Range send_msg;
        send_msg.header.stamp = msg->header.stamp;
        send_msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
        send_msg.max_range = 30.1;
        send_msg.min_range = 0;
        send_msg.field_of_view = 0.5 * M_PI / 180;
        for(size_t i = 0; i < _laser_msg.size; i++)
        {
            send_msg.header.frame_id = _laser_msg.get_tf_array()[i];
            send_msg.range = _laser_msg._laser[i];
            _laser_pub[i]->publish(send_msg);
        }
        for(size_t i = 0; i < _laser_msg.size100; i++)
        {
            send_msg.header.frame_id = _laser_msg.get_tf_array100()[i];
            send_msg.range = _laser_msg._laser100[i];
            _laser100_pub[i]->publish(send_msg);
        }
    }

public:
    LcmclRviz(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : LcmclRviz("", options){}
    LcmclRviz(const std::string &name_space, const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) 
        : rclcpp::Node("sim_rviz", name_space, options)
    {
        this->_tf = std::make_shared<lc::Tf>(this);
        
        this->_map = std::make_shared<lc::Map>();
        _map->Map_cons(this);
        _map->enable_publish_map();

        _laser_msg.init(_tf);
        for(size_t i = 0; i < _laser_msg.size; i++)
        {
            _laser_pub[i] = this->create_publisher<sensor_msgs::msg::Range>(_laser_msg.get_tf_array()[i], rclcpp::SensorDataQoS());
        }
        for(size_t i = 0; i < _laser_msg.size100; i++)
        {
            _laser100_pub[i] = this->create_publisher<sensor_msgs::msg::Range>(_laser_msg.get_tf_array100()[i], rclcpp::SensorDataQoS());
        }
        _laser_sub = this->create_subscription<lcmcl_msgs::msg::Laser>("sensor_laser", rclcpp::QoS(1).reliable(), std::bind(&LcmclRviz::laser_sub, this, std::placeholders::_1));

        // _lcmcl_topics.init(this, [this, tf=_tf->get_geometry_msgs(_laser_msg.get_tf_array()[1].c_str())](rclcpp::Time stamp){            
        //     tf2::Quaternion tf2_quat;
        //     tf2::fromMsg(tf.transform.rotation, tf2_quat);
        //     tf2::Quaternion q = _lcmcl_topics.sensor_rotation(tf2_quat);
                        
        //     // tf2::Quaternion から tf2::Matrix3x3 を生成
        //     tf2::Matrix3x3 matrix(q);

        //     // ヨー、ピッチ、ロールを抽出
        //     double roll, pitch, yaw;
        //     matrix.getRPY(roll, pitch, yaw);

        //     RCLCPP_INFO(this->get_logger(), "%lf, %lf, %lf", roll, pitch, yaw);
        // });
    }
};