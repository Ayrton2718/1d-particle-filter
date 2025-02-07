#pragma once

// line crossing determination
#include <rclcpp/rclcpp.hpp>
// #include <cmath>
// #include <float.h>
// #include <random>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/utils.h>

// #include <tut_tool/tut_tool.hpp>

#include "lc_type.hpp"

namespace lc
{

class Tf
{
private:
    // std::shared_ptr<blackbox::ParamParser>   _pp;

    pos_t       _initial_pos;

    std::shared_ptr<tf2_ros::TransformBroadcaster>    _base_link_tf;

    std::shared_ptr<tf2_ros::Buffer>                        _buffer_tf;
    std::shared_ptr<tf2_ros::TransformListener>             _listener_tf;

    std::pair<bool, geometry_msgs::msg::TransformStamped> get_transform(std::string target_frame, std::string source_frame)
    {
        if(this->_buffer_tf->canTransform(target_frame, source_frame, rclcpp::Time(0), rclcpp::Duration::from_seconds(10)))
        {
            geometry_msgs::msg::TransformStamped transformStamped = this->_buffer_tf->lookupTransform(target_frame, source_frame, rclcpp::Time(0), rclcpp::Duration::from_seconds(1));
            return std::make_pair(true, transformStamped);
        }else{
            geometry_msgs::msg::TransformStamped default_trans;
            return std::make_pair(false, default_trans);
        }
    }

public:
    Tf(blackbox::BlackBoxNode* node)
    {
        this->_base_link_tf = std::make_shared<tf2_ros::TransformBroadcaster>(*node);

        this->_buffer_tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
        this->_listener_tf = std::make_shared<tf2_ros::TransformListener>(*this->_buffer_tf);
    }

    Tf(rclcpp::Node* node)
    {        
        _initial_pos.x = 0;
        _initial_pos.y = 0;
        _initial_pos.rad = 0;
        this->_base_link_tf = std::make_shared<tf2_ros::TransformBroadcaster>(node);

        this->_buffer_tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
        this->_listener_tf = std::make_shared<tf2_ros::TransformListener>(*this->_buffer_tf);
    }

    void enable_send_tf()
    {
    }

    void set_est(const pos_t pos, rclcpp::Time stamp)
    {
        (void)(pos);
        (void)(stamp);
        
        if(_base_link_tf != nullptr)
        {
            // geometry_msgs::msg::TransformStamped t;
            // tf2::Quaternion q;
            // t.header.stamp = stamp;
            // t.header.frame_id = "map";
            // t.child_frame_id = "base_link";
            // t.transform.translation.x = pos.x;
            // t.transform.translation.y = pos.y;
            // t.transform.translation.z = 0.0;
            // q.setRPY(0, 0, pos.rad);
            // t.transform.rotation.x = q.x();
            // t.transform.rotation.y = q.y();
            // t.transform.rotation.z = q.z();
            // t.transform.rotation.w = q.w();
            // this->_base_link_tf->sendTransform(t);
        }
    }

    void set_true(const pos_t pos, rclcpp::Time stamp)
    {
        (void)(pos);
        (void)(stamp);

        if(_base_link_tf != nullptr)
        {
            geometry_msgs::msg::TransformStamped t;
            tf2::Quaternion q;
            t.header.stamp = stamp;
            t.header.frame_id = "map";
            t.child_frame_id = "base_link";
            t.transform.translation.x = pos.x;
            t.transform.translation.y = pos.y;
            t.transform.translation.z = 0.0;
            q.setRPY(0, 0, pos.rad);
            t.transform.rotation.x = q.x();
            t.transform.rotation.y = q.y();
            t.transform.rotation.z = q.z();
            t.transform.rotation.w = q.w();
            this->_base_link_tf->sendTransform(t);
        }
    }

    pos_t get_initial_pos(void)
    {
        return _initial_pos;
    }

    trans_t get_tf(std::string str, trans_t dflt={{0, 0, 0}, 0, 0,0})
    {
        auto result = this->get_transform("base_link", str);
        if(result.first){
            trans_t trans;
            trans.pos.x = result.second.transform.translation.x;
            trans.pos.y = result.second.transform.translation.y;
            trans.z = result.second.transform.translation.z;
            tf2::getEulerYPR<geometry_msgs::msg::Quaternion>(result.second.transform.rotation, trans.pos.rad, trans.pitch, trans.roll);
            return trans;
        }else{
            return dflt;
        }
    }

    geometry_msgs::msg::TransformStamped get_geometry_msgs(std::string str){
        return this->get_transform("base_link", str).second;
    }
};


inline pos_t pos_transformer(pos_t pos, pos_t tf)
{
    pos.x += (tf.x * cosf(pos.rad) + tf.y * -sinf(pos.rad));
    pos.y += (tf.x * sinf(pos.rad) + tf.y * cosf(pos.rad));
    pos.rad += tf.rad;
    return pos;
}


}