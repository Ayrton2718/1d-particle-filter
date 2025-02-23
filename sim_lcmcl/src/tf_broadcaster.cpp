#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

class OdomTfBroadcaster : public rclcpp::Node
{
public:
    OdomTfBroadcaster()
        : Node("odom_tf_broadcaster")
    {
        // パラメータとしてトピック名を宣言（デフォルト値は "odom"）
        this->declare_parameter<std::string>("odom_topic", "odom");
        this->declare_parameter<std::string>("offset_topic", "offset_odom");
        this->declare_parameter<float>("x_offset", 0.0);
        this->declare_parameter<float>("y_offset", 0.0);
        this->declare_parameter<float>("yaw_offset", 0.0);

        // パラメータからトピック名を取得
        std::string odom_topic, offset_topic;
        this->get_parameter("odom_topic", odom_topic);
        this->get_parameter("offset_topic", offset_topic);
        RCLCPP_INFO(this->get_logger(), "Subscribing to odometry topic: %s", odom_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Publishing to offset odometry topic: %s", offset_topic.c_str());

        // オフセットを取得
        this->get_parameter("x_offset", x_offset_);
        this->get_parameter("y_offset", y_offset_);
        this->get_parameter("yaw_offset", yaw_offset_);
        RCLCPP_INFO(this->get_logger(), "Offset: x=%f, y=%f, yaw=%f", x_offset_, y_offset_, yaw_offset_);

        // tf2 のブロードキャスターを生成
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        
        // 指定されたトピック名でサブスクライバーを作成
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 10,
            std::bind(&OdomTfBroadcaster::odomCallback, this, std::placeholders::_1));
        
        // オフセットを加えた Odometry をパブリッシュするためのパブリッシャーを作成
        offset_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(offset_topic, 10);
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // オフセットを加えた Odometry を作成
        auto offset_msg = std::make_shared<nav_msgs::msg::Odometry>(*msg);
        double x = offset_msg->pose.pose.position.x + x_offset_;
        double y = offset_msg->pose.pose.position.y + y_offset_;

        offset_msg->pose.pose.position.x = x * cos(yaw_offset_) - y * sin(yaw_offset_);
        offset_msg->pose.pose.position.y = x * sin(yaw_offset_) + y * cos(yaw_offset_);

        // 姿勢情報の設定（yaw_offsetを加える）
        tf2::Quaternion q_orig, q_offset, q_new;
        tf2::fromMsg(msg->pose.pose.orientation, q_orig);
        q_offset.setRPY(0, 0, yaw_offset_);
        q_new = q_orig * q_offset;
        q_new.normalize();
        offset_msg->pose.pose.orientation = tf2::toMsg(q_new);

        offset_pub_->publish(*offset_msg);


        // Odometry の内容から TransformStamped メッセージを作成
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = msg->header.stamp;
        transformStamped.header.frame_id = msg->header.frame_id; // 親フレーム（例："odom"）
        transformStamped.child_frame_id = msg->child_frame_id;   // 子フレーム（例："base_link"）

        // 位置情報の設定（オフセットを加える）
        transformStamped.transform.translation.x = offset_msg->pose.pose.position.x;
        transformStamped.transform.translation.y = offset_msg->pose.pose.position.y;
        transformStamped.transform.translation.z = offset_msg->pose.pose.position.z;

        // 姿勢情報の設定（yaw_offsetを加える）
        transformStamped.transform.rotation = offset_msg->pose.pose.orientation;

        // TF を送出
        tf_broadcaster_->sendTransform(transformStamped);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr offset_pub_;

    float x_offset_, y_offset_, yaw_offset_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomTfBroadcaster>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
