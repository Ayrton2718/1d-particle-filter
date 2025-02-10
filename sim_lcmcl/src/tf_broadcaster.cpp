#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
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

        // パラメータからトピック名を取得
        std::string odom_topic;
        this->get_parameter("odom_topic", odom_topic);
        RCLCPP_INFO(this->get_logger(), "Subscribing to odometry topic: %s", odom_topic.c_str());

        // tf2 のブロードキャスターを生成
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        
        // 指定されたトピック名でサブスクライバーを作成
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 10,
            std::bind(&OdomTfBroadcaster::odomCallback, this, std::placeholders::_1));
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Odometry の内容から TransformStamped メッセージを作成
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = msg->header.stamp;
        transformStamped.header.frame_id = msg->header.frame_id; // 親フレーム（例："odom"）
        transformStamped.child_frame_id = msg->child_frame_id;   // 子フレーム（例："base_link"）

        // 位置情報の設定
        transformStamped.transform.translation.x = msg->pose.pose.position.x;
        transformStamped.transform.translation.y = msg->pose.pose.position.y;
        transformStamped.transform.translation.z = msg->pose.pose.position.z;

        // 姿勢情報の設定
        transformStamped.transform.rotation = msg->pose.pose.orientation;

        // TF を送出
        tf_broadcaster_->sendTransform(transformStamped);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
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
