#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sim_lcmcl/lcmcl_tf_publisher.hpp>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::sleep_for(std::chrono::milliseconds(500));

    rclcpp::executors::MultiThreadedExecutor executor;

    std::shared_ptr<LcmclTfPublisher> lcmcl_tf_publisher = std::make_shared<LcmclTfPublisher>();
    executor.add_node(lcmcl_tf_publisher);
    
    executor.spin();
    rclcpp::shutdown();
    return 0;
}