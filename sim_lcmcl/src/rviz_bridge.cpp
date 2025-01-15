#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sim_lcmcl/lcmcl_rviz.hpp>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::sleep_for(std::chrono::milliseconds(500));

    rclcpp::executors::SingleThreadedExecutor executor;

    std::shared_ptr<LcmclRviz> lcmcl_rviz = std::make_shared<LcmclRviz>();
    executor.add_node(lcmcl_rviz);
    
    executor.spin();
    rclcpp::shutdown();
    return 0;
}