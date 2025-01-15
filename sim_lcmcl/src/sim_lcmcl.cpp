#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sim_lcmcl/lcmcl_simulator.hpp>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::sleep_for(std::chrono::milliseconds(500));

    rclcpp::executors::MultiThreadedExecutor executor;

    std::shared_ptr<LcmclSimulator> lcmcl_sim = std::make_shared<LcmclSimulator>();
    executor.add_node(lcmcl_sim);
    
    executor.spin();
    rclcpp::shutdown();
    return 0;
}