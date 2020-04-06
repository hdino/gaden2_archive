#ifndef GADEN_SIMULATOR_ENVIRONMENT_VISUALISATION_HPP_INCLUDED
#define GADEN_SIMULATOR_ENVIRONMENT_VISUALISATION_HPP_INCLUDED

#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rl_logging/logging_interface.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace gaden {

struct SimulatorConfig;

class EnvironmentVisualiser
{
public:
    EnvironmentVisualiser(const SimulatorConfig &config,
                          const std::string &node_name = "GadenEnvironment");
    ~EnvironmentVisualiser();

private:
    void performRosTasks();

    std::shared_ptr<rclcpp::Node> ros_node_;
    std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> environment_publisher_;
    std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> bounding_box_publisher_;
    std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> gas_sources_publisher_;

    visualization_msgs::msg::MarkerArray marker_environment_;
    visualization_msgs::msg::MarkerArray marker_bounding_box_;
    visualization_msgs::msg::MarkerArray marker_gas_sources_;

    rl::Logger logger_;

    bool run_ros_thread_;
    std::thread ros_thread_;
};

} // namespace gaden

#endif // GADEN_SIMULATOR_ENVIRONMENT_VISUALISATION_HPP_INCLUDED
