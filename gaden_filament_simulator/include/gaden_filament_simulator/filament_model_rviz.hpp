#ifndef GADEN_SIMULATOR_FILAMENT_MODEL_RVIZ_HPP_INCLUDED
#define GADEN_SIMULATOR_FILAMENT_MODEL_RVIZ_HPP_INCLUDED

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rl_logging/logging_interface.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace gaden {

class FilamentModel;

class FilamentModelRvizVisualisation
{
public:
    FilamentModelRvizVisualisation(std::shared_ptr<rclcpp::Node> ros_node,
                                   std::shared_ptr<FilamentModel> filament_model,
                                   const std::string &rviz_frame_id,
                                   double scale,
                                   rl::Logger &logger);

    ~FilamentModelRvizVisualisation();

    void publish();

private:
    rl::Logger logger_;

    std::shared_ptr<rclcpp::Node> ros_node_;
    std::shared_ptr<FilamentModel> filament_model_;

    std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::Marker>> publisher_;

    visualization_msgs::msg::Marker marker_;
};

} // namespace gaden

#endif // GADEN_SIMULATOR_FILAMENT_MODEL_RVIZ_HPP_INCLUDED
