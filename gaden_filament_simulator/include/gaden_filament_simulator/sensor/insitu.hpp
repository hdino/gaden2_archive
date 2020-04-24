#ifndef GADEN_SIMULATOR_SENSOR_INSITU_HPP_INCLUDED
#define GADEN_SIMULATOR_SENSOR_INSITU_HPP_INCLUDED

#include <chrono>
#include <memory>
#include <string>

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace gaden {

class GasDispersionModel;

} // namespace gaden

namespace gaden::sensor {

class InSitu
{
public:
    InSitu(std::shared_ptr<GasDispersionModel> gas_model,
           std::shared_ptr<rclcpp::Node> &ros_node,
           Eigen::Vector3d position,
           const std::string &rviz_frame);
    ~InSitu();

    void doEvents();
    void updatePosition(Eigen::Vector3d position);

private:
    void publishVisualisation();

    static size_t instance_counter_;
    static std::weak_ptr<rclcpp::Publisher<visualization_msgs::msg::Marker>> static_publisher_visualisation_;

    size_t instance_number_;
    std::shared_ptr<GasDispersionModel> gas_model_;
    Eigen::Vector3d position_;

    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64>> publisher_measurement_;
    std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::Marker>> publisher_visualisation_;

    visualization_msgs::msg::Marker visualisation_marker_;
    std::chrono::steady_clock::time_point t_next_visualisation_publish_;
};

} // namespace gaden::sensor

#endif // GADEN_SIMULATOR_SENSOR_INSITU_HPP_INCLUDED
