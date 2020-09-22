#ifndef GADEN_SIMULATOR_SENSOR_SENSOR_BASE_HPP_INCLUDED
#define GADEN_SIMULATOR_SENSOR_SENSOR_BASE_HPP_INCLUDED

#include <chrono>
#include <map>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rl_logging/logging_interface.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace gaden {
class GasDispersionModel;
} // namespace gaden

namespace gaden::sensor {

class SensorBase
{
public:
    SensorBase(const std::string &sensor_type,
               std::shared_ptr<GasDispersionModel> gas_model,
               std::shared_ptr<rclcpp::Node> &ros_node,
               const std::string &rviz_frame,
               rl::Logger logger_object);
    virtual ~SensorBase();

protected:
    void publishVisualisationOnTimeout();
    virtual void publishVisualisation();

    rl::Logger logger;

    size_t instance_number;
    std::string instance_name;

    std::shared_ptr<GasDispersionModel> gas_model;

    visualization_msgs::msg::Marker visualisation_marker;

private:
    std::map<std::string, size_t> instance_counter_; // maps sensor type --> number of instances

    std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::Marker>> visualisation_publisher_;
    std::chrono::steady_clock::time_point t_next_visualisation_publish_;
};

} // namespace gaden::sensor

#endif // GADEN_SIMULATOR_SENSOR_SENSOR_BASE_HPP_INCLUDED
