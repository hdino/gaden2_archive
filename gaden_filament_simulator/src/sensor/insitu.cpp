#include <gaden_common/ros_type_helper.h>
#include <gaden_filament_simulator/gas_dispersion_model.hpp>
#include <gaden_filament_simulator/sensor/insitu.hpp>

namespace gaden::sensor {

InSitu::InSitu(std::shared_ptr<GasDispersionModel> gas_model,
               std::shared_ptr<rclcpp::Node> &ros_node,
               Eigen::Vector3d position,
               const std::string &rviz_frame,
               rl::Logger logger_object)
    : SensorBase("insitu_sensor", gas_model, ros_node, rviz_frame, logger_object)
    , publisher_measurement_(
          ros_node->create_publisher<std_msgs::msg::Float64>(
              instance_name, 10))
{
    visualisation_marker.type = visualization_msgs::msg::Marker::SPHERE;
    visualisation_marker.scale = ros_type::getVector3(1);

    updatePosition(position);
}

InSitu::~InSitu()
{}

void InSitu::updatePosition(Eigen::Vector3d position)
{
    position_ = position;
    visualisation_marker.pose.position = ros_type::getPointFrom(position_);
    publishVisualisation();
}

void InSitu::doEvents()
{
    std_msgs::msg::Float64 msg;
    msg.data = gas_model->getConcentrationAt(position_);
    publisher_measurement_->publish(msg);

    publishVisualisationOnTimeout();
}

} // namespace gaden::sensor
