#include <gaden_common/ros_type_helper.h>
#include <gaden_filament_simulator/gas_dispersion_model.hpp>
#include <gaden_filament_simulator/sensor/insitu.hpp>

namespace gaden::sensor {

size_t InSitu::instance_counter_ = 0;
std::weak_ptr<rclcpp::Publisher<visualization_msgs::msg::Marker>> InSitu::static_publisher_visualisation_;

InSitu::InSitu(std::shared_ptr<GasDispersionModel> gas_model,
               std::shared_ptr<rclcpp::Node> &ros_node,
               Eigen::Vector3d position,
               const std::string &rviz_frame)
    : instance_number_(instance_counter_++)
    , gas_model_(gas_model)
    , publisher_measurement_(
          ros_node->create_publisher<std_msgs::msg::Float64>(
              "insitu_sensor" + std::to_string(instance_number_),
              10))
{
    publisher_visualisation_ = static_publisher_visualisation_.lock();
    if (!publisher_visualisation_)
    {
        publisher_visualisation_ =
                ros_node->create_publisher<visualization_msgs::msg::Marker>(
                    "insitu_sensor_visualisation", 1);
        static_publisher_visualisation_ = publisher_visualisation_;
    }

    updatePosition(position);

    visualisation_marker_.header.stamp = ros_node->get_clock()->now(); // TODO Update on publish?
    visualisation_marker_.header.frame_id = rviz_frame;
    visualisation_marker_.ns = "insitu_sensor";
    visualisation_marker_.id = instance_number_;
    visualisation_marker_.type = visualization_msgs::msg::Marker::SPHERE;
    visualisation_marker_.action = visualization_msgs::msg::Marker::ADD;
    visualisation_marker_.pose.orientation = ros_type::DefaultOrientation::get();
    visualisation_marker_.scale = ros_type::getVector3(1);
    visualisation_marker_.color = ros_type::getColor(0, 1, 0);

    publishVisualisation();
}

InSitu::~InSitu()
{}

void InSitu::updatePosition(Eigen::Vector3d position)
{
    position_ = position;
    visualisation_marker_.pose.position = ros_type::getPointFrom(position_);
    publishVisualisation();
}

void InSitu::doEvents()
{
    std_msgs::msg::Float64 msg;
    msg.data = gas_model_->getConcentrationAt(position_);
    publisher_measurement_->publish(msg);

    if (t_next_visualisation_publish_ >= std::chrono::steady_clock::now())
        publishVisualisation();
}

void InSitu::publishVisualisation()
{
    publisher_visualisation_->publish(visualisation_marker_);
    using namespace std::chrono_literals;
    t_next_visualisation_publish_ = std::chrono::steady_clock::now() + 5s;
}

} // namespace gaden::sensor
