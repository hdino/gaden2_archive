#include <gaden_common/ros_type_helper.h>
#include <gaden_filament_simulator/sensor/sensor_base.hpp>

namespace gaden::sensor {

auto getVisualisationPublisher(std::shared_ptr<rclcpp::Node> &ros_node) // Not thread safe!
{
    static std::weak_ptr<rclcpp::Publisher<visualization_msgs::msg::Marker>> visualisation_publisher;
    auto pub = visualisation_publisher.lock();
    if (!pub)
    {
        pub = ros_node->create_publisher<visualization_msgs::msg::Marker>(
                    "sensor_visualisation", 1);
        visualisation_publisher = pub;
    }
    return pub;
}

SensorBase::SensorBase(const std::string &sensor_type,
                       std::shared_ptr<GasDispersionModel> gas_model,
                       std::shared_ptr<rclcpp::Node> &ros_node,
                       const std::string &rviz_frame,
                       rl::Logger logger_object)
    : logger(logger_object)
    , gas_model(gas_model)
    , visualisation_publisher_(getVisualisationPublisher(ros_node))
{
    if (instance_counter_.find(sensor_type) == instance_counter_.end())
        instance_counter_[sensor_type] = 0;

    instance_number = instance_counter_[sensor_type]++;
    instance_name = sensor_type + std::to_string(instance_number);

    visualisation_marker.header.stamp = ros_node->get_clock()->now(); // TODO Update on publish?
    visualisation_marker.header.frame_id = rviz_frame;

    visualisation_marker.ns = sensor_type;
    visualisation_marker.id = instance_number;
    visualisation_marker.action = visualization_msgs::msg::Marker::ADD;

    // The following values should be overwritten by the derived class,
    // we just provide some defaults here.
    visualisation_marker.type = visualization_msgs::msg::Marker::CUBE;
    visualisation_marker.pose.position = ros_type::getPoint(0);
    visualisation_marker.pose.orientation = ros_type::DefaultOrientation::get();
    visualisation_marker.scale = ros_type::getVector3(1);
    visualisation_marker.color = ros_type::getColor(0, 1, 0);

    logger.info() << "Created " << instance_name;
}

SensorBase::~SensorBase()
{}

void SensorBase::publishVisualisationOnTimeout()
{
    if (t_next_visualisation_publish_ >= std::chrono::steady_clock::now())
        publishVisualisation();
}

void SensorBase::publishVisualisation()
{
    visualisation_publisher_->publish(visualisation_marker);
    using namespace std::chrono_literals;
    t_next_visualisation_publish_ = std::chrono::steady_clock::now() + 5s;
}

} // namespace gaden::sensor
