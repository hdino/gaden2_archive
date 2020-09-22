#include <gaden_common/ros_type_helper.h>
#include <gaden_filament_simulator/environment_model.hpp>
#include <gaden_filament_simulator/gas_dispersion_model.hpp>
#include <gaden_filament_simulator/sensor/open_path.hpp>

#include <Eigen/Core>

namespace gaden::sensor {

OpenPath::OpenPath(std::shared_ptr<EnvironmentModel> environment_model,
                   std::shared_ptr<GasDispersionModel> gas_model,
                   std::shared_ptr<rclcpp::Node> &ros_node,
                   Eigen::Vector3d position, Eigen::Vector3d direction,
                   const std::string &rviz_frame,
                   rl::Logger logger_object)
    : SensorBase("openpath_sensor", gas_model, ros_node, rviz_frame, logger_object)
    , environment_model_(environment_model)
    , publisher_measurement_(
          ros_node->create_publisher<olfaction_msgs::msg::OpenPathMeasurement>(
              instance_name, 10))
{
    visualisation_marker.type = visualization_msgs::msg::Marker::ARROW;
    visualisation_marker.scale = ros_type::getVector3(1.2, 0.4, 0.4);

    updatePose(position, direction);
}

OpenPath::~OpenPath()
{}

void OpenPath::updatePose(Eigen::Vector3d position, Eigen::Vector3d direction)
{
    position_ = position;
    direction_ = direction.normalized();

    visualisation_marker.pose.position = ros_type::getPointFrom(position_);
    Eigen::Quaterniond orientation_quaternion;
    orientation_quaternion.setFromTwoVectors(Eigen::Vector3d(1,0,0), direction_);
    visualisation_marker.pose.orientation = ros_type::getQuaternionFrom(orientation_quaternion);

    publishVisualisation();
}

olfaction_msgs::msg::OpenPathMeasurement OpenPath::measureRay(
        const Eigen::Vector3d &position,
        const Eigen::Vector3d &direction)
{
    olfaction_msgs::msg::OpenPathMeasurement measurement;

    CollisionTestResult collision = environment_model_->getCollisionDistance(position_, direction_, 60);
    //logger.info() << "Cillision with " << toString(collision.collision_type) << " d=" << collision.distance;
    if (collision.collision_type == Occupancy::Occupied || collision.collision_type == Occupancy::Outlet)
    {
        measurement.concentration = 0; // [ppm*m]
        double dx = 0.2; // [m], distance increment
        for (double distance = 0; distance < collision.distance; distance += dx)
        {
            Eigen::Vector3d p = position + distance * direction;
            measurement.concentration += gas_model->getConcentrationAt(p) * dx;
        }

        measurement.range = collision.distance;
        measurement.status = olfaction_msgs::msg::OpenPathMeasurement::STATUS_VALID;
    }
    else
    {
        // set measurement values to -1, so that everyone knows they are invalid
        measurement.concentration = -1;
        measurement.range = -1;
        measurement.status = olfaction_msgs::msg::OpenPathMeasurement::STATUS_INVALID;
    }

    return measurement;
}

void OpenPath::doEvents()
{
    auto measurement = measureRay(position_, direction_);

    // fill in position
    measurement.position.coordinate_system = olfaction_msgs::msg::FramePoint::CARTESIAN_FRAME;
    ros_type::fillInXyz(measurement.position, position_);

    // fill in orientation
    measurement.orientation.format = olfaction_msgs::msg::FrameOrientation::UNIT_VECTOR;
    ros_type::fillInXyz(measurement.orientation, direction_);

    publisher_measurement_->publish(measurement);

    publishVisualisationOnTimeout();
}

} // namespace gaden::sensor
