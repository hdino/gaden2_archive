#ifndef GADEN_SIMULATOR_SENSOR_OPEN_PATH_HPP_INCLUDED
#define GADEN_SIMULATOR_SENSOR_OPEN_PATH_HPP_INCLUDED

#include <Eigen/Core>
#include <olfaction_msgs/msg/open_path_measurement.hpp>

#include "sensor_base.hpp"

namespace gaden {
class EnvironmentModel;
} // namespace gaden

namespace gaden::sensor {

class OpenPath : public SensorBase
{
public:
    OpenPath(std::shared_ptr<EnvironmentModel> environment_model,
             std::shared_ptr<GasDispersionModel> gas_model,
             std::shared_ptr<rclcpp::Node> &ros_node,
             Eigen::Vector3d position,
             Eigen::Vector3d direction,
             const std::string &rviz_frame,
             rl::Logger logger_object);
    ~OpenPath();

    void doEvents();
    void updatePose(Eigen::Vector3d position,
                    Eigen::Vector3d direction);

private:
    olfaction_msgs::msg::OpenPathMeasurement measureRay(
            const Eigen::Vector3d &position,
            const Eigen::Vector3d &direction);

    std::shared_ptr<EnvironmentModel> environment_model_;

    Eigen::Vector3d position_;
    Eigen::Vector3d direction_;

    std::shared_ptr<rclcpp::Publisher<olfaction_msgs::msg::OpenPathMeasurement>> publisher_measurement_;
};

} // namespace gaden::sensor

#endif // GADEN_SIMULATOR_SENSOR_OPEN_PATH_HPP_INCLUDED
