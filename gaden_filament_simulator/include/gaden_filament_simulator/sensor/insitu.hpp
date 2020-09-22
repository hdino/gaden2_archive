#ifndef GADEN_SIMULATOR_SENSOR_INSITU_HPP_INCLUDED
#define GADEN_SIMULATOR_SENSOR_INSITU_HPP_INCLUDED

#include <Eigen/Core>
#include <std_msgs/msg/float64.hpp>

#include "sensor_base.hpp"

namespace gaden::sensor {

class InSitu : public SensorBase
{
public:
    InSitu(std::shared_ptr<GasDispersionModel> gas_model,
           std::shared_ptr<rclcpp::Node> &ros_node,
           Eigen::Vector3d position,
           const std::string &rviz_frame,
           rl::Logger logger_object);
    ~InSitu();

    void doEvents();
    void updatePosition(Eigen::Vector3d position);

private:
    Eigen::Vector3d position_;

    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64>> publisher_measurement_;
};

} // namespace gaden::sensor

#endif // GADEN_SIMULATOR_SENSOR_INSITU_HPP_INCLUDED
