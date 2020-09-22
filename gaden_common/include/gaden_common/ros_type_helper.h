#ifndef GADEN_COMMON_ROS_TYPE_HELPER_H_INCLUDED
#define GADEN_COMMON_ROS_TYPE_HELPER_H_INCLUDED

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/color_rgba.hpp>

namespace YAML {
class Node;
}

namespace gaden {
std::string toString(const geometry_msgs::msg::Point &point, size_t indention = 0);
std::string toString(const std_msgs::msg::ColorRGBA &color, size_t indention = 0);
} // namespace gaden

namespace gaden::ros_type {

geometry_msgs::msg::Vector3 getVector3(double x, double y, double z);
geometry_msgs::msg::Vector3 getVector3(double value);
geometry_msgs::msg::Vector3 getVector3From(const Eigen::Vector3d &v);

geometry_msgs::msg::Point getPoint(double x, double y, double z);
geometry_msgs::msg::Point getPoint(double value);
geometry_msgs::msg::Point getPointFrom(const Eigen::Vector3d &v);
geometry_msgs::msg::Point getPointFrom(const YAML::Node &sequence);

std_msgs::msg::ColorRGBA getColor(float r, float g, float b, float a = 1.0);
std_msgs::msg::ColorRGBA getColorFrom(const YAML::Node &sequence);

class DefaultOrientation
{
public:
    static const geometry_msgs::msg::Quaternion & get();

private:
    DefaultOrientation();
    geometry_msgs::msg::Quaternion q;
};

geometry_msgs::msg::Quaternion getQuaternion(double w, double x, double y, double z);
geometry_msgs::msg::Quaternion getQuaternionFrom(const Eigen::Quaterniond &q);

template <typename TRosType>
void fillInXyz(TRosType &ros_type, const Eigen::Vector3d &v)
{
    ros_type.x = v[0];
    ros_type.y = v[0];
    ros_type.z = v[0];
}

} // namespace gaden::ros_type

#endif // GADEN_COMMON_ROS_TYPE_HELPER_H_INCLUDED
