#ifndef GADEN_COMMON_ROS_TYPE_HELPER_H_INCLUDED
#define GADEN_COMMON_ROS_TYPE_HELPER_H_INCLUDED

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/color_rgba.hpp>

namespace YAML {
class Node;
}

namespace gaden::ros_type {

std_msgs::msg::ColorRGBA getColorFromYaml(const YAML::Node &parent);

geometry_msgs::msg::Point getPositionFromYaml(const YAML::Node &parent);

geometry_msgs::msg::Vector3 getVector3(double value);

geometry_msgs::msg::Point getPoint(double x, double y, double z);

inline geometry_msgs::msg::Point getPoint(double value)
{
    return getPoint(value, value, value);
}

std_msgs::msg::ColorRGBA getColor(float r, float g, float b, float a = 1.0);

class DefaultOrientation
{
public:
    static const geometry_msgs::msg::Quaternion & get();

private:
    DefaultOrientation();
    geometry_msgs::msg::Quaternion q;
};

} // namespace gaden::ros_type

#endif // GADEN_COMMON_ROS_TYPE_HELPER_H_INCLUDED
