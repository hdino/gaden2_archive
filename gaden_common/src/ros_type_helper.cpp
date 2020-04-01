#include <gaden_common/ros_type_helper.h>

#include <yaml-cpp/yaml.h>

namespace gaden::ros_type {

std_msgs::msg::ColorRGBA getColorFromYaml(const YAML::Node &parent)
{
    YAML::Node sequence = parent["color"];

    std_msgs::msg::ColorRGBA color;
    color.r = sequence[0].as<float>();
    color.g = sequence[1].as<float>();
    color.b = sequence[2].as<float>();
    if (sequence.size() > 3)
        color.a = sequence[3].as<float>();
    else
        color.a = 1.0f;
    return color;
}

geometry_msgs::msg::Point getPositionFromYaml(const YAML::Node &parent)
{
    YAML::Node sequence = parent["position"];

    geometry_msgs::msg::Point point;
    point.x = sequence[0].as<double>();
    point.y = sequence[1].as<double>();
    point.z = sequence[2].as<double>();
    return point;
}

geometry_msgs::msg::Vector3 getVector3(double value)
{
    geometry_msgs::msg::Vector3 vector;
    vector.x = vector.y = vector.z = value;
    return vector;
}

geometry_msgs::msg::Point getPoint(double x, double y, double z)
{
    geometry_msgs::msg::Point point;
    point.x = x;
    point.y = y;
    point.z = z;
    return point;
}

std_msgs::msg::ColorRGBA getColor(float r, float g, float b, float a)
{
    std_msgs::msg::ColorRGBA c;
    c.r = r;
    c.g = g;
    c.b = b;
    c.a = a;
    return c;
}

const geometry_msgs::msg::Quaternion & DefaultOrientation::get()
{
    static DefaultOrientation instance;
    return instance.q;
}

DefaultOrientation::DefaultOrientation()
{
    q.x = 0.0;
    q.y = 0.0;
    q.z = 0.0;
    q.w = 1.0;
}

} // namespace gaden::ros_type
