#include <gaden_environment/type_helper.h>

#include <yaml-cpp/yaml.h>

namespace gaden::type_helper {

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

} // namespace gaden::type_helper
