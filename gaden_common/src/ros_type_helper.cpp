#include <gaden_common/ros_type_helper.h>

#include <yaml-cpp/yaml.h>

namespace gaden {

std::string toString(const geometry_msgs::msg::Point &point, size_t indention)
{
    (void)indention;
    return "["  + std::to_string(point.x) +
           ", " + std::to_string(point.y) +
           ", " + std::to_string(point.z) + "]";
}

std::string toString(const std_msgs::msg::ColorRGBA &color, size_t indention)
{
    (void)indention;
    return "["  + std::to_string(color.r) +
           ", " + std::to_string(color.g) +
           ", " + std::to_string(color.b) +
           ", " + std::to_string(color.a) + "]";
}

} // namespace gaden

namespace gaden::ros_type {

geometry_msgs::msg::Vector3 getVector3(double x, double y, double z)
{
    geometry_msgs::msg::Vector3 vector;
    vector.x = x;
    vector.y = y;
    vector.z = z;
    return vector;
}

geometry_msgs::msg::Vector3 getVector3(double value)
{
    return getVector3(value, value, value);
}

geometry_msgs::msg::Vector3 getVector3From(const Eigen::Vector3d &v)
{
    return getVector3(v[0], v[1], v[2]);
}

geometry_msgs::msg::Point getPoint(double x, double y, double z)
{
    geometry_msgs::msg::Point point;
    point.x = x;
    point.y = y;
    point.z = z;
    return point;
}

geometry_msgs::msg::Point getPoint(double value)
{
    return getPoint(value, value, value);
}

geometry_msgs::msg::Point getPointFrom(const Eigen::Vector3d &v)
{
    return getPoint(v[0], v[1], v[2]);
}

geometry_msgs::msg::Point getPointFrom(const YAML::Node &sequence)
{
    return getPoint(sequence[0].as<double>(),
                    sequence[1].as<double>(),
                    sequence[2].as<double>());
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

std_msgs::msg::ColorRGBA getColorFrom(const YAML::Node &sequence)
{
    return getColor(sequence[0].as<float>(),
                    sequence[1].as<float>(),
                    sequence[2].as<float>(),
                    sequence.size() > 3 ? sequence[3].as<float>() : 1.0f);
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

geometry_msgs::msg::Quaternion getQuaternion(double w, double x, double y, double z)
{
    geometry_msgs::msg::Quaternion q;
    q.w = w;
    q.x = x;
    q.y = y;
    q.z = z;
    return q;
}

geometry_msgs::msg::Quaternion getQuaternionFrom(const Eigen::Quaterniond &q)
{
    return getQuaternion(q.w(), q.x(), q.y(), q.z());
}

} // namespace gaden::ros_type
