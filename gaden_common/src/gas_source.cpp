#include <gaden_common/gas_source.hpp>
#include <gaden_common/ros_type_helper.h>
#include <gaden_common/to_string.hpp>

namespace gaden {

std::string toString(const GasSource &gas_source, size_t indention)
{
    std::string newline = "\n" + std::string(indention, ' ');
    return "Position: " + toString(gas_source.position, indention + 4) +
           newline +
           "Color:    " + toString(gas_source.color, indention + 4) +
           newline +
           "Scale:    " + toString(gas_source.scale);
}

visualization_msgs::msg::Marker getAsMarker(const GasSource &gas_source, int id,
                                            const builtin_interfaces::msg::Time &stamp,
                                            const std::string &frame_id)
{
    visualization_msgs::msg::Marker source;
    source.header.stamp = stamp;
    source.header.frame_id = frame_id;

    source.id = id;
    source.ns = "gas_sources";
    source.action = visualization_msgs::msg::Marker::ADD;
    source.type = visualization_msgs::msg::Marker::CUBE;

    source.pose.position = gas_source.position;
    source.pose.position.z *= 0.5;

    source.scale = ros_type::getVector3(gas_source.scale);
    source.scale.z = gas_source.position.z;

    source.color = gas_source.color;

    source.pose.orientation = ros_type::DefaultOrientation::get();

    return source;
}

} // namespace gaden
