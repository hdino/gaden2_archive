#ifndef GADEN_COMMON_GAS_SOURCE_HPP_INCLUDED
#define GADEN_COMMON_GAS_SOURCE_HPP_INCLUDED

#include <string>

#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace gaden {

struct GasSource
{
    geometry_msgs::msg::Point position;
    std_msgs::msg::ColorRGBA color;
    double scale;
};
std::string toString(const GasSource &gas_source, size_t indention = 0);

visualization_msgs::msg::Marker getAsMarker(const GasSource &gas_source, int id,
                                            const builtin_interfaces::msg::Time &stamp,
                                            const std::string &frame_id);

} // namespace gaden

#endif // GADEN_COMMON_GAS_SOURCE_HPP_INCLUDED
