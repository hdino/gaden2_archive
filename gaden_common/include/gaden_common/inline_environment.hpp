#ifndef GADEN_COMMON_INLINE_ENVIRONMENT_HPP_INCLUDED
#define GADEN_COMMON_INLINE_ENVIRONMENT_HPP_INCLUDED

#include <openvdb/Types.h>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace YAML {
class Node;
}

namespace gaden {

//struct Box
//{
//    openvdb::Vec3d origin;
//    openvdb::Vec3d size;
//};
//std::string toString(const Box &box, size_t indention = 0);

//struct ColoredBox : public Box
//{
//    std_msgs::msg::ColorRGBA color;
//};
//std::string toString(const ColoredBox &box, size_t indention = 0);

//Box getBox(const YAML::Node &box_data);
//ColoredBox getColoredBox(const YAML::Node &box_data);

//visualization_msgs::msg::Marker getAsMarker(const ColoredBox &box, int id,
//                                            const builtin_interfaces::msg::Time &stamp,
//                                            const std::string &frame_id);

} // namespace gaden

#endif // GADEN_COMMON_INLINE_ENVIRONMENT_HPP_INCLUDED
