#include <gaden_common/inline_environment.hpp>
#include <gaden_common/openvdb_helper.h>
#include <gaden_common/ros_type_helper.h>

#include <yaml-cpp/yaml.h>

namespace gaden {

//std::string toString(const Box &box, size_t indention)
//{
//    std::string newline = "\n" + std::string(indention, ' ');
//    return "Origin: " + toString(box.origin) +
//           newline +
//           "Size: " + toString(box.size);
//}

//std::string toString(const ColoredBox &box, size_t indention)
//{
//    std::string newline = "\n" + std::string(indention, ' ');
//    return toString(static_cast<const Box &>(box))
//           + newline +
//           "Color: " + toString(box.color);
//}

//Box getBox(const YAML::Node &box_data)
//{
//    Box box;
//    box.origin = openvdb_helper::getVec3FromYaml<double>(box_data["origin"]);
//    box.size = openvdb_helper::getVec3FromYaml<double>(box_data["size"]);
//    return box;
//}

//ColoredBox getColoredBox(const YAML::Node &box_data)
//{
//    ColoredBox box;
//    static_cast<Box &>(box) = getBox(box_data);
//    box.color = ros_type::getColorFromYaml(box_data);
//    return box;
//}

//visualization_msgs::msg::Marker getAsMarker(const ColoredBox &box, int id,
//                                            const builtin_interfaces::msg::Time &stamp,
//                                            const std::string &frame_id)
//{
//    visualization_msgs::msg::Marker marker;
//    marker.header.frame_id = frame_id;
//    marker.header.stamp = stamp;
//    marker.ns = "box";
//    marker.id = id;
//    marker.type = visualization_msgs::msg::Marker::CUBE;
//    marker.action = visualization_msgs::msg::Marker::ADD;

//    openvdb::Vec3d box_center = box.origin + 0.5 * box.size;
//    marker.pose.position.x = box_center.x();
//    marker.pose.position.y = box_center.y();
//    marker.pose.position.z = box_center.z();
//    marker.pose.orientation = ros_type::DefaultOrientation::get();

//    marker.scale.x = box.size.x();
//    marker.scale.y = box.size.y();
//    marker.scale.z = box.size.z();

//    marker.color = box.color;

//    return marker;
//}

} // namespace gaden
