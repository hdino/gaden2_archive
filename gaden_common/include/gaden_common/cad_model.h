#ifndef GADEN_COMMON_CAD_MODEL_H_INCLUDED
#define GADEN_COMMON_CAD_MODEL_H_INCLUDED

#include <string>

#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace YAML {
class Node;
}

namespace gaden {

struct CadModel
{
    std::string path;
    std_msgs::msg::ColorRGBA color;
};

CadModel getCadModelFromYaml(const YAML::Node &node, const std::string &base_path);

visualization_msgs::msg::Marker getAsMarker(const CadModel &cad_model, int id,
                                            const builtin_interfaces::msg::Time &stamp,
                                            const std::string &frame_id);

} // namespace gaden

#endif // GADEN_COMMON_CAD_MODEL_H_INCLUDED
