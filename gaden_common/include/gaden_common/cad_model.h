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
};
std::string toString(const CadModel &cad_model, size_t indention = 0);

struct CadModelColor : public CadModel
{
    std_msgs::msg::ColorRGBA color;
};
std::string toString(const CadModelColor &cad_model, size_t indention = 0);

CadModel getCadModelFromYaml(const YAML::Node &node, const std::string &base_path);
CadModelColor getCadModelColorFromYaml(const YAML::Node &node, const std::string &base_path);

visualization_msgs::msg::Marker getAsMarker(const CadModelColor &cad_model, int id,
                                            const builtin_interfaces::msg::Time &stamp,
                                            const std::string &frame_id);

} // namespace gaden

#endif // GADEN_COMMON_CAD_MODEL_H_INCLUDED
