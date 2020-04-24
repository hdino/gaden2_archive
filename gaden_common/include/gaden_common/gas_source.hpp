#ifndef GADEN_COMMON_GAS_SOURCE_HPP_INCLUDED
#define GADEN_COMMON_GAS_SOURCE_HPP_INCLUDED

#include <string>
#include <vector>

#include <Eigen/Core>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace YAML {
class Node;
}

namespace gaden {

struct GasSourceVisualisation
{
    std_msgs::msg::ColorRGBA color;
    double scale;
};
std::string toString(const GasSourceVisualisation &gas_source, size_t indention = 0);

struct GasSource
{
    Eigen::Vector3d position;

    double release_rate; // [kg/h]
    //bool variable_release_rate;
        // If false, the release rate of the gas source will be used;
        // if true, a poisson process is used with the release rate of the
        // gas source as the distribution's lambda parameter

    GasSourceVisualisation visualisation;
};
std::string toString(const GasSource &gas_source, size_t indention = 0);

GasSource loadGasSourceFrom(const YAML::Node &yaml_node);
std::vector<GasSource> loadAllGasSourcesFrom(const YAML::Node &yaml_node);

visualization_msgs::msg::Marker getAsMarker(const GasSource &gas_source, int id,
                                            const builtin_interfaces::msg::Time &stamp,
                                            const std::string &frame_id);

} // namespace gaden

#endif // GADEN_COMMON_GAS_SOURCE_HPP_INCLUDED
