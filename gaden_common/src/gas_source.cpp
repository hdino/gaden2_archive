#include <gaden_common/eigen_helper.hpp>
#include <gaden_common/gas_source.hpp>
#include <gaden_common/ros_type_helper.h>
#include <gaden_common/to_string.hpp>

#include <yaml-cpp/yaml.h>

namespace gaden {

std::string toString(const GasSource &gas_source, size_t indention)
{
    std::string newline = "\n" + std::string(indention, ' ');
    return "Position: " + toString(gas_source.position, indention + 4) +
           newline +
           "Filament release rate: " + toString(gas_source.num_filaments_per_second) + "/s" +
           newline +
           "Color:    " + toString(gas_source.color, indention + 4) +
           newline +
           "Scale:    " + toString(gas_source.scale);
}

GasSource loadGasSourceFrom(const YAML::Node &yaml_node)
{
    GasSource gas_source;
    gas_source.position = eigen_helper::getVector3dFromYaml(yaml_node["position"]);
    gas_source.num_filaments_per_second = yaml_node["num_filaments_per_second"].as<unsigned>();
    gas_source.scale = yaml_node["scale"].as<double>();
    gas_source.color = ros_type::getColorFromYaml(yaml_node);
    return gas_source;
}

std::vector<GasSource> loadAllGasSourcesFrom(const YAML::Node &yaml_node)
{
    std::vector<GasSource> gas_sources;

    for (const YAML::Node &item : yaml_node["gas_sources"])
        gas_sources.push_back(loadGasSourceFrom(item));

    return gas_sources;
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

    source.pose.position = ros_type::getPointFrom(gas_source.position);
    source.pose.position.z *= 0.5;

    source.scale = ros_type::getVector3(gas_source.scale);
    source.scale.z = source.pose.position.z;

    source.color = gas_source.color;

    source.pose.orientation = ros_type::DefaultOrientation::get();

    return source;
}

} // namespace gaden
