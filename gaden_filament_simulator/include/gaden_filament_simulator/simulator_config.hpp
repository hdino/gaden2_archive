#ifndef GADEN_SIMULATOR_SIMULATOR_CONFIG_HPP_INCLUDED
#define GADEN_SIMULATOR_SIMULATOR_CONFIG_HPP_INCLUDED

#include <memory>
#include <string>
#include <vector>

#include <gaden_common/filesystem.h>
#include <gaden_common/gas_source.hpp>

namespace rclcpp {
class Node;
}

namespace gaden {

struct VisualisationConfig
{
    bool visualise_environment;
    bool visualise_bounding_box;
    std::string fixed_frame;
};
std::string toString(const VisualisationConfig &config, size_t indention = 0);

struct SimulatorConfig
{
    std::filesystem::path base_path;
    std::filesystem::path base_config_file;

    std::filesystem::path occupancy_file;
    bool recreate_existing_occupancy;

    std::vector<GasSource> gas_sources;

    VisualisationConfig visualisation;

    std::string wind_model;
};
std::string toString(const SimulatorConfig &config, size_t indention = 0);

SimulatorConfig loadSimulatorConfig(std::shared_ptr<rclcpp::Node> &ros_node);

} // namespace gaden

#endif // GADEN_SIMULATOR_SIMULATOR_CONFIG_HPP_INCLUDED
