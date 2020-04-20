#include <gaden_common/eigen_helper.hpp>
#include <gaden_common/ros_parameters.h>
#include <gaden_common/ros_type_helper.h>
#include <gaden_common/to_string.hpp>
#include <gaden_filament_simulator/simulator_config.hpp>

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

namespace gaden {

std::string toString(const VisualisationConfig &config, size_t indention)
{
    std::string newline = "\n" + std::string(indention, ' ');
    return "Visualise environment: " + toString(config.visualise_environment) +
           newline +
           "Visualise bounding box: " + toString(config.visualise_bounding_box) +
           newline +
           "Fixed frame: " + config.fixed_frame;
}

std::string toString(const SimulatorConfig &config, size_t indention)
{
    std::string newline = "\n" + std::string(indention, ' ');
    return "Base path: " + config.base_path.string() +
           newline +
           "Base config file: " + config.base_config_file.string() +
           newline +
           "Occupancy file: " + config.occupancy_file.string() +
           newline +
           "Recreate existing occupancy: " + toString(config.recreate_existing_occupancy) +
           newline + toString(config.visualisation, indention) +
           newline +
           "Gas sources: " +
           newline + "  " + toString(config.gas_sources, indention + 2);
}

VisualisationConfig loadVisualisationConfig(const YAML::Node &yaml_visualisation)
{
    VisualisationConfig config;

    config.fixed_frame = yaml_visualisation["fixed_frame"].as<std::string>();
    config.visualise_environment = yaml_visualisation["visualise_environment"].as<bool>();
    config.visualise_bounding_box = yaml_visualisation["visualise_bounding_box"].as<bool>();

    return config;
}

SimulatorConfig loadSimulatorConfig(std::shared_ptr<rclcpp::Node> &ros_node)
{
    SimulatorConfig config;

    std::string base_path_string;
    if (!getNodeParameter(ros_node, "base_path", base_path_string))
        return config;
    config.base_path = base_path_string;
    config.base_config_file = config.base_path / "config.yaml";

    YAML::Node yaml_config = YAML::LoadFile(config.base_config_file);

    config.occupancy_file = config.base_path / yaml_config["occupancy_file"].as<std::string>();
    config.recreate_existing_occupancy = yaml_config["recreate_existing_occupancy"].as<bool>();

    // fill in gas sources
    config.gas_sources = loadAllGasSourcesFrom(yaml_config);

    config.visualisation = loadVisualisationConfig(yaml_config["visualisation"]);

    config.wind_model = yaml_config["wind"]["format"].as<std::string>();

    return config;
}

} // namespace gaden
