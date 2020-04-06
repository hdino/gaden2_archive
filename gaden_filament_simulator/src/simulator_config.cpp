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
    for (const YAML::Node &item : yaml_config["gas_sources"])
    {
        std::cout << "GS: " << item << std::endl;
        GasSource gas_source;
        gas_source.position = ros_type::getPositionFromYaml(item);
        gas_source.scale = item["scale"].as<double>();
        gas_source.color = ros_type::getColorFromYaml(item);
        config.gas_sources.push_back(gas_source);
    }

    config.visualisation = loadVisualisationConfig(yaml_config["visualisation"]);

    return config;
}

} // namespace gaden
