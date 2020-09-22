#include <rclcpp/rclcpp.hpp>
#include <rl_logging/ros2_logging.hpp>
#include <yaml-cpp/yaml.h>

#include <gaden_common/occupancy_grid.h>
#include <gaden_filament_simulator/environment_visualisation.hpp>
#include <gaden_filament_simulator/farrells_wind_model.hpp>
#include <gaden_filament_simulator/filament_model.hpp>
#include <gaden_filament_simulator/filament_model_rviz.hpp>
#include <gaden_filament_simulator/inline_wind_model.hpp>
#include <gaden_filament_simulator/occupancy.hpp>
#include <gaden_filament_simulator/openvdb_environment_model.hpp>
#include <gaden_filament_simulator/simulator.hpp>
#include <gaden_filament_simulator/simulator_config.hpp>
#include <gaden_filament_simulator/sensor/insitu.hpp>
#include <gaden_filament_simulator/sensor/open_path.hpp>

int main(int argc, char **argv)
{
    std::string arguments;
    for (int i = 0; i < argc; ++i)
        arguments += std::string(argv[i]) + " ";
    arguments.pop_back();
    std::cout << arguments << std::endl;

    // 1. Init ROS and create a logger
    rclcpp::init(argc, argv);
    auto node_options = rclcpp::NodeOptions()
            .automatically_declare_parameters_from_overrides(true);
    auto ros_node =
            std::make_shared<rclcpp::Node>("simulator", node_options);

    rl::Logger logger =
            rl::logging::Ros2Logger::create(ros_node->get_logger());

    // 2. Load the configuration
    gaden::SimulatorConfig config = gaden::loadSimulatorConfig(ros_node);
    YAML::Node yaml_config = YAML::LoadFile(config.base_config_file);
    logger.info() << "Configuration:\n    " << gaden::toString(config, 4);

    // 3. Generate the occupancy grid file, if required
    if (config.recreate_existing_occupancy ||
        !std::filesystem::exists(config.occupancy_file))
    {
        if (!gaden::generateOccupancyFile(config, logger))
        {
            logger.error("Creating occupancy file failed.");
            return 0;
        }
    }

    // TODO: Environment visualisation
    gaden::EnvironmentVisualiser environment_visualiser(config);

    // 4. Load the occupancy grid file and create the environment model
    gaden::OccupancyGrid::Ptr grid =
            gaden::loadGridFromFile(config.occupancy_file, logger);
    auto environment_model =
            std::make_shared<gaden::OpenVdbEnvironmentModel>(grid, logger);

    // 5. Create the gas dispersion model
    auto gas_dispersion_model =
            std::make_shared<gaden::FilamentModel>(
                yaml_config,
                environment_model,
                logger);

    // 6. Create the wind model
    std::shared_ptr<gaden::WindModel> wind_model;
    if (config.wind_model == "inline")
        wind_model =
                std::make_shared<gaden::InlineWindModel>(
                    yaml_config, logger);
    else if (config.wind_model == "farrell")
        wind_model =
                std::make_shared<gaden::FarrellsWindModel>(
                    yaml_config, environment_model, logger);
    else
    {
        logger.error() << "Unsupported wind model specified: "
                       << config.wind_model;
        return 0;
    }

    // 7. Create the simulator
    auto simulator =
            gaden::Simulator::create(
                config,
                environment_model,
                gas_dispersion_model,
                wind_model,
                logger);

    auto filament_visualisation =
            std::make_shared<gaden::FilamentModelRvizVisualisation>(
                ros_node,
                gas_dispersion_model,
                config.visualisation.fixed_frame,
                0.25,
                logger);

    gaden::sensor::InSitu insitu_sensor(
                gas_dispersion_model,
                ros_node,
                Eigen::Vector3d(-20, 0, 15),
                config.visualisation.fixed_frame,
                logger);

    gaden::sensor::OpenPath openpath_sensor(
                environment_model, gas_dispersion_model,
                ros_node,
                Eigen::Vector3d(-20, 0, 20), Eigen::Vector3d(0, 0, -1),
                config.visualisation.fixed_frame,
                logger);

    rclcpp::executors::SingleThreadedExecutor ros_executor;
    while (rclcpp::ok() && simulator->simulate())
    {
        filament_visualisation->publish();
        insitu_sensor.doEvents();
        openpath_sensor.doEvents();
        ros_executor.spin_node_some(ros_node);
    }
}
