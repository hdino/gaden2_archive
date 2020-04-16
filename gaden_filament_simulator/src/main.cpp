#include <rclcpp/rclcpp.hpp>
#include <rl_logging/ros2_logging.hpp>

#include <gaden_common/occupancy_grid.h>
//#include <gaden_filament_simulator/filament_simulator.h>
#include <gaden_filament_simulator/environment_visualisation.hpp>
#include <gaden_filament_simulator/filament_model.hpp>
#include <gaden_filament_simulator/filament_model_rviz.hpp>
#include <gaden_filament_simulator/inline_wind_model.hpp>
#include <gaden_filament_simulator/occupancy.hpp>
#include <gaden_filament_simulator/openvdb_environment_model.hpp>
#include <gaden_filament_simulator/simulator.hpp>
#include <gaden_filament_simulator/simulator_config.hpp>

int main(int argc, char **argv)
{
    // 1. Init ROS and create a logger
    rclcpp::init(argc, argv);
    auto node_options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
    auto ros_node = std::make_shared<rclcpp::Node>("simulator", node_options);

    rl::Logger logger = rl::logging::Ros2Logger::create(ros_node->get_logger());

    // 2. Load the configuration
    gaden::SimulatorConfig config = gaden::loadSimulatorConfig(ros_node);
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
    gaden::OccupancyGrid::Ptr grid = gaden::loadGridFromFile(config.occupancy_file, logger);
    auto env_model = std::make_shared<gaden::OpenVdbEnvironmentModel>(grid, logger);

    // 5. Create the gas dispersion model
    auto gas_dispersion_model = std::make_shared<gaden::FilamentModel>(0.05, logger);

    // 6. Create the wind model
    auto wind_model = std::make_shared<gaden::InlineWindModel>();

    // 7. Create the simulator
    auto simulator = gaden::Simulator::create(config,
                                              env_model,
                                              gas_dispersion_model,
                                              wind_model,
                                              logger);

    auto filament_visualisation = std::make_shared<gaden::FilamentModelRvizVisualisation>(
                ros_node, gas_dispersion_model, config.visualisation.fixed_frame,
                gas_dispersion_model->getCellSize() * 10, logger);

    //rclcpp::Logger logger = ros_node->get_logger();

    // Wait preprocessing Node to finish?
//    preprocessing_done = false;
//    if(wait_preprocessing)
//    {
//        prepro_sub = n.subscribe("preprocessing_done", 1, &CFilamentSimulator::preprocessingCB, this);
//        while(ros::ok() && !preprocessing_done)
//        {
//            ros::Duration(0.5).sleep();
//            ros::spinOnce();
//            if (verbose) ROS_INFO("[filament] Waiting for node GADEN_preprocessing to end.");
//        }
//    }

    //Create simulator obj and initialize it
    //CFilamentSimulator sim;
    //gaden::FilamentSimulator sim;

    // Initiate Random Number generator with current time
    //srand(time(NULL)); // TODO Remove when all random functions use <random>

    //--------------
    // LOOP
    //--------------
    //ros::Rate r(100);
    //rclcpp::Rate rate(100)
    //while (ros::ok() && (sim.current_simulation_step<sim.numSteps) )
//    while (rclcpp::ok() && sim.simulate())
//    {
//        //r.sleep();
//        //ros::spinOnce();
//        rclcpp::spin_some(ros_node);
//    }
//    while (ros::ok())
//    {
//        rclcpp::
//    }

    while (rclcpp::ok() && simulator->simulate())
    {
        filament_visualisation->publish();
        rclcpp::spin_some(ros_node);
    }

    //rclcpp::spin(ros_node);
}
