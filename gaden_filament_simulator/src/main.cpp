#include <rclcpp/rclcpp.hpp>
#include <rl_logging/ros2_logging.hpp>

//#include <gaden_filament_simulator/filament_simulator.h>
#include <gaden_filament_simulator/environment_visualisation.hpp>
#include <gaden_filament_simulator/occupancy.hpp>
#include <gaden_filament_simulator/simulator_config.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node_options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
    auto ros_node = std::make_shared<rclcpp::Node>("simulator_main", node_options);

    rl::Logger logger = rl::logging::Ros2Logger::create(ros_node->get_logger());

    gaden::SimulatorConfig config = gaden::loadSimulatorConfig(ros_node);
    logger.info() << "Configuration:\n    " << gaden::toString(config, 4);

    if (config.recreate_existing_occupancy ||
        !std::filesystem::exists(config.occupancy_file))
    {
        if (!gaden::generateOccupancyFile(config, logger))
        {
            logger.error("Creating occupancy file failed.");
            return 0;
        }
    }

    gaden::EnvironmentVisualiser environment_visualiser(config);

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
    rclcpp::spin(ros_node);
}
