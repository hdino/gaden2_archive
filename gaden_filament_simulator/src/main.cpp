#include <rclcpp/rclcpp.hpp>

#include <gaden_filament_simulator/filament_simulator.h>

//==============================//
//			MAIN                //
//==============================//
int main(int argc, char **argv)
{
    // Init ROS-NODE
    //ros::init(argc, argv, "new_filament_simulator");
    //Init
    rclcpp::init(argc, argv);
    auto node_options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
    auto ros_node = std::make_shared<rclcpp::Node>("filament_simulator", node_options);
    rclcpp::Logger logger = ros_node->get_logger();

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
    gaden::FilamentSimulator sim;

    // Initiate Random Number generator with current time
    //srand(time(NULL)); // TODO Remove when all random functions use <random>

    //--------------
    // LOOP
    //--------------
    //ros::Rate r(100);
    //rclcpp::Rate rate(100)
    //while (ros::ok() && (sim.current_simulation_step<sim.numSteps) )
    while (rclcpp::ok() && sim.simulate())
    {
        //r.sleep();
        //ros::spinOnce();
        rclcpp::spin_some(ros_node);
    }
}
