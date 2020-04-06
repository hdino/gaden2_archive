#include <gaden_common/grid_to_marker.hpp>
#include <gaden_common/occupancy_grid.h>
#include <gaden_environment/environment.h>

#include <rclcpp/rclcpp.hpp>
#include <rl_logging/ros2_logging.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// ===============================//
//              MAIN              //
// ===============================//
int main( int argc, char** argv )
{
    //Init
    rclcpp::init(argc, argv);
    auto node_options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
    auto ros_node = std::make_shared<rclcpp::Node>("environment", node_options);
    rl::Logger logger = rl::logging::Ros2Logger::create(ros_node->get_logger());

    //Load Parameters
    auto config = gaden::loadEnvironmentConfig(ros_node, logger);
    logger.info() << "Configuration:\n    " << gaden::toString(config, 4);

    // Publishers
    auto gas_source_publisher = ros_node->create_publisher<
            visualization_msgs::msg::MarkerArray>("source_visualization", 10);
    auto environment_publisher = ros_node->create_publisher<
            visualization_msgs::msg::MarkerArray>("environment_visualization", 100);
    auto environment_cad_publisher = ros_node->create_publisher<
            visualization_msgs::msg::MarkerArray>("environment_cad_visualization", 100);

    // Subscribers
//    preprocessing_done = false;
//    ros::Subscriber sub = n.subscribe("preprocessing_done", 1, PreprocessingCB);

    //    // Wait for the GADEN_preprocessin node to finish?
    //    if( wait_preprocessing )
    //    {
    //        while(ros::ok() && !preprocessing_done)
    //        {
    //            ros::Duration(0.5).sleep();
    //            ros::spinOnce();
    //            if (verbose) ROS_INFO("[environment] Waiting for node GADEN_preprocessing to end.");
    //        }
    //	}


    // 1. ENVIRONMNET AS CAD MODELS
    //-------------------------------
    visualization_msgs::msg::MarkerArray cad_model_markers;

    for (size_t i = 0; i < config.cad_models.size(); ++i)
    {
        visualization_msgs::msg::Marker marker = gaden::getAsMarker(config.cad_models[i], i,
                                                                    ros_node->get_clock()->now(),
                                                                    config.fixed_frame);

        cad_model_markers.markers.push_back(marker);
    }

    // 2. ENVIRONMNET AS Occupancy3D file
    //------------------------------------
    //Display Environment as an array of Cube markers (Rviz)
    //visualization_msgs::MarkerArray environment;
    visualization_msgs::msg::MarkerArray environment;
    gaden::OccupancyGrid::Ptr occupancy_grid;
    if (!config.occupancy_grid_file.empty())
    {
        occupancy_grid = gaden::loadGridFromFile(config.occupancy_grid_file, logger);
        if (occupancy_grid)
            environment = gaden::getAsMarkerArray(occupancy_grid,
                                                  ros_node->get_clock()->now(),
                                                  config.fixed_frame,
                                                  logger);
    }
//    if (!occupancy3D_data.empty())
//        loadEnvironment(environment);

    // 3. GAS SOURCES
    //----------------
    //Generate Gas Source Markers
    //The shape are cylinders from the floor to the given z_size.
    visualization_msgs::msg::MarkerArray gas_source_markers;

    for (size_t i = 0; i < config.gas_sources.size(); ++i)
    {
        visualization_msgs::msg::Marker source = gaden::getAsMarker(config.gas_sources[i], i,
                                                                    ros_node->get_clock()->now(),
                                                                    config.fixed_frame);
        gas_source_markers.markers.push_back(source);
    }

    // Small sleep to allow RVIZ to startup
    //ros::Duration(1.0).sleep();

    //---------------
    //      LOOP
    //---------------
    rclcpp::Rate rate(0.3); //Just to refresh from time to time
    while (rclcpp::ok())
    {
        //Publish CAD Markers
        environment_cad_publisher->publish(cad_model_markers);

        // Publish 3D Occupancy
        if (occupancy_grid)
            environment_publisher->publish(environment);

        //Publish Gas Sources
        gas_source_publisher->publish(gas_source_markers);

        rclcpp::spin_some(ros_node);
        rate.sleep();
    }
}
