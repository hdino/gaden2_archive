/*
 * The only goal of this Node is to display the simulation environment and gas source location in RVIZ.
 *
 * 1. Loads the simulation environment (usually from CFD in the file format .env), and displays it as RVIZ markers.
 * 2. Displays the Gas-Source Location as two cylinders.
 */

#include <gaden_common/eigen_helper.hpp>
#include <gaden_common/ros_parameters.h>
#include <gaden_common/ros_type_helper.h>
#include <gaden_environment/environment.h>

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

//#include <iostream>

namespace gaden {

//std::string toString(const GasSource &gas_source, size_t indention)
//{
//    std::string ind(indention, ' ');
//    return "Position: " + toString(gas_source.position, indention + 4) +
//           "\n" + ind +
//           "Color:    " + toString(gas_source.color, indention + 4) +
//           "\n" + ind +
//           "Scale:    " + std::to_string(gas_source.scale);
//}

std::string toString(const EnvironmentConfig &config, size_t indention)
{
    std::string list_ind(indention + 2, ' ');

    std::string gas_sources = "Gas sources:";
    for (const GasSource &gas_source : config.gas_sources)
        gas_sources += "\n" + list_ind + "- " +
                       toString(gas_source, indention + 4);

    std::string cad_models = "CAD models:";
    for (const CadModelColor &cad_model : config.cad_models)
        cad_models += "\n" + list_ind + "- " +
                      toString(cad_model, indention + 4);

    std::string ind(indention, ' ');
    return "Fixed frame: " + config.fixed_frame +
           "\n" + ind +
           "Occupancy grid file: " + config.occupancy_grid_file +
           "\n" + ind +
           gas_sources +
           "\n" + ind +
           cad_models;
}

EnvironmentConfig loadEnvironmentConfig(std::shared_ptr<rclcpp::Node> &ros_node,
                                        rl::Logger &log)
{
//    // List all parameters
//    std::vector<std::string> empty_prefix_list;
//    rcl_interfaces::msg::ListParametersResult parameters = ros_node->list_parameters(empty_prefix_list, 0);
//    std::cout << "Parameters:" << std::endl;
//    for (const std::string &str : parameters.names)
//        std::cout << "  " << str << std::endl;

    EnvironmentConfig config;

    std::string base_path;
    if (!getNodeParameter(ros_node, "base_path", base_path))
    {
        log.error("ROS node parameter 'base_path' not specified");
        return config;
    }

    std::string config_file = base_path + "config.yaml";
    YAML::Node yaml_config = YAML::LoadFile(config_file);

    // common part
    YAML::Node yaml_common = yaml_config["common"];

    config.fixed_frame = yaml_common["fixed_frame"].as<std::string>();
    config.occupancy_grid_file = base_path + yaml_common["occupancy_file"].as<std::string>();

    // fill in gas sources
    config.gas_sources = loadAllGasSourcesFrom(yaml_common);

    // environment part
    YAML::Node environment = yaml_config["environment"];
    //log.info() << "Read config file " << config_file << " :\n"
    //           << environment;

    // fill in CAD models
    for (const YAML::Node &item : environment["cad_models"])
    {
        CadModelColor cad_model = getCadModelColorFromYaml(item, base_path);
        config.cad_models.push_back(cad_model);
    }

    return config;
}

//visualization_msgs::msg::Marker getAsMarker(const GasSource &gas_source, int id,
//                                            const builtin_interfaces::msg::Time &stamp,
//                                            const std::string &frame_id)
//{
//    visualization_msgs::msg::Marker source;
//    source.header.stamp = stamp;
//    source.header.frame_id = frame_id;

//    source.id = id;
//    source.ns = "gas_sources";
//    source.action = visualization_msgs::msg::Marker::ADD;
//    source.type = visualization_msgs::msg::Marker::CUBE;

//    source.pose.position = gas_source.position;
//    source.pose.position.z *= 0.5;

//    source.scale = ros_type::getVector3(gas_source.scale);
//    source.scale.z = gas_source.position.z;

//    source.color = gas_source.color;

//    source.pose.orientation = ros_type::DefaultOrientation::get();

//    return source;
//}

} // namespace gaden

//void loadNodeParameters(ros::NodeHandle private_nh)
//{
//    private_nh.param<bool>("verbose", verbose, false);
//    if (verbose) ROS_INFO("[env] The data provided in the roslaunch file is:");

//    private_nh.param<bool>("wait_preprocessing", wait_preprocessing, false);
//    if (verbose) ROS_INFO("[env] wait_preprocessing: %u",wait_preprocessing);

//    private_nh.param<std::string>("fixed_frame", fixed_frame, "map");
//    if (verbose) ROS_INFO("[env] Fixed Frame: %s",fixed_frame.c_str());

//    private_nh.param<int>("number_of_sources", number_of_sources, 0);
//    if (verbose) ROS_INFO("[env] number_of_sources: %i",number_of_sources);
//    gas_source_pos_x.resize(number_of_sources);
//    gas_source_pos_y.resize(number_of_sources);
//    gas_source_pos_z.resize(number_of_sources);
//    gas_source_scale.resize(number_of_sources);
//    gas_source_color.resize(number_of_sources);
//    for(int i=0;i<number_of_sources;i++)
//    {
//        //Get location of soruce for instance (i)
//        std::string paramNameX = boost::str( boost::format("source_%i_position_x") % i);
//        std::string paramNameY = boost::str( boost::format("source_%i_position_y") % i);
//        std::string paramNameZ = boost::str( boost::format("source_%i_position_z") % i);
//        std::string scale = boost::str( boost::format("source_%i_scale") % i);
//        std::string color = boost::str( boost::format("source_%i_color") % i);

//        private_nh.param<double>(paramNameX.c_str(), gas_source_pos_x[i], 0.0);
//        private_nh.param<double>(paramNameY.c_str(), gas_source_pos_y[i], 0.0);
//        private_nh.param<double>(paramNameZ.c_str(), gas_source_pos_z[i], 0.0);
//        private_nh.param<double>(scale.c_str(), gas_source_scale[i], 0.1);
//        gas_source_color[i].resize(3);
//        private_nh.getParam(color.c_str(),gas_source_color[i]);
//        if (verbose) ROS_INFO("[env] Gas_source(%i): pos=[%0.2f %0.2f %0.2f] scale=%.2f color=[%0.2f %0.2f %0.2f]",
//                 i, gas_source_pos_x[i], gas_source_pos_y[i], gas_source_pos_z[i],
//                 gas_source_scale[i],
//                 gas_source_color[i][0],gas_source_color[i][1],gas_source_color[i][2]);
//    }

//    // CAD MODELS
//    //-------------
//    //CAD model files
//    private_nh.param<int>("number_of_CAD", number_of_CAD, 0);
//    if (verbose) ROS_INFO("[env] number_of_CAD: %i",number_of_CAD);

//    CAD_models.resize(number_of_CAD);
//    CAD_color.resize(number_of_CAD);
//    for(int i=0;i<number_of_CAD;i++)
//    {
//        //Get location of CAD file for instance (i)
//        std::string paramName = boost::str( boost::format("CAD_%i") % i);
//        std::string paramColor = boost::str( boost::format("CAD_%i_color") % i);

//        private_nh.param<std::string>(paramName.c_str(), CAD_models[i], "");
//        CAD_color[i].resize(3);
//        private_nh.getParam(paramColor.c_str(),CAD_color[i]);
//        if (verbose) ROS_INFO("[env] CAD_models(%i): %s",i, CAD_models[i].c_str());
//    }



//    //Occupancy 3D gridmap
//    //---------------------
//    private_nh.param<std::string>("occupancy3D_data", occupancy3D_data, "");
//    if (verbose) ROS_INFO("[env] Occupancy3D file location: %s",occupancy3D_data.c_str());
//}


//=========================//
// PreProcessing CallBack  //
//=========================//
//void PreprocessingCB(const std_msgs::Bool& b)
//{
//    preprocessing_done = true;
//}
