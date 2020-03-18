#include <gaden_environment/environment.h>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// ===============================//
//              MAIN              //
// ===============================//
int main( int argc, char** argv )
{
    //Init
    rclcpp::init(argc, argv);
    //ros::init(argc, argv, "environment");
    auto node_options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
    auto ros_node = std::make_shared<rclcpp::Node>("environment", node_options);
    //ros::NodeHandle n;
    //ros::NodeHandle pnh("~");

    //Load Parameters
    gaden::loadNodeParameters(ros_node);
//    loadNodeParameters(pnh);

    // Publishers
    auto gas_source_publisher = ros_node->create_publisher<
            visualization_msgs::msg::MarkerArray>("source_visualization", 10);
    //ros::Publisher gas_source_pub = n.advertise<visualization_msgs::MarkerArray>("source_visualization", 10);
    auto environment_publisher = ros_node->create_publisher<
            visualization_msgs::msg::MarkerArray>("environment_visualization", 100);
    //ros::Publisher environmnet_pub = n.advertise<visualization_msgs::MarkerArray>("environment_visualization", 100);
    auto environment_cad_publisher = ros_node->create_publisher<
            visualization_msgs::msg::MarkerArray>("environment_cad_visualization", 100);
    //ros::Publisher environmnet_cad_pub = n.advertise<visualization_msgs::MarkerArray>("environment_cad_visualization", 100);

    // Subscribers
//    preprocessing_done = false;
//    ros::Subscriber sub = n.subscribe("preprocessing_done", 1, PreprocessingCB);


    // 1. ENVIRONMNET AS CAD MODELS
    //-------------------------------
    //visualization_msgs::MarkerArray CAD_model_markers;
    visualization_msgs::msg::MarkerArray cad_model_markers;

//    for (size_t i = 0; i < number_of_CAD; ++i)
//    {
//        // CAD model in Collada (.dae) format
//        //visualization_msgs::Marker cad;
//        visualization_msgs::msg::Marker cad;
//        cad.header.frame_id = fixed_frame;
//        //cad.header.stamp = ros::Time::now();
//        cad.header.stamp = ros_node->get_clock()->now();
//        cad.ns = "part_" + std::to_string(i);
//        cad.id = i;
////        cad.type = visualization_msgs::Marker::MESH_RESOURCE;
////        cad.action = visualization_msgs::Marker::ADD;
//        cad.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
//        cad.action = visualization_msgs::msg::Marker::ADD;
//        cad.mesh_resource = CAD_models[i];
//        cad.scale.x = 1.0;
//        cad.scale.y = 1.0;
//        cad.scale.z = 1.0;
//        cad.pose.position.x = 0.0;      //CAD models have the object pose within the file!
//        cad.pose.position.y = 0.0;
//        cad.pose.position.z = 0.0;
//        cad.pose.orientation.x = 0.0;
//        cad.pose.orientation.y = 0.0;
//        cad.pose.orientation.z = 0.0;
//        cad.pose.orientation.w = 1.0;

//        //Color (Collada has no color)
////        cad.color.r = CAD_color[i][0];
////        cad.color.g = CAD_color[i][1];
////        cad.color.b = CAD_color[i][2];
//        cad.color.a = 1.0;

//        //Add Marker to array
//        cad_model_markers.markers.push_back(cad);
//    }



    // 2. ENVIRONMNET AS Occupancy3D file
    //------------------------------------
    //Display Environment as an array of Cube markers (Rviz)
    //visualization_msgs::MarkerArray environment;
    visualization_msgs::msg::MarkerArray environment;
//    if (!occupancy3D_data.empty())
//        loadEnvironment(environment);
    size_t number_of_sources = 0;

    // 3. GAS SOURCES
    //----------------
    //Generate Gas Source Markers
    //The shape are cylinders from the floor to the given z_size.
    visualization_msgs::msg::MarkerArray gas_source_markers;
    //visualization_msgs::MarkerArray gas_source_markers;

//    for (size_t i = 0; i < number_of_sources; ++i)
//    {
//        //visualization_msgs::Marker source;
//        visualization_msgs::msg::Marker source;
//        source.header.frame_id = fixed_frame;
//        //source.header.stamp = ros::Time::now();
//        source.header.stamp = ros_node->get_clock()->now();
//        source.id = i;
//        source.ns = "gas_source_visualization";
//        source.action = visualization_msgs::msg::Marker::ADD;
////        source.action = visualization_msgs::Marker::ADD;
////        //source.type = visualization_msgs::Marker::CYLINDER;
//        source.type = visualization_msgs::msg::Marker::CUBE;
////        source.type = visualization_msgs::Marker::CUBE;

////        source.scale.x = gas_source_scale[i];
////        source.scale.y = gas_source_scale[i];
////        source.scale.z = gas_source_pos_z[i];
////        source.color.r = gas_source_color[i][0];
////        source.color.g = gas_source_color[i][1];
////        source.color.b = gas_source_color[i][2];
//        source.color.a = 1.0;
////        source.pose.position.x = gas_source_pos_x[i];
////        source.pose.position.y = gas_source_pos_y[i];
////        source.pose.position.z = gas_source_pos_z[i]/2;
//        source.pose.orientation.x = 0.0;
//        source.pose.orientation.y = 0.0;
//        source.pose.orientation.z = 1.0;
//        source.pose.orientation.w = 1.0;

//        //Add Marker to array
//        gas_source_markers.markers.push_back(source);
//    }

    // Small sleep to allow RVIZ to startup
    //ros::Duration(1.0).sleep();

    //---------------
    //      LOOP
    //---------------
    //ros::Rate r(0.3);     //Just to refresh from time to time
    rclcpp::Rate rate(0.3);
    //while (ros::ok())
    while (rclcpp::ok())
    {
        //Publish CAD Markers
        //environment_cad_publisher->publish(cad_model_markers);

        // Publish 3D Occupancy
//        if (!occupancy3D_data.empty())
//            environment_publisher->publish(environment);

        //Publish Gas Sources
//        gas_source_publisher->publish(gas_source_markers);

        //ros::spinOnce();
        rclcpp::spin_some(ros_node);
        //r.sleep();
        rate.sleep();
    }
}
