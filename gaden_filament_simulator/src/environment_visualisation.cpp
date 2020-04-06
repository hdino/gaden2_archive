#include <gaden_common/inline_environment.hpp>
#include <gaden_filament_simulator/environment_visualisation.hpp>
#include <gaden_filament_simulator/simulator_config.hpp>

#include <rl_logging/ros2_logging.hpp>
#include <yaml-cpp/yaml.h>

namespace gaden {

visualization_msgs::msg::MarkerArray getInlineEnvironmentAsMarkerArray(const YAML::Node &env,
                                                                       const builtin_interfaces::msg::Time &stamp,
                                                                       const std::string &fixed_frame,
                                                                       rl::Logger &logger)
{
    YAML::Node inline_env = env["inline_environment"];

    visualization_msgs::msg::MarkerArray marker_array;

    int id_counter = 0;
    for (const YAML::Node &object : inline_env["objects"])
    {
        std::string object_type = object["type"].as<std::string>();
        if (object_type == "box")
        {
            ColoredBox box = getColoredBox(object);
            marker_array.markers.push_back(getAsMarker(box, id_counter, stamp, fixed_frame));
        }
        else
            logger.warn() << "Unrecognised object type: " << object_type;
        ++id_counter;
    }

    return marker_array;
}

EnvironmentVisualiser::EnvironmentVisualiser(const SimulatorConfig &config,
                                             const std::string &node_name)
    : ros_node_(std::make_shared<rclcpp::Node>(node_name))
    , logger_(rl::logging::Ros2Logger::create(ros_node_->get_logger()))
    , run_ros_thread_(true)
{
    YAML::Node yaml_config = YAML::LoadFile(config.base_config_file);
    YAML::Node yaml_environment = yaml_config["environment"];

    /** ENVIRONMENT VISUALISATION **/
    if (config.visualisation.visualise_environment)
    {
        std::string environment_format = yaml_environment["format"].as<std::string>();
        if (environment_format == "inline")
            marker_environment_ = getInlineEnvironmentAsMarkerArray(yaml_environment,
                                                                    ros_node_->get_clock()->now(),
                                                                    config.visualisation.fixed_frame,
                                                                    logger_);
        else
            throw std::runtime_error("Invalid environment format: " + environment_format);

        environment_publisher_ = ros_node_->create_publisher<visualization_msgs::msg::MarkerArray>("environment_visualisation", 10);
    }

    /** BOUNDING BOX VISUALISATION **/
    if (config.visualisation.visualise_bounding_box)
    {
        bounding_box_publisher_ = ros_node_->create_publisher<visualization_msgs::msg::MarkerArray>("bounding_box_visualisation", 10);
    }

    /** GAS SOURCE VISUALISATION **/
    for (size_t i = 0; i < config.gas_sources.size(); ++i)
    {
        visualization_msgs::msg::Marker source = gaden::getAsMarker(config.gas_sources[i], i,
                                                                    ros_node_->get_clock()->now(),
                                                                    config.visualisation.fixed_frame);
        marker_gas_sources_.markers.push_back(source);
    }

    gas_sources_publisher_ = ros_node_->create_publisher<visualization_msgs::msg::MarkerArray>("gas_sources_visualisation", 10);

    ros_thread_ = std::thread(&EnvironmentVisualiser::performRosTasks, this);

    logger_.info("EnvironmentVisualiser created");
}

EnvironmentVisualiser::~EnvironmentVisualiser()
{
    run_ros_thread_ = false;
    if (ros_thread_.joinable())
        ros_thread_.join();

    logger_.info("Destructing EnvironmentVisualiser");
}

void EnvironmentVisualiser::performRosTasks()
{
    rclcpp::Rate rate(1.0/5);
    while (rclcpp::ok() && run_ros_thread_)
    {
        if (environment_publisher_)
            environment_publisher_->publish(marker_environment_);

//        if (bounding_box_publisher_)
//            bounding_box_publisher_->publish(marker_bounding_box_);

        gas_sources_publisher_->publish(marker_gas_sources_);

        rclcpp::spin_some(ros_node_);
        rate.sleep();
    }
}

} // namespace gaden
