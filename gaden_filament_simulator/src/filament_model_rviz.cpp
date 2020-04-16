#include <gaden_common/ros_type_helper.h>
#include <gaden_filament_simulator/filament_model.hpp>
#include <gaden_filament_simulator/filament_model_rviz.hpp>

namespace gaden {

FilamentModelRvizVisualisation::FilamentModelRvizVisualisation(
        std::shared_ptr<rclcpp::Node> ros_node,
        std::shared_ptr<FilamentModel> filament_model,
        const std::string &rviz_frame_id,
        double scale,
        rl::Logger &logger)
    : logger_(logger.getChild("FilamentModelRviz"))
    , ros_node_(ros_node)
    , filament_model_(filament_model)
    , publisher_(ros_node_->create_publisher<visualization_msgs::msg::Marker>("filament_visualisation", 1))
{
    marker_.header.frame_id = rviz_frame_id;
    marker_.ns = "filaments";
    marker_.action = visualization_msgs::msg::Marker::ADD;
    marker_.id = 0;
    marker_.type = visualization_msgs::msg::Marker::POINTS;
    marker_.pose.orientation = ros_type::DefaultOrientation::get();
    marker_.scale = ros_type::getVector3(scale);
    marker_.color.a = 1.0;
}

void FilamentModelRvizVisualisation::publish()
{
    //static std_msgs::msg::ColorRGBA colour_blue = ros_type::getColor(0, 0, 1);
    static std_msgs::msg::ColorRGBA colour_red = ros_type::getColor(1, 0, 0);

    marker_.points.clear();
    marker_.colors.clear();
    marker_.header.stamp = ros_node_->get_clock()->now();

    for (const Filament &filament : filament_model_->getFilaments())
    {
        marker_.points.push_back(ros_type::getPointFrom(filament.position));
        marker_.colors.push_back(colour_red);
    }

    logger_.info() << "Points: " << marker_.points.size() << " Colors: " << marker_.colors.size();

    publisher_->publish(marker_);
}

} // namespace gaden
