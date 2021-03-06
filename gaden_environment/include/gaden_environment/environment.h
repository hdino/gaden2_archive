#ifndef GADEN_ENVIRONMENT_ENVIRONMENT_H_INCLUDED
#define GADEN_ENVIRONMENT_ENVIRONMENT_H_INCLUDED

#include <memory>
#include <string>
#include <vector>

#include <rl_logging/logging_interface.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <gaden_common/cad_model.h>
#include <gaden_common/gas_source.hpp>

namespace rclcpp {
class Node;
}

namespace gaden {

//struct GasSource
//{
//    geometry_msgs::msg::Point position;
//    std_msgs::msg::ColorRGBA color;
//    double scale;
//};
//std::string toString(const GasSource &gas_source, size_t indention = 0);

struct EnvironmentConfig
{
    std::string fixed_frame;
    std::string occupancy_grid_file;
    std::vector<GasSource> gas_sources;
    std::vector<CadModelColor> cad_models;
};
std::string toString(const EnvironmentConfig &config, size_t indention = 0);

EnvironmentConfig loadEnvironmentConfig(std::shared_ptr<rclcpp::Node> &ros_node,
                                        rl::Logger &log);

//visualization_msgs::msg::Marker getAsMarker(const GasSource &gas_source, int id,
//                                            const builtin_interfaces::msg::Time &stamp,
//                                            const std::string &frame_id);

} // namespace gaden

//Gas Sources
//int                                 number_of_sources;
//std::vector<double>                 gas_source_pos_x;
//std::vector<double>                 gas_source_pos_y;
//std::vector<double>                 gas_source_pos_z;
//std::vector<double>                 gas_source_scale;
//std::vector< std::vector<double> >  gas_source_color;

//CAD models
//int                                 number_of_CAD;
//std::vector<std::string>            CAD_models;
//std::vector< std::vector<double> >  CAD_color;

//Environment 3D
//std::string occupancy3D_data;       //Location of the 3D Occupancy GridMap of the environment
//std::string	fixed_frame;            //Frame where to publish the markers
//int			env_cells_x;            //cells
//int 		env_cells_y;            //cells
//int 		env_cells_z;            //cells
//double      env_min_x;              //[m]
//double      env_max_x;              //[m]
//double      env_min_y;              //[m]
//double      env_max_y;              //[m]
//double      env_min_z;              //[m]
//double      env_max_z;              //[m]
//double		cell_size;              //[m]

//bool        verbose;
//bool        wait_preprocessing;
//bool        preprocessing_done;

//Methods
//void loadNodeParameters(ros::NodeHandle);
//void loadEnvironment(visualization_msgs::MarkerArray &env_marker);

#endif // GADEN_ENVIRONMENT_ENVIRONMENT_H_INCLUDED
