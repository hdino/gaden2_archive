#ifndef GADEN_COMMON_ROS_PARAMETERS_H_INCLUDED
#define GADEN_COMMON_ROS_PARAMETERS_H_INCLUDED

#include <memory>
#include <string>

namespace rclcpp {
class Node;
}

namespace gaden {

bool getNodeParameter(std::shared_ptr<rclcpp::Node> &ros_node,
                      const char *parameter_name, std::string &target);

} // namespace gaden

#endif // GADEN_COMMON_ROS_PARAMETERS_H_INCLUDED
