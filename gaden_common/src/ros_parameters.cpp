#include <gaden_common/ros_parameters.h>

#include <rclcpp/node.hpp>

namespace gaden {

bool getNodeParameter(std::shared_ptr<rclcpp::Node> &ros_node, const char *parameter_name, std::string &target)
{
    rclcpp::Parameter parameter;
    if (!ros_node->get_parameter(parameter_name, parameter))
    {
        RCLCPP_ERROR_STREAM(ros_node->get_logger(),
                            "Parameter " << parameter_name << " not defined.");
        return false;
    }
    if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING)
    {
        RCLCPP_ERROR_STREAM(ros_node->get_logger(),
                            "Parameter " << parameter_name << " type mismatch.");
        return false;
    }
    target = parameter.as_string();
    return true;
}

} // namespace gaden
