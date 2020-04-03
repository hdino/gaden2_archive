#include <gaden_common/cad_model.h>
#include <gaden_common/ros_type_helper.h>

#include <yaml-cpp/yaml.h>

namespace gaden {

std::string toString(const CadModel &cad_model, size_t indention)
{
    (void)indention;
    return "Path: " + cad_model.path;
}

std::string toString(const CadModelColor &cad_model, size_t indention)
{
    std::string ind(indention, ' ');
    return "Path:  " + cad_model.path +
           "\n" + ind +
           "Color: " + toString(cad_model.color);
}

CadModel getCadModelFromYaml(const YAML::Node &node, const std::string &base_path)
{
    CadModel cad_model;
    cad_model.path = base_path + node["cad_model_path"].as<std::string>();
    return cad_model;
}

CadModelColor getCadModelColorFromYaml(const YAML::Node &node, const std::string &base_path)
{
    CadModelColor result;
    static_cast<CadModel &>(result) = getCadModelFromYaml(node, base_path);
    result.color = ros_type::getColorFromYaml(node);
    return result;
}

visualization_msgs::msg::Marker getAsMarker(const CadModelColor &cad_model, int id,
                                            const builtin_interfaces::msg::Time &stamp,
                                            const std::string &frame_id)
{
    // CAD model in Collada (.dae) format
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = stamp;
    marker.header.frame_id = frame_id;
    marker.ns = "cad_models"; //"part_" + std::to_string(id); // TODO Does every model need its own ns?
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.mesh_resource = "file://" + cad_model.path; //CAD_models[i];
    marker.color = cad_model.color; //Color (Collada has no color)
    marker.scale = ros_type::getVector3(1.0);
    marker.pose.position = ros_type::getPoint(0.0);      //CAD models have the object pose within the file!
    marker.pose.orientation = ros_type::DefaultOrientation::get();
    return marker;
}

} // namespace gaden
