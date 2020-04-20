#include <gaden_common/eigen_helper.hpp>
#include <gaden_common/grid_helper.hpp>
#include <gaden_common/openvdb_box.hpp>
#include <gaden_common/openvdb_helper.h>
#include <gaden_common/ros_type_helper.h>

#include <yaml-cpp/yaml.h>

namespace gaden::open_vdb {

/** ******************** BOX ******************** **/

Box::Box(const Eigen::Vector3d &world_min,
         const Eigen::Vector3d &world_max,
         double cell_size)
    : cell_size_(cell_size)
    , world_min_(world_min), world_max_(world_max)
    , cell_bbox_(toCellCoord(world_min), toCellCoord(world_max))
{}

Box::Box(const YAML::Node &yaml, double cell_size)
    : Box(eigen_helper::getVector3dFromYaml(yaml["min"]),
          eigen_helper::getVector3dFromYaml(yaml["max"]),
          cell_size)
{}

openvdb::Coord Box::toCellCoord(const Eigen::Vector3d &p) const
{
    return grid_helper::getCellCoordinates(p, cell_size_);
}

Eigen::Vector3d Box::getMinInWorldCoordinates() const
{
    return world_min_;
}

Eigen::Vector3d Box::getMaxInWorldCoordinates() const
{
    return world_max_;
}

Eigen::Vector3d Box::getSizeInWorldCoordinates() const
{
    return world_max_ - world_min_;
}

Eigen::Vector3d Box::getCenterInWorldCoordinates() const
{
    return 0.5 * (world_min_ + world_max_);
}

std::string Box::toString(size_t indention) const
{
    return  "min = " + gaden::toString(world_min_, indention) +
           " max = " + gaden::toString(world_max_, indention) +
           " cell_size = " + std::to_string(cell_size_);
}

/** ******************** BOUNDING BOX ******************** **/

BoundingBox::BoundingBox(const Eigen::Vector3d &world_min,
                         const Eigen::Vector3d &world_max,
                         double cell_size)
    : Box(world_min, world_max, cell_size)
{}

BoundingBox::BoundingBox(const openvdb::GridBase::ConstPtr &grid)
    : Box(openvdb_helper::toEigen(
              grid->metaValue<openvdb::Vec3d>("bounding_box_min")),
          openvdb_helper::toEigen(
              grid->metaValue<openvdb::Vec3d>("bounding_box_max")),
          grid->metaValue<double>("cell_size"))
{}

BoundingBox::BoundingBox(const YAML::Node &yaml)
    : Box(yaml["bounding_box"],
          yaml["cell_size"].as<double>())
{}

void BoundingBox::addMetadataToGrid(openvdb::GridBase::Ptr grid) const
{
    grid->insertMeta("bounding_box_min",
                     openvdb::Vec3DMetadata(
                         openvdb_helper::fromEigen(world_min_)));
    grid->insertMeta("bounding_box_max",
                     openvdb::Vec3DMetadata(
                         openvdb_helper::fromEigen(world_max_)));
}

/** ******************** COLORED BOX ******************** **/

ColoredBox::ColoredBox(const YAML::Node &yaml,
                       double cell_size)
    : Box(yaml, cell_size)
    , color(ros_type::getColorFromYaml(yaml))
{}

visualization_msgs::msg::Marker
ColoredBox::getAsMarker(int id,
                        const builtin_interfaces::msg::Time &stamp,
                        const std::string &frame_id)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = stamp;
    marker.ns = "box";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position = ros_type::getPointFrom(getCenterInWorldCoordinates());
    marker.pose.orientation = ros_type::DefaultOrientation::get();

    marker.scale = ros_type::getVector3From(getSizeInWorldCoordinates());

    marker.color = color;

    return marker;
}

} // namespace gaden::open_vdb
