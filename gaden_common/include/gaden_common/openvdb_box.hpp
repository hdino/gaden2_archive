#ifndef GADEN_COMMON_OPENVDB_BOX_HPP_INCLUDED
#define GADEN_COMMON_OPENVDB_BOX_HPP_INCLUDED

#include <string>

#include <Eigen/Core>
#include <openvdb/Grid.h>
#include <openvdb/Types.h>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace YAML {
class Node;
}

namespace gaden::open_vdb {

class Box
{
public:
    Box(const Eigen::Vector3d &world_min,
        const Eigen::Vector3d &world_max,
        double cell_size);

    Box(const YAML::Node &yaml, double cell_size);

    virtual ~Box() {}

    inline double getCellSize() const { return cell_size_; }

    openvdb::Coord toCellCoord(const Eigen::Vector3d &p) const;

    inline bool isInside(const Eigen::Vector3d &p) const
    {
        return (p.array() >= world_min_.array()).all() &&
               (p.array() <= world_max_.array()).all();
    }

    inline bool isInside(const openvdb::Coord &cell) const
    {
        return cell_bbox_.isInside(cell);
    }

    Eigen::Vector3d getMinInWorldCoordinates() const;
    Eigen::Vector3d getMaxInWorldCoordinates() const;
    Eigen::Vector3d getSizeInWorldCoordinates() const;
    Eigen::Vector3d getCenterInWorldCoordinates() const;

    template <typename TGridPtr>
    void addToGrid(TGridPtr &grid);

    virtual std::string toString(size_t indention = 0) const;

protected:
    double cell_size_; // [length]

    Eigen::Vector3d world_min_; // [length] min in world coordinates
    Eigen::Vector3d world_max_; // [length] max in world coordinates

    openvdb::CoordBBox cell_bbox_; // [] min/max in cell coordinates
};

class BoundingBox : public Box
{
public:
    BoundingBox(const Eigen::Vector3d &world_min,
                const Eigen::Vector3d &world_max,
                double cell_size);

    BoundingBox(const openvdb::GridBase::ConstPtr &grid);

    BoundingBox(const YAML::Node &yaml);

    void addMetadataToGrid(openvdb::GridBase::Ptr grid) const;

private:
};

class ColoredBox : public Box
{
public:
    ColoredBox(const YAML::Node &yaml,
               double cell_size);

    visualization_msgs::msg::Marker
    getAsMarker(int id,
                const builtin_interfaces::msg::Time &stamp,
                const std::string &frame_id);

    std_msgs::msg::ColorRGBA color;
};

template <typename TGridPtr>
void Box::addToGrid(TGridPtr &grid)
{
    auto grid_accessor = grid->getAccessor();

    openvdb::Coord xyz;
    for (xyz.z() = cell_bbox_.min().z(); xyz.z() < cell_bbox_.max().z(); ++xyz.z())
        for (xyz.y() = cell_bbox_.min().y(); xyz.y() < cell_bbox_.max().y(); ++xyz.y())
            for (xyz.x() = cell_bbox_.min().x(); xyz.x() < cell_bbox_.max().x(); ++xyz.x())
                grid_accessor.setValue(xyz, 1);
}

//void addBoxToGrid(const Box &box, OccupancyGrid::Ptr &grid)
//{
//    double cell_size = grid->metaValue<double>("cell_size");

//    openvdb::Vec3i box_origin = box.origin / cell_size;
//    openvdb::Vec3i box_size = box.size / cell_size;
//    openvdb::Vec3i box_end = box_origin + box_size;

//    //std::cout << "Origin: " << toString(box_origin) << std::endl;
//    //std::cout << "Size: " << toString(box_size) << std::endl;
//    //std::cout << "End: " << toString(box_end) << std::endl;

//    auto grid_accessor = grid->getAccessor();

//    openvdb::Coord xyz;
//    for (xyz.z() = box_origin.z(); xyz.z() < box_end.z(); ++xyz.z())
//        for (xyz.y() = box_origin.y(); xyz.y() < box_end.y(); ++xyz.y())
//            for (xyz.x() = box_origin.x(); xyz.x() < box_end.x(); ++xyz.x())
//                grid_accessor.setValue(xyz, 1);
//}



} // namespace gaden::open_vdb

#endif // GADEN_COMMON_OPENVDB_BOX_HPP_INCLUDED
