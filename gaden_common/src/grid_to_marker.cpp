#include <gaden_common/grid_to_marker.hpp>
#include <gaden_common/ros_type_helper.h>

namespace gaden {

visualization_msgs::msg::MarkerArray
getAsMarkerArray(OccupancyGrid::Ptr grid,
                 const builtin_interfaces::msg::Time &stamp,
                 const std::string &frame_id,
                 rl::Logger &log)
{
    auto grid_accessor = grid->getAccessor();
    double cell_size = grid->metaValue<double>("cell_size");
    log.info() << "Cell size of imported grid: " << cell_size;

    visualization_msgs::msg::MarkerArray marker_array;

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = stamp;
    marker.ns = "environment_visualization";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation = ros_type::DefaultOrientation::get();
    marker.scale = ros_type::getVector3(cell_size);

    std_msgs::msg::ColorRGBA colour_occupied = ros_type::getColor(0.8, 0.8, 0.8);
    std_msgs::msg::ColorRGBA colour_outlet = ros_type::getColor(0.9, 0.1, 0.1);

    // Iterate over all values in the grid that are "on",
    // i.e. have a value different from the background value
    for (auto it = grid->cbeginValueOn(); it.test(); ++it)
    {
        if (it.isVoxelValue()) // a voxel is a single element in the grid
        {
            openvdb::Coord coord = it.getCoord();
            auto grid_value = grid_accessor.getValue(coord);
            Occupancy typed_value = static_cast<Occupancy>(grid_value);

            openvdb::Vec3d xyz = coord.asVec3d();
            xyz += 0.5;
            xyz *= cell_size;

            marker.pose.position = ros_type::getPoint(xyz.x(), xyz.y(), xyz.z());

            if (typed_value == Occupancy::Occupied)
                marker.color = colour_occupied;
            else if (typed_value == Occupancy::Outlet)
                marker.color = colour_outlet;
            else
                continue;

            marker_array.markers.push_back(marker);
            ++marker.id;
        }
        else // a tile describes a larger area in the grid, but this is not used by GADEN
        {
            openvdb::CoordBBox bbox;
            it.getBoundingBox(bbox);
            log.warn("Grid contains a tile, but this is not supported (yet)");
        }
    }

    return marker_array;
}

} // namespace gaden
