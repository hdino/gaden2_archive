#include <gaden_common/openvdb_helper.h>
#include <gaden_filament_simulator/openvdb_environment_model.hpp>

#include <openvdb/openvdb.h>

namespace gaden {

OpenVdbEnvironmentModel::OpenVdbEnvironmentModel(OccupancyGrid::Ptr &grid, rl::Logger &parent_logger)
    : EnvironmentModel(parent_logger)
    , grid_(grid)
    , accessor_(grid_->getConstAccessor())
{
    cell_size_ = grid_->metaValue<double>("cell_size");
    //bounding_box_origin_ = grid_->metaValue<openvdb::Vec3d>("bounding_box_origin");
    //bounding_box_size_ = grid_->metaValue<openvdb::Vec3d>("bounding_box_size");

    logger.info() << "Cell size: " << cell_size_;
    //logger.info() << "Bbox origin: " << toString(bounding_box_origin_);
    //logger.info() << "Bbox size: " << toString(bounding_box_size_);
}

openvdb::Coord OpenVdbEnvironmentModel::toCoord(const Eigen::Vector3d &p) const
{
    return openvdb::Coord(p[0] / cell_size_,
                          p[1] / cell_size_,
                          p[2] / cell_size_);
}

bool OpenVdbEnvironmentModel::hasObstacleBetweenPoints(
        const Eigen::Vector3d &pa,
        const Eigen::Vector3d &pb) const
{
    openvdb::Coord coord_a = toCoord(pa);
    openvdb::Coord coord_b = toCoord(pb);

    // Check whether one of the points is outside the valid environment or is not free
    if (getOccupancy(coord_a) != Occupancy::Free) return true;
    if (getOccupancy(coord_b) != Occupancy::Free) return true;

    // Calculate normal displacement vector
    Eigen::Vector3d v_ab = pb - pa;
    double distance = v_ab.norm();
    v_ab /= distance;

    // Traverse path
    unsigned steps = std::ceil(distance / cell_size_);
        // Make sure no two iteration steps are separated by more than 1 cell

    double increment = distance / steps;

    // TODO Optimise. No need to check first and last point because already checked above?
    for (double s = 0; s <= distance; s += increment)
    {
        // Determine point in space to evaluate
        Eigen::Vector3d current_position = pa + s * v_ab;

        // Determine cell to evaluate (some cells might get evaluated twice due to the current code
        openvdb::Coord current_cell = toCoord(current_position);

        // Check if the cell is occupied
        if (static_cast<Occupancy>(accessor_.getValue(current_cell)) != Occupancy::Free)
            return true;
    }

    return false;
}

Occupancy OpenVdbEnvironmentModel::getOccupancy(const Eigen::Vector3d &p) const
{
    return getOccupancy(toCoord(p));
}

// Check if a given 3D position falls in:
// Occupancy::Free      free space
// Occupancy::Obstacle  obstacle / wall
// Occupancy::Outlet    outlet or outside the environment
Occupancy OpenVdbEnvironmentModel::getOccupancy(const openvdb::Coord &coord) const
{
    if (!bounding_box_.isInside(coord))
        return Occupancy::Outlet;

    return static_cast<Occupancy>(accessor_.getValue(coord));
}

} // namespace gaden
