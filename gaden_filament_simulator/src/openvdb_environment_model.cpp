#include <gaden_common/inline_environment.hpp>
#include <gaden_common/openvdb_helper.h>
#include <gaden_filament_simulator/openvdb_environment_model.hpp>

#include <openvdb/openvdb.h>

namespace gaden {

OpenVdbEnvironmentModel::OpenVdbEnvironmentModel(OccupancyGrid::Ptr &grid, rl::Logger &parent_logger)
    : EnvironmentModel(parent_logger)
    , grid_(grid)
    , accessor_(grid_->getConstAccessor())
    , bounding_box_(grid)
    , coord_tf_(bounding_box_.getCellSize())
{
    logger.info() << "Bounding box: " << bounding_box_.toString();
}

bool OpenVdbEnvironmentModel::hasObstacleBetweenPoints(
        const Eigen::Vector3d &pa,
        const Eigen::Vector3d &pb) const
{
    openvdb::Coord coord_a = coord_tf_.toCellCoordinates(pa);
    openvdb::Coord coord_b = coord_tf_.toCellCoordinates(pb);

    // Check whether one of the points is outside the valid environment or is not free
    if (getOccupancy(coord_a) != Occupancy::Free) return true;
    if (getOccupancy(coord_b) != Occupancy::Free) return true;

    // Calculate normal displacement vector
    Eigen::Vector3d v_ab = pb - pa;
    double distance = v_ab.norm();
    v_ab /= distance;

    // Traverse path
    unsigned steps = std::ceil(distance / coord_tf_.getCellSize());
        // Make sure no two iteration steps are separated by more than 1 cell

    double increment = distance / steps;

    // TODO Optimise. No need to check first and last point because already checked above?
    for (double s = 0; s <= distance; s += increment)
    {
        // Determine point in space to evaluate
        Eigen::Vector3d current_position = pa + s * v_ab;

        // Determine cell to evaluate (some cells might get evaluated twice due to the current code
        openvdb::Coord current_cell = coord_tf_.toCellCoordinates(current_position);

        // Check if the cell is occupied
        if (static_cast<Occupancy>(accessor_.getValue(current_cell)) != Occupancy::Free)
            return true;
    }

    return false;
}

Occupancy OpenVdbEnvironmentModel::getOccupancy(const Eigen::Vector3d &p) const
{
    if (!bounding_box_.isInside(p))
        return Occupancy::Outlet;

    return getGridValueAt(coord_tf_.toCellCoordinates(p));
}

Occupancy OpenVdbEnvironmentModel::getOccupancy(const openvdb::Coord &coord) const
{
    if (!bounding_box_.isInside(coord))
        return Occupancy::Outlet;

    return getGridValueAt(coord);
}

Occupancy OpenVdbEnvironmentModel::getGridValueAt(const openvdb::Coord &coord) const
{
    return static_cast<Occupancy>(accessor_.getValue(coord));
}

} // namespace gaden
