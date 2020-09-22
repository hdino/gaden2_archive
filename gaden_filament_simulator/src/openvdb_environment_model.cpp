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

Eigen::Vector3d OpenVdbEnvironmentModel::getEnvironmentMin() const
{
    return bounding_box_.getMinInWorldCoordinates();
}

Eigen::Vector3d OpenVdbEnvironmentModel::getEnvironmentMax() const
{
    return bounding_box_.getMaxInWorldCoordinates();
}

Occupancy OpenVdbEnvironmentModel::getOccupancy(const Eigen::Vector3d &p) const
{
    if (!bounding_box_.isInside(p))
        return Occupancy::OutOfWorld;

    return getGridValueAt(coord_tf_.toCellCoordinates(p));
}

Occupancy OpenVdbEnvironmentModel::getOccupancy(const openvdb::Coord &coord) const
{
    if (!bounding_box_.isInside(coord))
        return Occupancy::OutOfWorld;

    return getGridValueAt(coord);
}

Occupancy OpenVdbEnvironmentModel::getGridValueAt(const openvdb::Coord &coord) const
{
    return static_cast<Occupancy>(accessor_.getValue(coord));
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
    // TODO: This is not reliable! The path might traverse some cells on a length << cell_size

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

CollisionTestResult OpenVdbEnvironmentModel::getCollisionDistance(
        const Eigen::Vector3d &start_point,
        const Eigen::Vector3d &direction,
        double max_distance) const
{
    // TODO Improve, this approach is quick and dirty

    double check_distance = 0.01; // [m], check every cm
    Eigen::Vector3d direction_normalised = direction.normalized();

    // set last_cell to invalid cell
    openvdb::Coord last_cell = bounding_box_.getMinInCellCoordinates() - openvdb::Coord(1,1,1);

    double distance = 0;
    while (distance <= max_distance)
    {
        Eigen::Vector3d p = start_point + distance * direction_normalised;
        openvdb::Coord cell = coord_tf_.toCellCoordinates(p);

        if (cell != last_cell)
        {
            last_cell = cell;
            Occupancy cell_occupancy = getOccupancy(cell);
            if (cell_occupancy != Occupancy::Free) // that is Occupied, Outlet or OutOfWorld
                return CollisionTestResult(cell_occupancy, distance);
        }

        distance += check_distance;
    }

    return CollisionTestResult(Occupancy::Free, distance);
}

} // namespace gaden
