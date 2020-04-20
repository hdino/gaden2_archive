#ifndef GADEN_SIMULATOR_OPENVDB_ENVIRONMENT_MODEL_HPP_INCLUDED
#define GADEN_SIMULATOR_OPENVDB_ENVIRONMENT_MODEL_HPP_INCLUDED

#include <Eigen/Core>

#include <gaden_common/occupancy_grid_type.h>
#include <gaden_common/openvdb_box.hpp>
#include <gaden_common/openvdb_coordinates.hpp>

#include "environment_model.hpp"

namespace gaden {

class OpenVdbEnvironmentModel : public EnvironmentModel
{
public:
    OpenVdbEnvironmentModel(OccupancyGrid::Ptr &grid,
                            rl::Logger &parent_logger);

    Eigen::Vector3d getEnvironmentMin() const;
    Eigen::Vector3d getEnvironmentMax() const;

    bool hasObstacleBetweenPoints(const Eigen::Vector3d &pa,
                                  const Eigen::Vector3d &pb) const;

    Occupancy getOccupancy(const Eigen::Vector3d &p) const;

    Occupancy getOccupancy(const openvdb::Coord &coord) const;

private:
    Occupancy getGridValueAt(const openvdb::Coord &coord) const;

    OccupancyGrid::Ptr grid_;
    OccupancyGrid::ConstAccessor accessor_;

    open_vdb::BoundingBox bounding_box_;
    open_vdb::CoordinateTransformer coord_tf_;
};

} // namespace gaden

#endif // GADEN_SIMULATOR_OPENVDB_ENVIRONMENT_MODEL_HPP_INCLUDED
