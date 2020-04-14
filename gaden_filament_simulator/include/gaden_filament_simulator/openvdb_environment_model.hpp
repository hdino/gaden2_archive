#ifndef GADEN_SIMULATOR_OPENVDB_ENVIRONMENT_MODEL_HPP_INCLUDED
#define GADEN_SIMULATOR_OPENVDB_ENVIRONMENT_MODEL_HPP_INCLUDED

#include <Eigen/Core>

#include <gaden_common/occupancy_grid_type.h>

#include "environment_model.hpp"

namespace gaden {

class OpenVdbEnvironmentModel : public EnvironmentModel
{
public:
    OpenVdbEnvironmentModel(OccupancyGrid::Ptr &grid,
                            rl::Logger &parent_logger);

    bool hasObstacleBetweenPoints(const Eigen::Vector3d &pa,
                                  const Eigen::Vector3d &pb) const;

    Occupancy getOccupancy(const Eigen::Vector3d &p) const;

    Occupancy getOccupancy(const openvdb::Coord &coord) const;

private:
    openvdb::Coord toCoord(const Eigen::Vector3d &p) const;

    OccupancyGrid::Ptr grid_;
    OccupancyGrid::ConstAccessor accessor_;
    double cell_size_;
    openvdb::CoordBBox bounding_box_;
    //Eigen::Vector3d bounding_box_origin_;
    //Eigen::Vector3d bounding_box_size_;
};

} // namespace gaden

#endif // GADEN_SIMULATOR_OPENVDB_ENVIRONMENT_MODEL_HPP_INCLUDED
