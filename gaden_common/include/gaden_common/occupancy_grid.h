#ifndef GADEN_COMMON_OCCUPANCY_GRID_H_INCLUDED
#define GADEN_COMMON_OCCUPANCY_GRID_H_INCLUDED

//#include <array>
//#include <cmath>
#include <string>

//#include <Eigen/Core>
#include <rl_logging/logging_interface.hpp>

#include "occupancy_grid_type.h"

namespace gaden {

OccupancyGrid::Ptr createGrid(double cell_size);

OccupancyGrid::Ptr loadGridFromFile(const std::string &filename,
                                    rl::Logger &log);

} // namespace gaden

#endif // GADEN_COMMON_OCCUPANCY_GRID_H_INCLUDED
