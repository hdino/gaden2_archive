#ifndef GADEN_COMMON_GADEN1_OCCUPANCY_GRID_IMPORTER_H_INCLUDED
#define GADEN_COMMON_GADEN1_OCCUPANCY_GRID_IMPORTER_H_INCLUDED

#include <string>

#include "occupancy_grid_type.h"

namespace gaden {

OccupancyGrid::Ptr importOccupancyGridFromGaden1(const std::string &filename);

} // namespace gaden

#endif // GADEN_COMMON_GADEN1_OCCUPANCY_GRID_IMPORTER_H_INCLUDED
