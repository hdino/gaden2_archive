#include <gaden_preprocessing/occupancy_grid.h>

#include <openvdb/openvdb.h>

namespace gaden {

OccupancyGrid::Ptr createGrid()
{
    static bool openvdb_initialised = false;
    if (!openvdb_initialised)
    {
        openvdb::initialize();
        openvdb_initialised = true;
    }

    return OccupancyGrid::create();
}

} // namespace gaden
