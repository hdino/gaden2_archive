#include <gaden_preprocessing/math_helper.h>
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

    static_assert(toUnderlying(Occupancy::Free) == 0,
                  "Occupancy grid background value should be 0.");
    return OccupancyGrid::create(); // if the background value is not 0, it must be specified here
}

} // namespace gaden
